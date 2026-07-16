
#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

struct Buffer {
    void* start = nullptr;
    size_t length = 0;
};

struct Stats {
    uint64_t valid = 0;
    uint64_t error_buf = 0;
    uint64_t empty = 0;
    uint64_t dq_errors = 0;
    uint64_t q_errors = 0;
    uint64_t seq_drops = 0;
    uint64_t size_mismatch = 0;
    uint64_t startup_bad = 0;
    double elapsed = 0.0;
    double effective_fps = 0.0;
    double stream_fps = 0.0;
    double first_ms = -1.0;
    bool fatal = false;
    std::string reason;
};

class Camera {
public:
    Camera(std::string dev, int w, int h, int fps, int nbuf)
        : dev_(std::move(dev)), req_w_(w), req_h_(h), req_fps_(fps), req_nbuf_(nbuf) {}

    ~Camera() { stop(); cleanup(); }

    bool init() {
        fd_ = ::open(dev_.c_str(), O_RDWR | O_CLOEXEC);
        if (fd_ < 0) return fail("open: " + err());

        v4l2_capability cap{};
        if (xioctl(VIDIOC_QUERYCAP, &cap) < 0) return fail("QUERYCAP: " + err());

        uint32_t caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS)
                      ? cap.device_caps : cap.capabilities;
        if (!(caps & V4L2_CAP_VIDEO_CAPTURE)) return fail("not VIDEO_CAPTURE");
        if (!(caps & V4L2_CAP_STREAMING)) return fail("no STREAMING support");

        bus_ = reinterpret_cast<const char*>(cap.bus_info);

        v4l2_format fmt{};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = req_w_;
        fmt.fmt.pix.height = req_h_;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        if (xioctl(VIDIOC_S_FMT, &fmt) < 0) return fail("S_FMT: " + err());

        width_ = fmt.fmt.pix.width;
        height_ = fmt.fmt.pix.height;
        pixfmt_ = fmt.fmt.pix.pixelformat;
        size_image_ = fmt.fmt.pix.sizeimage;

        if (width_ != static_cast<uint32_t>(req_w_) ||
            height_ != static_cast<uint32_t>(req_h_) ||
            pixfmt_ != V4L2_PIX_FMT_UYVY) {
            return fail("unexpected negotiated format");
        }

        setFpsBestEffort();

        v4l2_requestbuffers req{};
        req.count = req_nbuf_;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (xioctl(VIDIOC_REQBUFS, &req) < 0) return fail("REQBUFS: " + err());
        if (req.count < 2) return fail("too few mmap buffers");

        bufs_.resize(req.count);

        for (uint32_t i = 0; i < req.count; ++i) {
            v4l2_buffer b{};
            b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            b.memory = V4L2_MEMORY_MMAP;
            b.index = i;

            if (xioctl(VIDIOC_QUERYBUF, &b) < 0)
                return fail("QUERYBUF[" + std::to_string(i) + "]: " + err());

            void* p = ::mmap(nullptr, b.length, PROT_READ | PROT_WRITE,
                             MAP_SHARED, fd_, b.m.offset);
            if (p == MAP_FAILED)
                return fail("mmap[" + std::to_string(i) + "]: " + err());

            bufs_[i] = {p, b.length};
        }

        for (uint32_t i = 0; i < bufs_.size(); ++i) {
            v4l2_buffer b{};
            b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            b.memory = V4L2_MEMORY_MMAP;
            b.index = i;
            if (xioctl(VIDIOC_QBUF, &b) < 0)
                return fail("initial QBUF[" + std::to_string(i) + "]: " + err());
        }

        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(VIDIOC_STREAMON, &type) < 0)
            return fail("STREAMON: " + err());

        streaming_.store(true);

        std::cout << "[READY] " << dev_
                  << " | " << width_ << "x" << height_
                  << " UYVY"
                  << " | buffers=" << bufs_.size()
                  << " | bus=" << bus_ << "\n";
        return true;
    }

    void startThread() {
        stop_requested_.store(false);
        th_ = std::thread(&Camera::captureLoop, this);
    }

    void beginMeasure() {
        std::lock_guard<std::mutex> lk(mtx_);
        stats_ = Stats{};
        stats_.startup_bad = startup_bad_.load();
        have_prev_seq_ = false;
        have_first_ = false;
        measure_start_ = Clock::now();
        measuring_.store(true);
    }

    void endMeasure() {
        measuring_.store(false);
        std::lock_guard<std::mutex> lk(mtx_);
        auto end = Clock::now();
        stats_.elapsed = std::chrono::duration<double>(end - measure_start_).count();

        if (stats_.elapsed > 0.0)
            stats_.effective_fps = static_cast<double>(stats_.valid) / stats_.elapsed;

        if (stats_.valid >= 2 && have_first_) {
            double span = std::chrono::duration<double>(last_valid_ - first_valid_).count();
            if (span > 0.0)
                stats_.stream_fps = static_cast<double>(stats_.valid - 1) / span;
        }

        stats_.fatal = fatal_.load();
        stats_.reason = reason_;
    }

    void requestStop() { stop_requested_.store(true); }

    void streamOff() {
        bool expected = true;
        if (!streaming_.compare_exchange_strong(expected, false)) return;
        if (fd_ >= 0) {
            v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            ::ioctl(fd_, VIDIOC_STREAMOFF, &type);
        }
    }

    void join() {
        if (th_.joinable()) th_.join();
    }

    void stop() {
        requestStop();
        streamOff();
        join();
    }

    Stats stats() const {
        std::lock_guard<std::mutex> lk(mtx_);
        Stats s = stats_;
        s.startup_bad = startup_bad_.load();
        s.fatal = fatal_.load();
        s.reason = reason_;
        return s;
    }

    const std::string& dev() const { return dev_; }
    int targetFps() const { return req_fps_; }

private:
    using Clock = std::chrono::steady_clock;

    std::string dev_;
    int req_w_, req_h_, req_fps_, req_nbuf_;
    int fd_ = -1;
    std::string bus_;

    uint32_t width_ = 0, height_ = 0, pixfmt_ = 0, size_image_ = 0;
    std::vector<Buffer> bufs_;
    std::thread th_;

    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> streaming_{false};
    std::atomic<bool> measuring_{false};
    std::atomic<bool> fatal_{false};
    std::atomic<uint64_t> startup_bad_{0};

    mutable std::mutex mtx_;
    Stats stats_;
    std::string reason_;

    Clock::time_point measure_start_{};
    Clock::time_point first_valid_{};
    Clock::time_point last_valid_{};
    bool have_first_ = false;
    bool have_prev_seq_ = false;
    uint32_t prev_seq_ = 0;

    static std::string err() { return std::strerror(errno); }

    int xioctl(unsigned long req, void* arg) {
        int r;
        do { r = ::ioctl(fd_, req, arg); }
        while (r < 0 && errno == EINTR);
        return r;
    }

    bool fail(const std::string& s) {
        fatal_.store(true);
        reason_ = s;
        return false;
    }

    void setFpsBestEffort() {
        v4l2_streamparm p{};
        p.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(VIDIOC_G_PARM, &p) < 0) return;
        if (!(p.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)) return;

        p.parm.capture.timeperframe.numerator = 1;
        p.parm.capture.timeperframe.denominator = req_fps_;
        if (xioctl(VIDIOC_S_PARM, &p) < 0)
            std::cerr << "[WARN] " << dev_ << " S_PARM failed: " << err() << "\n";
    }

    void captureLoop() {
        while (!stop_requested_.load()) {
            v4l2_buffer b{};
            b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            b.memory = V4L2_MEMORY_MMAP;

            if (xioctl(VIDIOC_DQBUF, &b) < 0) {
                if (stop_requested_.load() || !streaming_.load()) break;

                if (measuring_.load()) {
                    std::lock_guard<std::mutex> lk(mtx_);
                    stats_.dq_errors++;
                }

                if (errno == EIO) continue;
                fail("DQBUF: " + err());
                break;
            }

            const bool is_err = (b.flags & V4L2_BUF_FLAG_ERROR) != 0;
            const bool is_empty = b.bytesused == 0;
            const bool valid = !is_err && !is_empty;

            if (measuring_.load()) {
                auto now = Clock::now();
                std::lock_guard<std::mutex> lk(mtx_);

                if (is_err) stats_.error_buf++;
                if (is_empty) stats_.empty++;

                if (valid) {
                    stats_.valid++;

                    if (size_image_ > 0 && b.bytesused != size_image_)
                        stats_.size_mismatch++;

                    if (!have_first_) {
                        have_first_ = true;
                        first_valid_ = now;
                        last_valid_ = now;
                        stats_.first_ms =
                            std::chrono::duration<double, std::milli>(
                                now - measure_start_).count();
                    } else {
                        last_valid_ = now;
                    }

                    if (have_prev_seq_) {
                        uint32_t delta = b.sequence - prev_seq_;
                        if (delta > 1 && delta < 0x80000000U)
                            stats_.seq_drops += static_cast<uint64_t>(delta - 1);
                    }
                    prev_seq_ = b.sequence;
                    have_prev_seq_ = true;
                }
            } else if (is_err || is_empty) {
                startup_bad_.fetch_add(1);
            }

            if (stop_requested_.load() || !streaming_.load()) break;

            if (xioctl(VIDIOC_QBUF, &b) < 0) {
                if (stop_requested_.load() || !streaming_.load()) break;

                if (measuring_.load()) {
                    std::lock_guard<std::mutex> lk(mtx_);
                    stats_.q_errors++;
                }

                fail("QBUF: " + err());
                break;
            }
        }
    }

    void cleanup() {
        for (auto& b : bufs_) {
            if (b.start && b.start != MAP_FAILED)
                ::munmap(b.start, b.length);
        }
        bufs_.clear();

        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }
};

struct Config {
    int count = 1;
    int duration = 30;
    int warmup = 2;
    int width = 1920;
    int height = 1280;
    int fps = 30;
    int buffers = 8;
    std::string csv;
    std::vector<std::string> devices = {
        "/dev/video0", "/dev/video2", "/dev/video4",
        "/dev/video6", "/dev/video8", "/dev/video10"
    };
};

static bool toInt(const std::string& s, int& v) {
    try {
        size_t n = 0;
        int x = std::stoi(s, &n);
        if (n != s.size()) return false;
        v = x;
        return true;
    } catch (...) {
        return false;
    }
}

static std::vector<std::string> split(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ','))
        if (!item.empty()) out.push_back(item);
    return out;
}

static void usage(const char* p) {
    std::cout
        << "Usage: " << p << " [options]\n"
        << "  --count N\n"
        << "  --duration SEC\n"
        << "  --warmup SEC\n"
        << "  --width PX\n"
        << "  --height PX\n"
        << "  --fps FPS\n"
        << "  --buffers N\n"
        << "  --devices /dev/video0,/dev/video2,...\n"
        << "  --csv FILE\n";
}

static bool parseArgs(int argc, char** argv, Config& c) {
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto val = [&]() -> std::string {
            if (i + 1 >= argc) return {};
            return argv[++i];
        };

        std::string v;
        if (a == "--count") { v = val(); if (!toInt(v, c.count)) return false; }
        else if (a == "--duration") { v = val(); if (!toInt(v, c.duration)) return false; }
        else if (a == "--warmup") { v = val(); if (!toInt(v, c.warmup)) return false; }
        else if (a == "--width") { v = val(); if (!toInt(v, c.width)) return false; }
        else if (a == "--height") { v = val(); if (!toInt(v, c.height)) return false; }
        else if (a == "--fps") { v = val(); if (!toInt(v, c.fps)) return false; }
        else if (a == "--buffers") { v = val(); if (!toInt(v, c.buffers)) return false; }
        else if (a == "--devices") { v = val(); c.devices = split(v); }
        else if (a == "--csv") { c.csv = val(); }
        else if (a == "--help") { usage(argv[0]); return false; }
        else { std::cerr << "Unknown option: " << a << "\n"; return false; }
    }

    if (c.count < 1 || c.count > static_cast<int>(c.devices.size()) ||
        c.duration < 1 || c.warmup < 0 || c.width < 1 || c.height < 1 ||
        c.fps < 1 || c.buffers < 2) {
        return false;
    }
    return true;
}

static std::string status(const Stats& s, int target_fps) {
    if (s.fatal || s.valid == 0) return "FAIL";
    if (s.effective_fps >= target_fps * 0.90 &&
        s.dq_errors == 0 && s.q_errors == 0)
        return "PASS";
    return "WARN";
}

static void printSummary(const std::vector<std::unique_ptr<Camera>>& cams) {
    std::cout << "\n"
              << "====================================================================================================\n"
              << " Camera Stress Test Summary\n"
              << "====================================================================================================\n";

    std::cout << std::left
              << std::setw(14) << "Device"
              << std::setw(9) << "Status"
              << std::setw(10) << "Valid"
              << std::setw(9) << "ErrBuf"
              << std::setw(9) << "Empty"
              << std::setw(10) << "SeqDrop"
              << std::setw(10) << "EffFPS"
              << std::setw(10) << "StrFPS"
              << std::setw(11) << "First(ms)"
              << std::setw(11) << "StartBad"
              << "\n";

    for (const auto& c : cams) {
        Stats s = c->stats();
        std::cout << std::left
                  << std::setw(14) << c->dev()
                  << std::setw(9) << status(s, c->targetFps())
                  << std::setw(10) << s.valid
                  << std::setw(9) << s.error_buf
                  << std::setw(9) << s.empty
                  << std::setw(10) << s.seq_drops
                  << std::setw(10) << std::fixed << std::setprecision(2) << s.effective_fps
                  << std::setw(10) << s.stream_fps
                  << std::setw(11) << std::setprecision(1) << s.first_ms
                  << std::setw(11) << s.startup_bad
                  << "\n";

        if (s.fatal) std::cout << "  -> FATAL: " << s.reason << "\n";
    }

    std::cout << "====================================================================================================\n";
}

static bool writeCsv(const std::string& path,
                     const std::vector<std::unique_ptr<Camera>>& cams) {
    std::ofstream out(path);
    if (!out) return false;

    out << "device,status,valid,error_buf,empty,dq_errors,q_errors,"
           "seq_drops,size_mismatch,startup_bad,elapsed,effective_fps,"
           "stream_fps,first_ms,reason\n";

    for (const auto& c : cams) {
        Stats s = c->stats();
        std::string reason = s.reason;
        for (char& ch : reason) if (ch == ',') ch = ';';

        out << c->dev() << ","
            << status(s, c->targetFps()) << ","
            << s.valid << ","
            << s.error_buf << ","
            << s.empty << ","
            << s.dq_errors << ","
            << s.q_errors << ","
            << s.seq_drops << ","
            << s.size_mismatch << ","
            << s.startup_bad << ","
            << s.elapsed << ","
            << s.effective_fps << ","
            << s.stream_fps << ","
            << s.first_ms << ","
            << reason << "\n";
    }
    return true;
}

int main(int argc, char** argv) {
    Config cfg;
    if (!parseArgs(argc, argv, cfg)) {
        usage(argv[0]);
        return 1;
    }

    std::cout
        << "============================================================\n"
        << " Prius Multi-Camera V4L2 Stress Test\n"
        << "============================================================\n"
        << " Cameras : " << cfg.count << "\n"
        << " Mode    : " << cfg.width << "x" << cfg.height
        << " UYVY @ " << cfg.fps << " FPS\n"
        << " Buffers : " << cfg.buffers << "\n"
        << " Warm-up : " << cfg.warmup << " sec\n"
        << " Measure : " << cfg.duration << " sec\n"
        << " I/O     : mmap + blocking VIDIOC_DQBUF\n"
        << "============================================================\n";

    std::vector<std::unique_ptr<Camera>> cams;

    for (int i = 0; i < cfg.count; ++i) {
        auto cam = std::make_unique<Camera>(
            cfg.devices[i], cfg.width, cfg.height, cfg.fps, cfg.buffers);

        if (!cam->init()) {
            std::cerr << "[INIT FAIL] " << cfg.devices[i]
                      << " | " << cam->stats().reason << "\n";
            cams.push_back(std::move(cam));
            for (auto& c : cams) c->stop();
            return 2;
        }

        cam->startThread();
        cams.push_back(std::move(cam));
    }

    if (cfg.warmup > 0) {
        std::cout << "[WARMUP] all streams active for "
                  << cfg.warmup << " sec\n";
        std::this_thread::sleep_for(std::chrono::seconds(cfg.warmup));
    }

    for (auto& c : cams) c->beginMeasure();
    std::cout << "[MEASURE] synchronized measurement started\n";

    for (int sec = 1; sec <= cfg.duration; ++sec) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        if (sec % 5 == 0 || sec == cfg.duration) {
            std::cout << "[LIVE] " << sec << "/" << cfg.duration << " sec";
            for (const auto& c : cams)
                std::cout << " | " << c->dev() << "=" << c->stats().valid;
            std::cout << "\n";
        }
    }

    for (auto& c : cams) c->endMeasure();

    std::cout << "[STOP] stopping streams\n";
    for (auto& c : cams) c->requestStop();
    for (auto& c : cams) c->streamOff();
    for (auto& c : cams) c->join();

    printSummary(cams);

    if (!cfg.csv.empty()) {
        if (writeCsv(cfg.csv, cams))
            std::cout << "[CSV] " << cfg.csv << "\n";
        else
            std::cerr << "[WARN] cannot write CSV " << cfg.csv << "\n";
    }

    bool all_pass = true;
    for (const auto& c : cams) {
        Stats s = c->stats();
        if (status(s, c->targetFps()) != "PASS") all_pass = false;
    }

    return all_pass ? 0 : 3;
}
