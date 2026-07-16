#include <linux/videodev2.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <ctime>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using SteadyClock = std::chrono::steady_clock;
using SystemClock = std::chrono::system_clock;

struct MmapBuffer
{
    void * start = nullptr;
    size_t length = 0;
};

struct FrameSnapshot
{
    std::vector<uint8_t> uyvy;
    uint32_t sequence = 0;
    SteadyClock::time_point capture_time{};
    bool valid = false;
};

struct CameraConfig
{
    std::string device;
    std::string name;
    int width = 1920;
    int height = 1280;
    int fps = 30;
    int buffer_count = 8;
};

class V4L2Camera
{
public:
    explicit V4L2Camera(CameraConfig config)
        : config_(std::move(config))
    {
    }

    ~V4L2Camera()
    {
        stop();
    }

    bool start()
    {
        if (running_.load())
        {
            return true;
        }

        if (!openAndStart())
        {
            return false;
        }

        stop_requested_.store(false);
        running_.store(true);
        worker_ = std::thread(&V4L2Camera::captureLoop, this);
        return true;
    }

    void stop()
    {
        stop_requested_.store(true);

        if (fd_ >= 0 && streaming_.load())
        {
            v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            ::ioctl(fd_, VIDIOC_STREAMOFF, &type);
            streaming_.store(false);
        }

        if (worker_.joinable())
        {
            worker_.join();
        }

        running_.store(false);
        cleanup();
    }

    bool getLatest(FrameSnapshot & out) const
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        if (!latest_.valid)
        {
            return false;
        }

        out = latest_;
        return true;
    }

    const CameraConfig & config() const
    {
        return config_;
    }

private:
    CameraConfig config_;

    int fd_ = -1;
    std::vector<MmapBuffer> buffers_;

    std::thread worker_;
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> running_{false};
    std::atomic<bool> streaming_{false};

    mutable std::mutex frame_mutex_;
    FrameSnapshot latest_;

    static std::string errnoText()
    {
        return std::strerror(errno);
    }

    int xioctl(unsigned long request, void * arg)
    {
        int result;
        do
        {
            result = ::ioctl(fd_, request, arg);
        }
        while (result < 0 && errno == EINTR);

        return result;
    }

    bool openAndStart()
    {
        fd_ = ::open(config_.device.c_str(), O_RDWR | O_CLOEXEC);
        if (fd_ < 0)
        {
            std::cerr << "[FAIL] " << config_.device
                      << " open: " << errnoText() << "\n";
            return false;
        }

        v4l2_capability cap{};
        if (xioctl(VIDIOC_QUERYCAP, &cap) < 0)
        {
            std::cerr << "[FAIL] " << config_.device
                      << " QUERYCAP: " << errnoText() << "\n";
            return false;
        }

        uint32_t caps = cap.capabilities;
        if (cap.capabilities & V4L2_CAP_DEVICE_CAPS)
        {
            caps = cap.device_caps;
        }

        if (!(caps & V4L2_CAP_VIDEO_CAPTURE) ||
            !(caps & V4L2_CAP_STREAMING))
        {
            std::cerr << "[FAIL] " << config_.device
                      << " does not support capture + streaming\n";
            return false;
        }

        v4l2_format fmt{};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = static_cast<uint32_t>(config_.width);
        fmt.fmt.pix.height = static_cast<uint32_t>(config_.height);
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        if (xioctl(VIDIOC_S_FMT, &fmt) < 0)
        {
            std::cerr << "[FAIL] " << config_.device
                      << " S_FMT: " << errnoText() << "\n";
            return false;
        }

        if (fmt.fmt.pix.width != static_cast<uint32_t>(config_.width) ||
            fmt.fmt.pix.height != static_cast<uint32_t>(config_.height) ||
            fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_UYVY)
        {
            std::cerr << "[FAIL] " << config_.device
                      << " negotiated unexpected format "
                      << fmt.fmt.pix.width << "x" << fmt.fmt.pix.height
                      << "\n";
            return false;
        }

        v4l2_streamparm parm{};
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (xioctl(VIDIOC_G_PARM, &parm) == 0 &&
            (parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME))
        {
            parm.parm.capture.timeperframe.numerator = 1;
            parm.parm.capture.timeperframe.denominator =
                static_cast<uint32_t>(config_.fps);

            if (xioctl(VIDIOC_S_PARM, &parm) < 0)
            {
                std::cerr << "[WARN] " << config_.device
                          << " S_PARM: " << errnoText() << "\n";
            }
        }

        v4l2_requestbuffers req{};
        req.count = static_cast<uint32_t>(config_.buffer_count);
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (xioctl(VIDIOC_REQBUFS, &req) < 0 || req.count < 2)
        {
            std::cerr << "[FAIL] " << config_.device
                      << " REQBUFS failed\n";
            return false;
        }

        buffers_.resize(req.count);

        for (uint32_t i = 0; i < req.count; ++i)
        {
            v4l2_buffer buf{};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (xioctl(VIDIOC_QUERYBUF, &buf) < 0)
            {
                std::cerr << "[FAIL] " << config_.device
                          << " QUERYBUF index=" << i << "\n";
                return false;
            }

            void * ptr = ::mmap(
                nullptr,
                buf.length,
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                fd_,
                buf.m.offset);

            if (ptr == MAP_FAILED)
            {
                std::cerr << "[FAIL] " << config_.device
                          << " mmap index=" << i << "\n";
                return false;
            }

            buffers_[i].start = ptr;
            buffers_[i].length = buf.length;
        }

        for (uint32_t i = 0; i < req.count; ++i)
        {
            v4l2_buffer buf{};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (xioctl(VIDIOC_QBUF, &buf) < 0)
            {
                std::cerr << "[FAIL] " << config_.device
                          << " initial QBUF index=" << i << "\n";
                return false;
            }
        }

        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(VIDIOC_STREAMON, &type) < 0)
        {
            std::cerr << "[FAIL] " << config_.device
                      << " STREAMON: " << errnoText() << "\n";
            return false;
        }

        streaming_.store(true);

        std::cout << "[READY] " << config_.name
                  << " " << config_.device
                  << " " << config_.width << "x" << config_.height
                  << " UYVY @" << config_.fps << "\n";

        return true;
    }

    void captureLoop()
    {
        const size_t expected_bytes =
            static_cast<size_t>(config_.width) *
            static_cast<size_t>(config_.height) * 2U;

        while (!stop_requested_.load())
        {
            v4l2_buffer buf{};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;

            if (xioctl(VIDIOC_DQBUF, &buf) < 0)
            {
                if (stop_requested_.load() || !streaming_.load())
                {
                    break;
                }

                if (errno == EIO)
                {
                    continue;
                }

                std::cerr << "[ERROR] " << config_.device
                          << " DQBUF: " << errnoText() << "\n";
                break;
            }

            const bool valid =
                buf.index < buffers_.size() &&
                buf.bytesused == expected_bytes &&
                !(buf.flags & V4L2_BUF_FLAG_ERROR);

            if (valid)
            {
                FrameSnapshot snap;
                snap.uyvy.resize(expected_bytes);
                std::memcpy(
                    snap.uyvy.data(),
                    buffers_[buf.index].start,
                    expected_bytes);

                snap.sequence = buf.sequence;
                snap.capture_time = SteadyClock::now();
                snap.valid = true;

                {
                    std::lock_guard<std::mutex> lock(frame_mutex_);
                    latest_ = std::move(snap);
                }
            }

            if (stop_requested_.load() || !streaming_.load())
            {
                break;
            }

            if (xioctl(VIDIOC_QBUF, &buf) < 0)
            {
                if (!stop_requested_.load())
                {
                    std::cerr << "[ERROR] " << config_.device
                              << " QBUF: " << errnoText() << "\n";
                }
                break;
            }
        }

        running_.store(false);
    }

    void cleanup()
    {
        for (auto & b : buffers_)
        {
            if (b.start != nullptr && b.start != MAP_FAILED)
            {
                ::munmap(b.start, b.length);
                b.start = nullptr;
            }
        }
        buffers_.clear();

        if (fd_ >= 0)
        {
            ::close(fd_);
            fd_ = -1;
        }
    }
};

struct AppConfig
{
    std::string output_dir = "calibration_dataset";
    int width = 1920;
    int height = 1280;
    int fps = 30;
    int buffers = 8;
    int auto_count = 0;
    double auto_interval_sec = 2.0;

    std::vector<std::string> devices = {
        "/dev/video0",
        "/dev/video2",
        "/dev/video4",
        "/dev/video6",
        "/dev/video8",
        "/dev/video10"
    };

    std::vector<std::string> names = {
        "camera_0",
        "camera_1",
        "camera_2",
        "camera_3",
        "camera_4",
        "camera_5"
    };
};

static void printUsage(const char * program)
{
    std::cout
        << "Usage:\n"
        << "  " << program << " [options]\n\n"
        << "Options:\n"
        << "  --output DIR          Output directory\n"
        << "  --auto-count N        Automatically capture N sets\n"
        << "  --interval SEC        Auto capture interval, default 2.0\n"
        << "  --devices LIST        Comma-separated /dev/video list\n"
        << "  --names LIST          Comma-separated camera names\n"
        << "  --help                Show help\n\n"
        << "Interactive mode:\n"
        << "  ENTER or s + ENTER    Save one 6-camera image set\n"
        << "  q + ENTER             Quit\n";
}

static std::vector<std::string> splitCsv(const std::string & text)
{
    std::vector<std::string> out;
    std::stringstream ss(text);
    std::string item;

    while (std::getline(ss, item, ','))
    {
        if (!item.empty())
        {
            out.push_back(item);
        }
    }

    return out;
}

static bool parseArgs(int argc, char ** argv, AppConfig & cfg)
{
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];

        auto nextValue = [&]() -> std::string
        {
            if (i + 1 >= argc)
            {
                return {};
            }
            return argv[++i];
        };

        if (arg == "--help")
        {
            printUsage(argv[0]);
            return false;
        }
        else if (arg == "--output")
        {
            cfg.output_dir = nextValue();
        }
        else if (arg == "--auto-count")
        {
            cfg.auto_count = std::stoi(nextValue());
        }
        else if (arg == "--interval")
        {
            cfg.auto_interval_sec = std::stod(nextValue());
        }
        else if (arg == "--devices")
        {
            cfg.devices = splitCsv(nextValue());
        }
        else if (arg == "--names")
        {
            cfg.names = splitCsv(nextValue());
        }
        else
        {
            std::cerr << "Unknown argument: " << arg << "\n";
            return false;
        }
    }

    if (cfg.devices.empty() || cfg.devices.size() != cfg.names.size())
    {
        std::cerr
            << "devices and names must be non-empty and have equal length\n";
        return false;
    }

    return true;
}

static std::string wallTimeString()
{
    const auto now = SystemClock::now();
    const std::time_t t = SystemClock::to_time_t(now);
    const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;

    std::tm tm{};
    localtime_r(&t, &tm);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S")
        << "_" << std::setw(3) << std::setfill('0') << ms.count();

    return oss.str();
}

struct SetCaptureResult
{
    bool success = false;
    double max_skew_ms = 0.0;
    double oldest_age_ms = 0.0;
};

static SetCaptureResult saveImageSet(
    const std::vector<std::unique_ptr<V4L2Camera>> & cameras,
    const fs::path & root,
    int set_index)
{
    std::vector<FrameSnapshot> frames(cameras.size());
    const auto snapshot_time = SteadyClock::now();

    for (size_t i = 0; i < cameras.size(); ++i)
    {
        if (!cameras[i]->getLatest(frames[i]))
        {
            std::cerr << "[SAVE FAIL] No frame from "
                      << cameras[i]->config().name << "\n";
            return {};
        }
    }

    auto min_time = frames.front().capture_time;
    auto max_time = frames.front().capture_time;

    for (const auto & frame : frames)
    {
        min_time = std::min(min_time, frame.capture_time);
        max_time = std::max(max_time, frame.capture_time);
    }

    const double skew_ms =
        std::chrono::duration<double, std::milli>(
            max_time - min_time).count();

    const double oldest_age_ms =
        std::chrono::duration<double, std::milli>(
            snapshot_time - min_time).count();

    std::ostringstream set_name;
    set_name << "set_" << std::setw(4) << std::setfill('0') << set_index;

    const fs::path set_dir = root / set_name.str();
    fs::create_directories(set_dir);

    std::ofstream manifest(set_dir / "manifest.csv");
    manifest
        << "camera_name,device,sequence,frame_age_ms,"
        << "relative_to_newest_ms,image_file\n";

    for (size_t i = 0; i < cameras.size(); ++i)
    {
        const auto & cfg = cameras[i]->config();
	auto & frame = frames[i];

	cv::Mat uyvy(
	    cfg.height,
	    cfg.width,
	    CV_8UC2,
	    frame.uyvy.data());

        cv::Mat bgr;
        cv::cvtColor(uyvy, bgr, cv::COLOR_YUV2BGR_UYVY);

        const std::string filename = cfg.name + ".png";
        const fs::path image_path = set_dir / filename;

        std::vector<int> png_params = {
            cv::IMWRITE_PNG_COMPRESSION, 3
        };

        if (!cv::imwrite(image_path.string(), bgr, png_params))
        {
            std::cerr << "[SAVE FAIL] " << image_path << "\n";
            return {};
        }

        const double age_ms =
            std::chrono::duration<double, std::milli>(
                snapshot_time - frame.capture_time).count();

        const double relative_ms =
            std::chrono::duration<double, std::milli>(
                max_time - frame.capture_time).count();

        manifest
            << cfg.name << ","
            << cfg.device << ","
            << frame.sequence << ","
            << std::fixed << std::setprecision(3) << age_ms << ","
            << std::fixed << std::setprecision(3) << relative_ms << ","
            << filename << "\n";
    }

    std::ofstream summary(set_dir / "set_info.txt");
    summary
        << "set_index=" << set_index << "\n"
        << "saved_at=" << wallTimeString() << "\n"
        << "camera_count=" << cameras.size() << "\n"
        << "max_software_snapshot_skew_ms="
        << std::fixed << std::setprecision(3) << skew_ms << "\n"
        << "oldest_frame_age_ms="
        << std::fixed << std::setprecision(3) << oldest_age_ms << "\n"
        << "note=Software snapshot of latest frames; not hardware synchronized.\n";

    std::cout
        << "[SAVED] " << set_dir
        << " | cameras=" << cameras.size()
        << " | skew=" << std::fixed << std::setprecision(2)
        << skew_ms << " ms"
        << " | oldest_age=" << oldest_age_ms << " ms\n";

    return {true, skew_ms, oldest_age_ms};
}

int main(int argc, char ** argv)
{
    AppConfig cfg;

    if (!parseArgs(argc, argv, cfg))
    {
        return argc > 1 ? 1 : 0;
    }

    fs::path output_root =
        fs::path(cfg.output_dir) /
        ("session_" + wallTimeString());

    fs::create_directories(output_root);

    std::ofstream session_info(output_root / "session_info.txt");
    session_info
        << "width=" << cfg.width << "\n"
        << "height=" << cfg.height << "\n"
        << "fps=" << cfg.fps << "\n"
        << "pixel_format=UYVY\n"
        << "camera_count=" << cfg.devices.size() << "\n";

    std::cout
        << "============================================================\n"
        << " Prius Multi-Camera Calibration Capture Tool\n"
        << "============================================================\n"
        << " Output : " << output_root << "\n"
        << " Mode   : " << cfg.width << "x" << cfg.height
        << " UYVY @" << cfg.fps << "\n"
        << " Cameras: " << cfg.devices.size() << "\n"
        << "============================================================\n";

    std::vector<std::unique_ptr<V4L2Camera>> cameras;

    for (size_t i = 0; i < cfg.devices.size(); ++i)
    {
        CameraConfig cam_cfg;
        cam_cfg.device = cfg.devices[i];
        cam_cfg.name = cfg.names[i];
        cam_cfg.width = cfg.width;
        cam_cfg.height = cfg.height;
        cam_cfg.fps = cfg.fps;
        cam_cfg.buffer_count = cfg.buffers;

        auto camera = std::make_unique<V4L2Camera>(cam_cfg);

        if (!camera->start())
        {
            std::cerr << "[FATAL] Failed to start " << cam_cfg.device << "\n";
            for (auto & c : cameras)
            {
                c->stop();
            }
            return 2;
        }

        cameras.push_back(std::move(camera));
    }

    std::cout << "[WARMUP] Waiting 3 seconds for stable frames...\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));

    for (const auto & camera : cameras)
    {
        FrameSnapshot frame;
        if (!camera->getLatest(frame))
        {
            std::cerr << "[FATAL] No valid frame from "
                      << camera->config().name << "\n";
            for (auto & c : cameras)
            {
                c->stop();
            }
            return 3;
        }
    }

    std::cout << "[READY] All cameras have valid frames.\n";

    int set_index = 1;

    if (cfg.auto_count > 0)
    {
        std::cout
            << "[AUTO] count=" << cfg.auto_count
            << " interval=" << cfg.auto_interval_sec << " sec\n";

        for (int i = 0; i < cfg.auto_count; ++i)
        {
            if (!saveImageSet(cameras, output_root, set_index).success)
            {
                std::cerr << "[AUTO] Capture failed.\n";
                break;
            }

            ++set_index;

            if (i + 1 < cfg.auto_count)
            {
                std::this_thread::sleep_for(
                    std::chrono::duration<double>(
                        cfg.auto_interval_sec));
            }
        }
    }
    else
    {
        std::cout
            << "\nInteractive controls:\n"
            << "  ENTER           capture one set\n"
            << "  s + ENTER       capture one set\n"
            << "  q + ENTER       quit\n\n";

        std::string line;

        while (true)
        {
            std::cout << "capture> " << std::flush;

            if (!std::getline(std::cin, line))
            {
                break;
            }

            if (line == "q" || line == "Q")
            {
                break;
            }

            if (line.empty() || line == "s" || line == "S")
            {
                const auto result =
                    saveImageSet(cameras, output_root, set_index);

                if (result.success)
                {
                    ++set_index;
                }
            }
        }
    }

    std::cout << "[STOP] Closing cameras...\n";

    for (auto & camera : cameras)
    {
        camera->stop();
    }

    std::cout << "[DONE] Dataset: " << output_root << "\n";
    return 0;
}
