#include <linux/videodev2.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include <cerrno>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>


struct MmapBuffer
{
    void * start = nullptr;
    size_t length = 0;
};


class V4L2CameraTester
{
public:
    V4L2CameraTester(
        const std::string & device,
        int width,
        int height,
        const std::string & output_file)
        : device_(device),
          width_(width),
          height_(height),
          output_file_(output_file)
    {
    }


    ~V4L2CameraTester()
    {
        cleanup();
    }


    bool run()
    {
        std::cout << std::endl;
        std::cout << "==========================================" << std::endl;
        std::cout << "Testing camera: " << device_ << std::endl;
        std::cout << "==========================================" << std::endl;

        if (!openDevice())
        {
            return false;
        }

        if (!checkCapability())
        {
            return false;
        }

        if (!setFormat())
        {
            return false;
        }

        if (!initMmap())
        {
            return false;
        }

        if (!startStreaming())
        {
            return false;
        }

        bool success = captureValidFrame();

        stopStreaming();

        return success;
    }


private:
    std::string device_;
    int width_;
    int height_;
    std::string output_file_;

    int fd_ = -1;

    std::vector<MmapBuffer> buffers_;

    bool streaming_ = false;


    bool openDevice()
    {
        // 使用 blocking mode
        fd_ = open(
            device_.c_str(),
            O_RDWR
        );

        if (fd_ < 0)
        {
            std::cerr
                << "[FAIL] Cannot open "
                << device_
                << ": "
                << std::strerror(errno)
                << std::endl;

            return false;
        }

        std::cout
            << "[OK] Device opened."
            << std::endl;

        return true;
    }


    bool checkCapability()
    {
        v4l2_capability cap{};

        if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0)
        {
            std::cerr
                << "[FAIL] VIDIOC_QUERYCAP: "
                << std::strerror(errno)
                << std::endl;

            return false;
        }

        std::cout
            << "Driver : "
            << reinterpret_cast<const char *>(cap.driver)
            << std::endl;

        std::cout
            << "Card   : "
            << reinterpret_cast<const char *>(cap.card)
            << std::endl;

        std::cout
            << "Bus    : "
            << reinterpret_cast<const char *>(cap.bus_info)
            << std::endl;


        uint32_t capabilities = cap.capabilities;

        if (cap.capabilities & V4L2_CAP_DEVICE_CAPS)
        {
            capabilities = cap.device_caps;
        }


        if (!(capabilities & V4L2_CAP_VIDEO_CAPTURE))
        {
            std::cerr
                << "[FAIL] This is not a Video Capture node."
                << std::endl;

            return false;
        }


        if (!(capabilities & V4L2_CAP_STREAMING))
        {
            std::cerr
                << "[FAIL] Streaming I/O not supported."
                << std::endl;

            return false;
        }

        std::cout
            << "[OK] Video Capture + Streaming supported."
            << std::endl;

        return true;
    }


    bool setFormat()
    {
        v4l2_format fmt{};

        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        fmt.fmt.pix.width = width_;
        fmt.fmt.pix.height = height_;

        fmt.fmt.pix.pixelformat =
            V4L2_PIX_FMT_UYVY;

        fmt.fmt.pix.field =
            V4L2_FIELD_NONE;


        if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0)
        {
            std::cerr
                << "[FAIL] VIDIOC_S_FMT: "
                << std::strerror(errno)
                << std::endl;

            return false;
        }


        width_ = fmt.fmt.pix.width;
        height_ = fmt.fmt.pix.height;


        char fourcc[5] = {
            static_cast<char>(
                fmt.fmt.pix.pixelformat & 0xFF),

            static_cast<char>(
                (fmt.fmt.pix.pixelformat >> 8) & 0xFF),

            static_cast<char>(
                (fmt.fmt.pix.pixelformat >> 16) & 0xFF),

            static_cast<char>(
                (fmt.fmt.pix.pixelformat >> 24) & 0xFF),

            '\0'
        };


        std::cout
            << "[OK] Actual format: "
            << width_
            << "x"
            << height_
            << " "
            << fourcc
            << std::endl;


        if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_UYVY)
        {
            std::cerr
                << "[FAIL] Camera did not accept UYVY."
                << std::endl;

            return false;
        }


        return true;
    }


    bool initMmap()
    {
        v4l2_requestbuffers req{};

        req.count = 8;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;


        if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0)
        {
            std::cerr
                << "[FAIL] VIDIOC_REQBUFS: "
                << std::strerror(errno)
                << std::endl;

            return false;
        }


        if (req.count < 2)
        {
            std::cerr
                << "[FAIL] Not enough V4L2 buffers."
                << std::endl;

            return false;
        }


        buffers_.resize(req.count);


        for (size_t i = 0; i < buffers_.size(); ++i)
        {
            v4l2_buffer buf{};

            buf.type =
                V4L2_BUF_TYPE_VIDEO_CAPTURE;

            buf.memory =
                V4L2_MEMORY_MMAP;

            buf.index =
                static_cast<unsigned int>(i);


            if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0)
            {
                std::cerr
                    << "[FAIL] VIDIOC_QUERYBUF index "
                    << i
                    << ": "
                    << std::strerror(errno)
                    << std::endl;

                return false;
            }


            buffers_[i].length = buf.length;

            buffers_[i].start = mmap(
                nullptr,
                buf.length,
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                fd_,
                buf.m.offset
            );


            if (buffers_[i].start == MAP_FAILED)
            {
                buffers_[i].start = nullptr;

                std::cerr
                    << "[FAIL] mmap buffer "
                    << i
                    << std::endl;

                return false;
            }


            if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0)
            {
                std::cerr
                    << "[FAIL] VIDIOC_QBUF index "
                    << i
                    << ": "
                    << std::strerror(errno)
                    << std::endl;

                return false;
            }
        }


        std::cout
            << "[OK] mmap buffers: "
            << buffers_.size()
            << std::endl;

        return true;
    }


    bool startStreaming()
    {
        v4l2_buf_type type =
            V4L2_BUF_TYPE_VIDEO_CAPTURE;


        if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0)
        {
            std::cerr
                << "[FAIL] VIDIOC_STREAMON: "
                << std::strerror(errno)
                << std::endl;

            return false;
        }


        streaming_ = true;

        std::cout
            << "[OK] Streaming started."
            << std::endl;

        return true;
    }


    bool captureValidFrame()
    {
        constexpr int MAX_ATTEMPTS = 30;

        int bad_count = 0;


        for (int attempt = 1;
             attempt <= MAX_ATTEMPTS;
             ++attempt)
        {
            v4l2_buffer buf{};

            buf.type =
                V4L2_BUF_TYPE_VIDEO_CAPTURE;

            buf.memory =
                V4L2_MEMORY_MMAP;


            if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0)
            {
                if (errno == EINTR)
                {
                    --attempt;
                    continue;
                }


                std::cerr
                    << "[FAIL] VIDIOC_DQBUF: "
                    << std::strerror(errno)
                    << std::endl;

                return false;
            }


            bool valid =
                buf.bytesused > 0 &&
                !(buf.flags & V4L2_BUF_FLAG_ERROR);


            if (!valid)
            {
                ++bad_count;

                std::cout
                    << "[WARN] Bad frame "
                    << bad_count
                    << " bytesused="
                    << buf.bytesused
                    << " flags=0x"
                    << std::hex
                    << buf.flags
                    << std::dec
                    << std::endl;


                ioctl(fd_, VIDIOC_QBUF, &buf);

                continue;
            }


            std::cout
                << "[OK] Valid frame received."
                << std::endl;

            std::cout
                << "     Sequence  : "
                << buf.sequence
                << std::endl;

            std::cout
                << "     Bytes used: "
                << buf.bytesused
                << std::endl;

            std::cout
                << "     Buffer ID : "
                << buf.index
                << std::endl;


            cv::Mat uyvy(
                height_,
                width_,
                CV_8UC2,
                buffers_[buf.index].start
            );


            cv::Mat bgr;

            cv::cvtColor(
                uyvy,
                bgr,
                cv::COLOR_YUV2BGR_UYVY
            );


            bool save_ok =
                cv::imwrite(
                    output_file_,
                    bgr
                );


            if (!save_ok)
            {
                std::cerr
                    << "[FAIL] Cannot save image: "
                    << output_file_
                    << std::endl;

                ioctl(fd_, VIDIOC_QBUF, &buf);

                return false;
            }


            ioctl(fd_, VIDIOC_QBUF, &buf);


            std::cout << std::endl;
            std::cout
                << "========== CAMERA PASS =========="
                << std::endl;

            std::cout
                << "Device : "
                << device_
                << std::endl;

            std::cout
                << "Image  : "
                << output_file_
                << std::endl;

            std::cout
                << "Bad initial frames: "
                << bad_count
                << std::endl;

            std::cout
                << "================================="
                << std::endl;


            return true;
        }


        std::cerr
            << "[FAIL] No valid frame after "
            << MAX_ATTEMPTS
            << " attempts."
            << std::endl;

        return false;
    }


    void stopStreaming()
    {
        if (!streaming_ || fd_ < 0)
        {
            return;
        }


        v4l2_buf_type type =
            V4L2_BUF_TYPE_VIDEO_CAPTURE;


        ioctl(
            fd_,
            VIDIOC_STREAMOFF,
            &type
        );


        streaming_ = false;
    }


    void cleanup()
    {
        stopStreaming();


        for (auto & buffer : buffers_)
        {
            if (buffer.start != nullptr)
            {
                munmap(
                    buffer.start,
                    buffer.length
                );

                buffer.start = nullptr;
            }
        }


        buffers_.clear();


        if (fd_ >= 0)
        {
            close(fd_);
            fd_ = -1;
        }
    }
};


int main(int argc, char ** argv)
{
    if (argc < 3)
    {
        std::cout
            << "Usage:"
            << std::endl;

        std::cout
            << "  "
            << argv[0]
            << " /dev/video0 camera0.jpg"
            << std::endl;

        return 1;
    }


    std::string device = argv[1];
    std::string output = argv[2];


    V4L2CameraTester tester(
        device,
        1920,
        1280,
        output
    );


    bool success = tester.run();


    return success ? 0 : 1;
}
