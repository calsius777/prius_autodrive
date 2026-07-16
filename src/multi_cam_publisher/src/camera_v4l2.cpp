#include "multi_cam_publisher/camera_v4l2.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <sstream>
#include <utility>

namespace multi_cam_publisher
{

CameraV4L2::CameraV4L2(CameraConfig config)
: config_(std::move(config))
{
}

CameraV4L2::~CameraV4L2()
{
  stop();
}

void CameraV4L2::start()
{
  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true)) {
    return;
  }

  reconnect_requested_.store(false);
  worker_thread_ = std::thread(&CameraV4L2::workerLoop, this);
}

void CameraV4L2::stop()
{
  if (!running_.exchange(false)) {
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
    return;
  }

  reconnect_requested_.store(true);
  interruptBlockingDqbuf();

  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void CameraV4L2::requestReconnect()
{
  if (!running_.load()) {
    return;
  }

  reconnect_requested_.store(true);
  interruptBlockingDqbuf();
}

bool CameraV4L2::getLatestFrame(FrameSnapshot & out) const
{
  std::lock_guard<std::mutex> lock(latest_mutex_);

  if (!have_latest_frame_ || latest_buffer_index_ < 0) {
    return false;
  }

  const auto index = static_cast<size_t>(latest_buffer_index_);
  if (index >= buffers_.size() || buffers_[index].start == nullptr) {
    return false;
  }

  const size_t copy_size = std::min<size_t>(latest_bytes_used_, buffers_[index].length);
  if (copy_size == 0) {
    return false;
  }

  const auto * src = static_cast<const uint8_t *>(buffers_[index].start);
  out.uyvy.assign(src, src + copy_size);
  out.width = actual_width_;
  out.height = actual_height_;
  out.bytes_per_line = bytes_per_line_;
  out.sequence = latest_sequence_;
  out.capture_time = latest_capture_time_;
  return true;
}

CameraCounters CameraV4L2::getCounters() const
{
  std::lock_guard<std::mutex> lock(counters_mutex_);
  CameraCounters out = counters_;

  if (out.valid_frames > 0) {
    const auto now = std::chrono::steady_clock::now();
    out.last_frame_age_sec =
      std::chrono::duration<double>(now - last_valid_frame_time_).count();
  }

  return out;
}

const std::string & CameraV4L2::device() const noexcept
{
  return config_.device;
}

void CameraV4L2::workerLoop()
{
  while (running_.load()) {
    reconnect_requested_.store(false);

    if (!openAndStart()) {
      stopStreamAndClose();
      if (running_.load()) {
        sleepInterruptibly(std::chrono::milliseconds(config_.reconnect_delay_ms));
      }
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(counters_mutex_);
      counters_.connected = true;
      counters_.reconnect_count++;
      have_previous_sequence_ = false;
    }

    captureUntilFailure();

    {
      std::lock_guard<std::mutex> lock(counters_mutex_);
      counters_.connected = false;
    }

    stopStreamAndClose();

    if (running_.load()) {
      sleepInterruptibly(std::chrono::milliseconds(config_.reconnect_delay_ms));
    }
  }

  stopStreamAndClose();
}

bool CameraV4L2::openAndStart()
{
  const int new_fd = ::open(config_.device.c_str(), O_RDWR | O_CLOEXEC);
  if (new_fd < 0) {
    std::cerr << "[CAM] " << config_.device << " open failed: " << errnoText() << '\n';
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    fd_ = new_fd;
  }

  v4l2_capability cap{};
  if (xioctl(VIDIOC_QUERYCAP, &cap) < 0) {
    std::cerr << "[CAM] " << config_.device << " QUERYCAP failed: " << errnoText() << '\n';
    return false;
  }

  uint32_t capabilities = cap.capabilities;
  if (cap.capabilities & V4L2_CAP_DEVICE_CAPS) {
    capabilities = cap.device_caps;
  }

  if (!(capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
      !(capabilities & V4L2_CAP_STREAMING))
  {
    std::cerr << "[CAM] " << config_.device
              << " is not a streaming capture device\n";
    return false;
  }

  if (!configureFormat()) {
    return false;
  }

  configureFrameRateBestEffort();

  if (!allocateAndQueueBuffers()) {
    return false;
  }

  if (!streamOn()) {
    return false;
  }

  std::cout << "[CAM] " << config_.device
            << " streaming " << actual_width_ << 'x' << actual_height_
            << " UYVY @ requested " << config_.fps
            << " FPS, mmap buffers=" << buffers_.size() << '\n';

  return true;
}

void CameraV4L2::captureUntilFailure()
{
  while (running_.load() && !reconnect_requested_.load()) {
    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;

    if (xioctl(VIDIOC_DQBUF, &buffer) < 0) {
      if (!running_.load() || reconnect_requested_.load()) {
        break;
      }

      {
        std::lock_guard<std::mutex> lock(counters_mutex_);
        counters_.dqbuf_errors++;
      }

      if (errno == EINTR || errno == EIO) {
        continue;
      }

      std::cerr << "[CAM] " << config_.device
                << " DQBUF failed: " << errnoText() << '\n';
      break;
    }

    const bool is_error = (buffer.flags & V4L2_BUF_FLAG_ERROR) != 0;
    const bool is_empty = buffer.bytesused == 0;

    if (is_error || is_empty) {
      {
        std::lock_guard<std::mutex> lock(counters_mutex_);
        if (is_error) {
          counters_.error_frames++;
        }
        if (is_empty) {
          counters_.empty_frames++;
        }
      }

      if (!queueBuffer(buffer.index)) {
        break;
      }
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(counters_mutex_);
      counters_.valid_frames++;
      counters_.last_sequence = buffer.sequence;

      if (have_previous_sequence_) {
        const uint32_t delta = buffer.sequence - previous_sequence_;
        if (delta > 1 && delta < 0x80000000U) {
          counters_.sequence_drops += static_cast<uint64_t>(delta - 1);
        }
      }

      previous_sequence_ = buffer.sequence;
      have_previous_sequence_ = true;
      last_valid_frame_time_ = std::chrono::steady_clock::now();
    }

    if (!publishLatestBuffer(buffer.index, buffer)) {
      break;
    }
  }
}

void CameraV4L2::stopStreamAndClose()
{
  int old_fd = -1;
  bool was_streaming = false;

  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    old_fd = fd_;
    was_streaming = stream_active_;
    fd_ = -1;
    stream_active_ = false;
  }

  if (old_fd < 0) {
    return;
  }

  if (was_streaming) {
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ::ioctl(old_fd, VIDIOC_STREAMOFF, &type);
  }

  {
    std::lock_guard<std::mutex> latest_lock(latest_mutex_);
    have_latest_frame_ = false;
    latest_buffer_index_ = -1;
    latest_bytes_used_ = 0;

    for (auto & buffer : buffers_) {
      if (buffer.start != nullptr && buffer.start != MAP_FAILED) {
        ::munmap(buffer.start, buffer.length);
        buffer.start = nullptr;
      }
    }
    buffers_.clear();
  }

  ::close(old_fd);
}

void CameraV4L2::interruptBlockingDqbuf()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (fd_ >= 0 && stream_active_) {
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ::ioctl(fd_, VIDIOC_STREAMOFF, &type);
    stream_active_ = false;
  }
}

bool CameraV4L2::configureFormat()
{
  v4l2_format format{};
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  format.fmt.pix.width = static_cast<uint32_t>(config_.width);
  format.fmt.pix.height = static_cast<uint32_t>(config_.height);
  format.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
  format.fmt.pix.field = V4L2_FIELD_NONE;

  if (xioctl(VIDIOC_S_FMT, &format) < 0) {
    std::cerr << "[CAM] " << config_.device
              << " S_FMT failed: " << errnoText() << '\n';
    return false;
  }

  actual_width_ = format.fmt.pix.width;
  actual_height_ = format.fmt.pix.height;
  bytes_per_line_ = format.fmt.pix.bytesperline;
  size_image_ = format.fmt.pix.sizeimage;

  if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_UYVY ||
      actual_width_ != static_cast<uint32_t>(config_.width) ||
      actual_height_ != static_cast<uint32_t>(config_.height))
  {
    std::cerr << "[CAM] " << config_.device
              << " negotiated unexpected format: "
              << actual_width_ << 'x' << actual_height_ << ' '
              << fourccToString(format.fmt.pix.pixelformat) << '\n';
    return false;
  }

  if (bytes_per_line_ == 0) {
    bytes_per_line_ = actual_width_ * 2U;
  }
  if (size_image_ == 0) {
    size_image_ = bytes_per_line_ * actual_height_;
  }

  return true;
}

void CameraV4L2::configureFrameRateBestEffort()
{
  v4l2_streamparm parm{};
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (xioctl(VIDIOC_G_PARM, &parm) < 0) {
    return;
  }

  if (!(parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)) {
    return;
  }

  parm.parm.capture.timeperframe.numerator = 1;
  parm.parm.capture.timeperframe.denominator = static_cast<uint32_t>(config_.fps);

  if (xioctl(VIDIOC_S_PARM, &parm) < 0) {
    std::cerr << "[CAM] " << config_.device
              << " warning: S_PARM failed: " << errnoText() << '\n';
  }
}

bool CameraV4L2::allocateAndQueueBuffers()
{
  v4l2_requestbuffers request{};
  request.count = static_cast<uint32_t>(config_.buffer_count);
  request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  request.memory = V4L2_MEMORY_MMAP;

  if (xioctl(VIDIOC_REQBUFS, &request) < 0) {
    std::cerr << "[CAM] " << config_.device
              << " REQBUFS failed: " << errnoText() << '\n';
    return false;
  }

  if (request.count < 2) {
    std::cerr << "[CAM] " << config_.device
              << " returned too few mmap buffers: " << request.count << '\n';
    return false;
  }

  buffers_.assign(request.count, MmapBuffer{});

  for (uint32_t i = 0; i < request.count; ++i) {
    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = i;

    if (xioctl(VIDIOC_QUERYBUF, &buffer) < 0) {
      std::cerr << "[CAM] " << config_.device
                << " QUERYBUF(" << i << ") failed: " << errnoText() << '\n';
      return false;
    }

    void * mapped = ::mmap(
      nullptr,
      buffer.length,
      PROT_READ | PROT_WRITE,
      MAP_SHARED,
      fd_,
      buffer.m.offset);

    if (mapped == MAP_FAILED) {
      std::cerr << "[CAM] " << config_.device
                << " mmap(" << i << ") failed: " << errnoText() << '\n';
      return false;
    }

    buffers_[i].start = mapped;
    buffers_[i].length = buffer.length;
  }

  for (uint32_t i = 0; i < request.count; ++i) {
    if (!queueBuffer(i)) {
      return false;
    }
  }

  return true;
}

bool CameraV4L2::streamOn()
{
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(VIDIOC_STREAMON, &type) < 0) {
    std::cerr << "[CAM] " << config_.device
              << " STREAMON failed: " << errnoText() << '\n';
    return false;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  stream_active_ = true;
  return true;
}

bool CameraV4L2::queueBuffer(uint32_t index)
{
  v4l2_buffer buffer{};
  buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buffer.memory = V4L2_MEMORY_MMAP;
  buffer.index = index;

  if (xioctl(VIDIOC_QBUF, &buffer) < 0) {
    if (!running_.load() || reconnect_requested_.load()) {
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(counters_mutex_);
      counters_.qbuf_errors++;
    }

    std::cerr << "[CAM] " << config_.device
              << " QBUF(" << index << ") failed: " << errnoText() << '\n';
    return false;
  }

  return true;
}

bool CameraV4L2::publishLatestBuffer(uint32_t new_index, const v4l2_buffer & buffer)
{
  std::lock_guard<std::mutex> lock(latest_mutex_);

  if (latest_buffer_index_ >= 0) {
    const uint32_t previous_index = static_cast<uint32_t>(latest_buffer_index_);
    if (!queueBuffer(previous_index)) {
      return false;
    }
  }

  latest_buffer_index_ = static_cast<int>(new_index);
  latest_bytes_used_ = buffer.bytesused;
  latest_sequence_ = buffer.sequence;
  latest_capture_time_ = std::chrono::steady_clock::now();
  have_latest_frame_ = true;
  return true;
}

void CameraV4L2::clearLatestBufferReference()
{
  std::lock_guard<std::mutex> lock(latest_mutex_);
  have_latest_frame_ = false;
  latest_buffer_index_ = -1;
  latest_bytes_used_ = 0;
}

int CameraV4L2::xioctl(unsigned long request, void * arg) const
{
  int local_fd = -1;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    local_fd = fd_;
  }

  if (local_fd < 0) {
    errno = EBADF;
    return -1;
  }

  int result = -1;
  do {
    result = ::ioctl(local_fd, request, arg);
  } while (result < 0 && errno == EINTR && running_.load());

  return result;
}

void CameraV4L2::sleepInterruptibly(std::chrono::milliseconds duration) const
{
  constexpr auto step = std::chrono::milliseconds(50);
  auto remaining = duration;

  while (running_.load() && remaining.count() > 0) {
    const auto current = std::min(step, remaining);
    std::this_thread::sleep_for(current);
    remaining -= current;
  }
}

std::string CameraV4L2::errnoText()
{
  return std::strerror(errno);
}

std::string CameraV4L2::fourccToString(uint32_t fourcc)
{
  std::string text(4, '\0');
  text[0] = static_cast<char>(fourcc & 0xFF);
  text[1] = static_cast<char>((fourcc >> 8) & 0xFF);
  text[2] = static_cast<char>((fourcc >> 16) & 0xFF);
  text[3] = static_cast<char>((fourcc >> 24) & 0xFF);
  return text;
}

}  // namespace multi_cam_publisher
