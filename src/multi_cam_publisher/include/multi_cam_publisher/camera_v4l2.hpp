#pragma once

#include <linux/videodev2.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace multi_cam_publisher
{

struct CameraConfig
{
  std::string device;
  int width{1920};
  int height{1280};
  int fps{30};
  int buffer_count{8};
  int reconnect_delay_ms{1000};
};

struct CameraCounters
{
  bool connected{false};
  uint64_t valid_frames{0};
  uint64_t error_frames{0};
  uint64_t empty_frames{0};
  uint64_t dqbuf_errors{0};
  uint64_t qbuf_errors{0};
  uint64_t sequence_drops{0};
  uint64_t reconnect_count{0};
  uint32_t last_sequence{0};
  double last_frame_age_sec{-1.0};
};

struct FrameSnapshot
{
  std::vector<uint8_t> uyvy;
  uint32_t width{0};
  uint32_t height{0};
  uint32_t bytes_per_line{0};
  uint32_t sequence{0};
  std::chrono::steady_clock::time_point capture_time{};
};

class CameraV4L2
{
public:
  explicit CameraV4L2(CameraConfig config);
  ~CameraV4L2();

  CameraV4L2(const CameraV4L2 &) = delete;
  CameraV4L2 & operator=(const CameraV4L2 &) = delete;

  void start();
  void stop();
  void requestReconnect();

  bool getLatestFrame(FrameSnapshot & out) const;
  CameraCounters getCounters() const;

  const std::string & device() const noexcept;

private:
  struct MmapBuffer
  {
    void * start{nullptr};
    size_t length{0};
  };

  void workerLoop();
  bool openAndStart();
  void captureUntilFailure();
  void stopStreamAndClose();
  void interruptBlockingDqbuf();

  bool configureFormat();
  void configureFrameRateBestEffort();
  bool allocateAndQueueBuffers();
  bool streamOn();

  bool queueBuffer(uint32_t index);
  bool publishLatestBuffer(uint32_t new_index, const v4l2_buffer & buffer);
  void clearLatestBufferReference();

  int xioctl(unsigned long request, void * arg) const;
  void sleepInterruptibly(std::chrono::milliseconds duration) const;

  static std::string errnoText();
  static std::string fourccToString(uint32_t fourcc);

  CameraConfig config_;

  mutable std::mutex state_mutex_;
  int fd_{-1};
  bool stream_active_{false};
  std::vector<MmapBuffer> buffers_;
  uint32_t actual_width_{0};
  uint32_t actual_height_{0};
  uint32_t bytes_per_line_{0};
  uint32_t size_image_{0};

  mutable std::mutex latest_mutex_;
  int latest_buffer_index_{-1};
  uint32_t latest_bytes_used_{0};
  uint32_t latest_sequence_{0};
  std::chrono::steady_clock::time_point latest_capture_time_{};
  bool have_latest_frame_{false};

  mutable std::mutex counters_mutex_;
  CameraCounters counters_;
  bool have_previous_sequence_{false};
  uint32_t previous_sequence_{0};
  std::chrono::steady_clock::time_point last_valid_frame_time_{};

  std::atomic<bool> running_{false};
  std::atomic<bool> reconnect_requested_{false};
  std::thread worker_thread_;
};

}  // namespace multi_cam_publisher
