#include "multi_cam_publisher/camera_v4l2.hpp"

#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace multi_cam_publisher
{

class MultiCamNode final : public rclcpp::Node
{
public:
  MultiCamNode()
  : Node("multi_cam_publisher")
  {
    declareParameters();
    loadParameters();
    validateParameters();
    createCamerasAndPublishers();

    const auto publish_period = std::chrono::duration<double>(1.0 / publish_hz_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
      std::bind(&MultiCamNode::publishLatestFrames, this));

    stats_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(stats_period_sec_)),
      std::bind(&MultiCamNode::reportStatisticsAndWatchdog, this));

    RCLCPP_INFO(
      get_logger(),
      "Started %zu cameras: capture=%dx%d UYVY @ %d FPS, publish=%.2f Hz, output=%s",
      cameras_.size(), capture_width_, capture_height_, capture_fps_, publish_hz_,
      output_encoding_.c_str());
  }

  ~MultiCamNode() override
  {
    for (auto & camera : cameras_) {
      camera->stop();
    }
  }

private:
  void declareParameters()
  {
    declare_parameter<std::vector<std::string>>(
      "devices",
      {"/dev/video0", "/dev/video2", "/dev/video4",
       "/dev/video6", "/dev/video8", "/dev/video10"});

    declare_parameter<std::vector<std::string>>(
      "topics",
      {"/camera/device_0/image_raw", "/camera/device_1/image_raw",
       "/camera/device_2/image_raw", "/camera/device_3/image_raw",
       "/camera/device_4/image_raw", "/camera/device_5/image_raw"});

    declare_parameter<std::vector<std::string>>(
      "frame_ids",
      {"camera_0_optical_frame", "camera_1_optical_frame",
       "camera_2_optical_frame", "camera_3_optical_frame",
       "camera_4_optical_frame", "camera_5_optical_frame"});

    declare_parameter<int>("capture_width", 1920);
    declare_parameter<int>("capture_height", 1280);
    declare_parameter<int>("capture_fps", 30);
    declare_parameter<double>("publish_hz", 10.0);
    declare_parameter<std::string>("output_encoding", "bgr8");
    declare_parameter<int>("buffer_count", 8);
    declare_parameter<int>("reconnect_delay_ms", 1000);
    declare_parameter<double>("frame_timeout_sec", 3.0);
    declare_parameter<double>("stats_period_sec", 5.0);
  }

  void loadParameters()
  {
    devices_ = get_parameter("devices").as_string_array();
    topics_ = get_parameter("topics").as_string_array();
    frame_ids_ = get_parameter("frame_ids").as_string_array();

    capture_width_ = static_cast<int>(get_parameter("capture_width").as_int());
    capture_height_ = static_cast<int>(get_parameter("capture_height").as_int());
    capture_fps_ = static_cast<int>(get_parameter("capture_fps").as_int());
    publish_hz_ = get_parameter("publish_hz").as_double();
    output_encoding_ = get_parameter("output_encoding").as_string();
    buffer_count_ = static_cast<int>(get_parameter("buffer_count").as_int());
    reconnect_delay_ms_ =
      static_cast<int>(get_parameter("reconnect_delay_ms").as_int());
    frame_timeout_sec_ = get_parameter("frame_timeout_sec").as_double();
    stats_period_sec_ = get_parameter("stats_period_sec").as_double();
  }

  void validateParameters()
  {
    if (devices_.empty()) {
      throw std::runtime_error("devices parameter must not be empty");
    }

    if (devices_.size() != topics_.size() || devices_.size() != frame_ids_.size()) {
      throw std::runtime_error(
        "devices, topics and frame_ids must contain the same number of entries");
    }

    if (capture_width_ <= 0 || capture_height_ <= 0 || capture_fps_ <= 0) {
      throw std::runtime_error("capture dimensions and FPS must be positive");
    }

    if (!(publish_hz_ > 0.0) || !(stats_period_sec_ > 0.0)) {
      throw std::runtime_error("publish_hz and stats_period_sec must be positive");
    }

    if (output_encoding_ != "bgr8" && output_encoding_ != "yuv422") {
      throw std::runtime_error("output_encoding must be either bgr8 or yuv422");
    }

    if (buffer_count_ < 2) {
      throw std::runtime_error("buffer_count must be at least 2");
    }
  }

  void createCamerasAndPublishers()
  {
    cameras_.reserve(devices_.size());
    publishers_.reserve(devices_.size());
    previous_counters_.resize(devices_.size());
    last_published_sequence_.resize(devices_.size(), 0U);
    have_published_sequence_.resize(devices_.size(), false);

    for (size_t i = 0; i < devices_.size(); ++i) {
      CameraConfig config;
      config.device = devices_[i];
      config.width = capture_width_;
      config.height = capture_height_;
      config.fps = capture_fps_;
      config.buffer_count = buffer_count_;
      config.reconnect_delay_ms = reconnect_delay_ms_;

      auto camera = std::make_unique<CameraV4L2>(config);
      auto publisher = create_publisher<sensor_msgs::msg::Image>(
        topics_[i], rclcpp::SensorDataQoS());

      camera->start();
      cameras_.push_back(std::move(camera));
      publishers_.push_back(std::move(publisher));
    }
  }

  void publishLatestFrames()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    const rclcpp::Time ros_now = now();

    for (size_t i = 0; i < cameras_.size(); ++i) {
      FrameSnapshot frame;
      if (!cameras_[i]->getLatestFrame(frame)) {
        continue;
      }

      if (frame.width == 0 || frame.height == 0 || frame.bytes_per_line == 0) {
        continue;
      }

      const size_t required_size =
        static_cast<size_t>(frame.bytes_per_line) * static_cast<size_t>(frame.height);

      if (frame.uyvy.size() < required_size) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "%s frame too small: got=%zu expected_at_least=%zu",
          devices_[i].c_str(), frame.uyvy.size(), required_size);
        continue;
      }

      if (have_published_sequence_[i] && last_published_sequence_[i] == frame.sequence) {
        continue;
      }

      try {
        auto message = std::make_unique<sensor_msgs::msg::Image>();

        const auto age_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          steady_now - frame.capture_time).count();

        const int64_t clamped_age_ns = std::max<int64_t>(0, age_ns);
        message->header.stamp =
          ros_now - rclcpp::Duration::from_nanoseconds(clamped_age_ns);
        message->header.frame_id = frame_ids_[i];
        message->height = frame.height;
        message->width = frame.width;
        message->is_bigendian = false;

        if (output_encoding_ == "yuv422") {
          message->encoding = "yuv422";  // ROS 2 Humble name for UYVY 4:2:2.
          message->step = frame.bytes_per_line;
          message->data.assign(
            frame.uyvy.begin(),
            frame.uyvy.begin() + static_cast<std::ptrdiff_t>(required_size));
        } else {
          cv::Mat uyvy(
            static_cast<int>(frame.height),
            static_cast<int>(frame.width),
            CV_8UC2,
            frame.uyvy.data(),
            static_cast<size_t>(frame.bytes_per_line));

          cv::Mat bgr;
          cv::cvtColor(uyvy, bgr, cv::COLOR_YUV2BGR_UYVY);

          message->encoding = "bgr8";
          message->step = frame.width * 3U;
          message->data.assign(bgr.datastart, bgr.dataend);
        }

        publishers_[i]->publish(std::move(message));
        last_published_sequence_[i] = frame.sequence;
        have_published_sequence_[i] = true;
      } catch (const cv::Exception & error) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "%s OpenCV conversion failed: %s",
          devices_[i].c_str(), error.what());
      }
    }
  }

  void reportStatisticsAndWatchdog()
  {
    const auto stats_now = std::chrono::steady_clock::now();
    const double dt = std::max(
      1e-6, std::chrono::duration<double>(stats_now - last_stats_time_).count());

    for (size_t i = 0; i < cameras_.size(); ++i) {
      const CameraCounters current = cameras_[i]->getCounters();
      const CameraCounters previous = previous_counters_[i];

      const uint64_t frame_delta = current.valid_frames - previous.valid_frames;
      const double capture_fps = static_cast<double>(frame_delta) / dt;

      RCLCPP_INFO(
        get_logger(),
        "[%s] connected=%d capture=%.2f FPS valid=%llu error=%llu empty=%llu "
        "seq_drop=%llu dqerr=%llu qerr=%llu reconnect=%llu age=%.3f s",
        devices_[i].c_str(),
        current.connected ? 1 : 0,
        capture_fps,
        static_cast<unsigned long long>(current.valid_frames),
        static_cast<unsigned long long>(current.error_frames),
        static_cast<unsigned long long>(current.empty_frames),
        static_cast<unsigned long long>(current.sequence_drops),
        static_cast<unsigned long long>(current.dqbuf_errors),
        static_cast<unsigned long long>(current.qbuf_errors),
        static_cast<unsigned long long>(current.reconnect_count),
        current.last_frame_age_sec);

      if (current.connected &&
          current.last_frame_age_sec >= 0.0 &&
          current.last_frame_age_sec > frame_timeout_sec_)
      {
        RCLCPP_WARN(
          get_logger(),
          "%s no valid frame for %.2f s; requesting reconnect",
          devices_[i].c_str(), current.last_frame_age_sec);
        cameras_[i]->requestReconnect();
      }

      previous_counters_[i] = current;
    }

    last_stats_time_ = stats_now;
  }

  std::vector<std::string> devices_;
  std::vector<std::string> topics_;
  std::vector<std::string> frame_ids_;

  int capture_width_{1920};
  int capture_height_{1280};
  int capture_fps_{30};
  double publish_hz_{10.0};
  std::string output_encoding_{"bgr8"};
  int buffer_count_{8};
  int reconnect_delay_ms_{1000};
  double frame_timeout_sec_{3.0};
  double stats_period_sec_{5.0};

  std::vector<std::unique_ptr<CameraV4L2>> cameras_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers_;
  std::vector<CameraCounters> previous_counters_;
  std::vector<uint32_t> last_published_sequence_;
  std::vector<bool> have_published_sequence_;
  std::chrono::steady_clock::time_point last_stats_time_{std::chrono::steady_clock::now()};

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

}  // namespace multi_cam_publisher

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<multi_cam_publisher::MultiCamNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    std::cerr << "multi_cam_publisher fatal error: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
