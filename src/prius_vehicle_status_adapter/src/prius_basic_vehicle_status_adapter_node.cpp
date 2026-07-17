#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "prius_can_interface/msg/prius_can_status.hpp"

using namespace std::chrono_literals;

namespace
{

float clamp_float(const double value, const double lo, const double hi)
{
  return static_cast<float>(std::min(std::max(value, lo), hi));
}

std::string control_mode_text_from_wire(const bool wire_control_enabled)
{
  return wire_control_enabled ? "autonomous_wire_enabled" : "manual_or_wire_disabled";
}

}  // namespace

class PriusBasicVehicleStatusAdapterNode final : public rclcpp::Node
{
public:
  PriusBasicVehicleStatusAdapterNode()
  : Node("prius_basic_vehicle_status_adapter_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/prius/can/status");
    publish_hz_ = declare_parameter<double>("publish_hz", 50.0);
    status_timeout_sec_ = declare_parameter<double>("status_timeout_sec", 0.5);
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
    steering_ratio_ = declare_parameter<double>("steering_ratio", 14.8);
    wheel_base_m_ = declare_parameter<double>("wheel_base_m", 2.70);
    speed_source_ = declare_parameter<std::string>("speed_source", "speed");
    estimate_heading_rate_from_steering_ = declare_parameter<bool>(
      "estimate_heading_rate_from_steering", false);

    if (publish_hz_ <= 0.0) {
      throw std::runtime_error("publish_hz must be > 0");
    }
    if (steering_ratio_ <= 0.0) {
      throw std::runtime_error("steering_ratio must be > 0");
    }
    if (wheel_base_m_ <= 0.0) {
      throw std::runtime_error("wheel_base_m must be > 0");
    }

    sub_ = create_subscription<prius_can_interface::msg::PriusCanStatus>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PriusBasicVehicleStatusAdapterNode::on_prius_status, this, std::placeholders::_1));

    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/prius/vehicle/twist", rclcpp::SensorDataQoS());
    speed_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/prius/vehicle/speed_mps", rclcpp::SensorDataQoS());
    steering_wheel_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/prius/vehicle/steering_wheel_angle_rad", rclcpp::SensorDataQoS());
    steering_tire_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/prius/vehicle/steering_tire_angle_rad", rclcpp::SensorDataQoS());
    brake_pressed_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/prius/vehicle/brake_pressed", rclcpp::SensorDataQoS());
    gas_pedal_pub_ = create_publisher<std_msgs::msg::UInt8>(
      "/prius/vehicle/gas_pedal_raw", rclcpp::SensorDataQoS());
    control_mode_text_pub_ = create_publisher<std_msgs::msg::String>(
      "/prius/vehicle/control_mode_text", rclcpp::QoS(10));
    turn_signal_text_pub_ = create_publisher<std_msgs::msg::String>(
      "/prius/vehicle/turn_signal_text", rclcpp::QoS(10));

    const auto publish_period = std::chrono::duration<double>(1.0 / publish_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
      std::bind(&PriusBasicVehicleStatusAdapterNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "Prius basic vehicle status adapter started");
    RCLCPP_INFO(get_logger(), "Input: %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "steering_ratio=%.3f wheel_base_m=%.3f speed_source=%s",
      steering_ratio_, wheel_base_m_, speed_source_.c_str());
  }

private:
  void on_prius_status(const prius_can_interface::msg::PriusCanStatus::SharedPtr msg)
  {
    last_status_ = *msg;
    has_status_ = true;
    last_status_time_ = now();
  }

  bool status_is_stale() const
  {
    if (!has_status_) {
      return true;
    }
    const double age = (now() - last_status_time_).seconds();
    return age > status_timeout_sec_;
  }

  double choose_speed_mps() const
  {
    if (speed_source_ == "wheel_average" && last_status_.has_wheel_speeds) {
      return (last_status_.wheel_fr_mps + last_status_.wheel_fl_mps +
             last_status_.wheel_rr_mps + last_status_.wheel_rl_mps) / 4.0;
    }
    return last_status_.speed_mps;
  }

  double steering_tire_angle_rad() const
  {
    return last_status_.steering_wheel_rad / steering_ratio_;
  }

  double heading_rate_radps(const double speed_mps, const double steering_tire_rad) const
  {
    if (!estimate_heading_rate_from_steering_) {
      return 0.0;
    }
    return speed_mps * std::tan(steering_tire_rad) / wheel_base_m_;
  }

  void on_timer()
  {
    if (status_is_stale()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No fresh Prius CAN status. Waiting for %s", input_topic_.c_str());
      return;
    }

    const auto stamp = now();
    const double speed_mps = choose_speed_mps();
    const double steer_wheel_rad = last_status_.steering_wheel_rad;
    const double steer_tire_rad = steering_tire_angle_rad();
    const double yaw_rate = heading_rate_radps(speed_mps, steer_tire_rad);

    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = stamp;
    twist.header.frame_id = base_frame_id_;
    twist.twist.linear.x = speed_mps;
    twist.twist.linear.y = 0.0;
    twist.twist.linear.z = 0.0;
    twist.twist.angular.x = 0.0;
    twist.twist.angular.y = 0.0;
    twist.twist.angular.z = yaw_rate;
    twist_pub_->publish(twist);

    std_msgs::msg::Float32 speed_msg;
    speed_msg.data = clamp_float(speed_mps, -200.0, 200.0);
    speed_pub_->publish(speed_msg);

    std_msgs::msg::Float32 steer_wheel_msg;
    steer_wheel_msg.data = clamp_float(steer_wheel_rad, -100.0, 100.0);
    steering_wheel_pub_->publish(steer_wheel_msg);

    std_msgs::msg::Float32 steer_tire_msg;
    steer_tire_msg.data = clamp_float(steer_tire_rad, -10.0, 10.0);
    steering_tire_pub_->publish(steer_tire_msg);

    std_msgs::msg::Bool brake_msg;
    brake_msg.data = last_status_.brake_pressed;
    brake_pressed_pub_->publish(brake_msg);

    std_msgs::msg::UInt8 gas_msg;
    gas_msg.data = last_status_.gas_pedal_raw;
    gas_pedal_pub_->publish(gas_msg);

    std_msgs::msg::String mode_msg;
    mode_msg.data = control_mode_text_from_wire(last_status_.wire_control_enabled);
    control_mode_text_pub_->publish(mode_msg);

    std_msgs::msg::String turn_msg;
    turn_msg.data = last_status_.turn_signal_text;
    turn_signal_text_pub_->publish(turn_msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "speed=%.2f m/s steer_tire=%.4f rad brake=%s turn=%s wire=%s",
      speed_mps,
      steer_tire_rad,
      last_status_.brake_pressed ? "pressed" : "released",
      last_status_.turn_signal_text.c_str(),
      last_status_.wire_control_enabled ? "enabled" : "disabled");
  }

  std::string input_topic_;
  double publish_hz_{50.0};
  double status_timeout_sec_{0.5};
  std::string base_frame_id_{"base_link"};
  double steering_ratio_{14.8};
  double wheel_base_m_{2.70};
  std::string speed_source_{"speed"};
  bool estimate_heading_rate_from_steering_{false};

  bool has_status_{false};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  prius_can_interface::msg::PriusCanStatus last_status_;

  rclcpp::Subscription<prius_can_interface::msg::PriusCanStatus>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_wheel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_tire_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr brake_pressed_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr gas_pedal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_mode_text_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr turn_signal_text_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PriusBasicVehicleStatusAdapterNode>());
  } catch (const std::exception & e) {
    std::cerr << "Fatal error: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
