#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "prius_can_interface/msg/prius_can_status.hpp"

#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_vehicle_msgs/msg/hazard_lights_report.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/turn_indicators_report.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"

using namespace std::chrono_literals;

class PriusAutowareVehicleStatusAdapterNode final : public rclcpp::Node
{
public:
  PriusAutowareVehicleStatusAdapterNode()
  : Node("prius_autoware_vehicle_status_adapter_node")
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
    publish_hazard_disabled_ = declare_parameter<bool>("publish_hazard_disabled", true);

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
      std::bind(&PriusAutowareVehicleStatusAdapterNode::on_prius_status, this, std::placeholders::_1));

    velocity_pub_ = create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
      "/vehicle/status/velocity_status", rclcpp::SensorDataQoS());
    steering_pub_ = create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
      "/vehicle/status/steering_status", rclcpp::SensorDataQoS());
    control_mode_pub_ = create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(
      "/vehicle/status/control_mode", rclcpp::QoS(10));
    turn_pub_ = create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>(
      "/vehicle/status/turn_indicators_status", rclcpp::QoS(10));
    hazard_pub_ = create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>(
      "/vehicle/status/hazard_lights_status", rclcpp::QoS(10));

    const auto publish_period = std::chrono::duration<double>(1.0 / publish_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
      std::bind(&PriusAutowareVehicleStatusAdapterNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "Prius Autoware vehicle status adapter started");
    RCLCPP_INFO(get_logger(), "Input: %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Output: /vehicle/status/{velocity_status,steering_status,control_mode,turn_indicators_status,hazard_lights_status}");
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

  uint8_t to_control_mode() const
  {
    if (status_is_stale()) {
      return autoware_vehicle_msgs::msg::ControlModeReport::NOT_READY;
    }
    if (last_status_.wire_control_enabled) {
      return autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
    }
    return autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;
  }

  uint8_t to_turn_report() const
  {
    if (last_status_.turn_signal_text == "left") {
      return autoware_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT;
    }
    if (last_status_.turn_signal_text == "right") {
      return autoware_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT;
    }
    return autoware_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
  }

  void publish_not_ready(const rclcpp::Time & stamp)
  {
    autoware_vehicle_msgs::msg::ControlModeReport mode;
    mode.stamp = stamp;
    mode.mode = autoware_vehicle_msgs::msg::ControlModeReport::NOT_READY;
    control_mode_pub_->publish(mode);
  }

  void on_timer()
  {
    const auto stamp = now();

    if (status_is_stale()) {
      publish_not_ready(stamp);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No fresh Prius CAN status. Publishing control_mode=NOT_READY. Waiting for %s",
        input_topic_.c_str());
      return;
    }

    const double speed_mps = choose_speed_mps();
    const double steer_tire_rad = steering_tire_angle_rad();
    const double yaw_rate = heading_rate_radps(speed_mps, steer_tire_rad);

    autoware_vehicle_msgs::msg::VelocityReport velocity;
    velocity.header.stamp = stamp;
    velocity.header.frame_id = base_frame_id_;
    velocity.longitudinal_velocity = static_cast<float>(speed_mps);
    velocity.lateral_velocity = 0.0F;
    velocity.heading_rate = static_cast<float>(yaw_rate);
    velocity_pub_->publish(velocity);

    autoware_vehicle_msgs::msg::SteeringReport steering;
    steering.stamp = stamp;
    steering.steering_tire_angle = static_cast<float>(steer_tire_rad);
    steering_pub_->publish(steering);

    autoware_vehicle_msgs::msg::ControlModeReport control_mode;
    control_mode.stamp = stamp;
    control_mode.mode = to_control_mode();
    control_mode_pub_->publish(control_mode);

    autoware_vehicle_msgs::msg::TurnIndicatorsReport turn;
    turn.stamp = stamp;
    turn.report = to_turn_report();
    turn_pub_->publish(turn);

    if (publish_hazard_disabled_) {
      autoware_vehicle_msgs::msg::HazardLightsReport hazard;
      hazard.stamp = stamp;
      hazard.report = autoware_vehicle_msgs::msg::HazardLightsReport::DISABLE;
      hazard_pub_->publish(hazard);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "velocity=%.2f m/s steering_tire=%.4f rad control_mode=%u turn=%u",
      speed_mps,
      steer_tire_rad,
      control_mode.mode,
      turn.report);
  }

  std::string input_topic_;
  double publish_hz_{50.0};
  double status_timeout_sec_{0.5};
  std::string base_frame_id_{"base_link"};
  double steering_ratio_{14.8};
  double wheel_base_m_{2.70};
  std::string speed_source_{"speed"};
  bool estimate_heading_rate_from_steering_{false};
  bool publish_hazard_disabled_{true};

  bool has_status_{false};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  prius_can_interface::msg::PriusCanStatus last_status_;

  rclcpp::Subscription<prius_can_interface::msg::PriusCanStatus>::SharedPtr sub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsReport>::SharedPtr hazard_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PriusAutowareVehicleStatusAdapterNode>());
  } catch (const std::exception & e) {
    std::cerr << "Fatal error: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
