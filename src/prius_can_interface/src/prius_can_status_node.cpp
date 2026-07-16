#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <string>
#include <unordered_map>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "prius_can_interface/msg/prius_can_status.hpp"

using namespace std::chrono_literals;

namespace
{

static uint16_t u16_be(const uint8_t high, const uint8_t low)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(high) << 8) | low);
}

static uint16_t u16_le(const uint8_t low, const uint8_t high)
{
  return static_cast<uint16_t>(low | (static_cast<uint16_t>(high) << 8));
}

static std::string engine_status_text(const uint8_t code)
{
  switch (code) {
    case 0: return "engine_off";
    case 1: return "engine_stop";
    case 2: return "engine_starting";
    case 3: return "engine_running";
    default: return "unknown";
  }
}

static std::string turn_signal_text(const uint8_t code)
{
  switch (code) {
    case 1: return "left";
    case 2: return "right";
    case 3: return "none";
    default: return "unknown";
  }
}

}  // namespace

class PriusCanStatusNode final : public rclcpp::Node
{
public:
  PriusCanStatusNode()
  : Node("prius_can_status_node")
  {
    interface_ = declare_parameter<std::string>("interface", "can0");
    publish_hz_ = declare_parameter<double>("publish_hz", 50.0);
    read_timer_ms_ = declare_parameter<int>("read_timer_ms", 1);
    max_frames_per_cycle_ = declare_parameter<int>("max_frames_per_cycle", 500);
    steering_center_offset_deg_ = declare_parameter<double>("steering_center_offset_deg", 0.0);
    wheel_speed_baseline_raw_ = declare_parameter<int>("wheel_speed_baseline_raw", 0x1A6F);
    warn_unknown_ids_ = declare_parameter<bool>("warn_unknown_ids", false);

    if (publish_hz_ <= 0.0) {
      throw std::runtime_error("publish_hz must be > 0");
    }
    if (read_timer_ms_ <= 0) {
      throw std::runtime_error("read_timer_ms must be > 0");
    }
    if (max_frames_per_cycle_ <= 0) {
      throw std::runtime_error("max_frames_per_cycle must be > 0");
    }

    open_can_socket(interface_);

    status_pub_ = create_publisher<prius_can_interface::msg::PriusCanStatus>(
      "/prius/can/status", rclcpp::SensorDataQoS());

    read_timer_ = create_wall_timer(
      std::chrono::milliseconds(read_timer_ms_),
      std::bind(&PriusCanStatusNode::read_can_timer_callback, this));

    const auto publish_period = std::chrono::duration<double>(1.0 / publish_hz_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
      std::bind(&PriusCanStatusNode::publish_timer_callback, this));

    RCLCPP_INFO(get_logger(), "Prius CAN status node started in READ-ONLY mode");
    RCLCPP_INFO(get_logger(), "SocketCAN interface: %s", interface_.c_str());
    RCLCPP_INFO(get_logger(), "Publishing: /prius/can/status @ %.2f Hz", publish_hz_);
  }

  ~PriusCanStatusNode() override
  {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
      socket_fd_ = -1;
    }
  }

private:
  void open_can_socket(const std::string & interface)
  {
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
      throw std::runtime_error("socket(PF_CAN, SOCK_RAW, CAN_RAW) failed: " +
        std::string(std::strerror(errno)));
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
      const auto msg = "ioctl(SIOCGIFINDEX) failed for " + interface + ": " +
        std::string(std::strerror(errno));
      close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error(msg);
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      const auto msg = "bind() failed for " + interface + ": " + std::string(std::strerror(errno));
      close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error(msg);
    }

    const int flags = fcntl(socket_fd_, F_GETFL, 0);
    if (flags < 0 || fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
      const auto msg = "fcntl(O_NONBLOCK) failed: " + std::string(std::strerror(errno));
      close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error(msg);
    }
  }

  void read_can_timer_callback()
  {
    for (int i = 0; i < max_frames_per_cycle_; ++i) {
      struct can_frame frame;
      const ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));

      if (nbytes == static_cast<ssize_t>(sizeof(frame))) {
        handle_frame(frame);
        continue;
      }

      if (nbytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        return;
      }

      if (nbytes < 0) {
        ++state_.rx_error_count;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "CAN read failed: %s", std::strerror(errno));
        return;
      }

      if (nbytes == 0) {
        return;
      }

      ++state_.rx_error_count;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Short CAN read: %zd bytes", nbytes);
      return;
    }
  }

  void handle_frame(const struct can_frame & frame)
  {
    ++state_.rx_total_count;

    if ((frame.can_id & CAN_ERR_FLAG) != 0U) {
      ++state_.rx_error_count;
      return;
    }

    const bool is_extended = (frame.can_id & CAN_EFF_FLAG) != 0U;
    const uint32_t can_id = is_extended ? (frame.can_id & CAN_EFF_MASK) : (frame.can_id & CAN_SFF_MASK);
    id_counts_[can_id]++;

    const uint8_t * d = frame.data;
    const uint8_t dlc = frame.can_dlc;

    switch (can_id) {
      case 0x0B4:
        decode_speed(d, dlc);
        break;
      case 0x025:
        decode_steering(d, dlc);
        break;
      case 0x0AA:
        decode_wheel_speeds(d, dlc);
        break;
      case 0x226:
        decode_brake(d, dlc);
        break;
      case 0x245:
        decode_gas(d, dlc);
        break;
      case 0x203:
        decode_wire_status(d, dlc);
        break;
      case 0x614:
        decode_steering_levers(d, dlc);
        break;
      case 0x1AA:
        decode_abs(d, dlc);
        break;
      default:
        if (warn_unknown_ids_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Unknown CAN ID: 0x%03X", can_id);
        }
        break;
    }
  }

  void decode_speed(const uint8_t * d, const uint8_t dlc)
  {
    if (dlc < 7) {return;}
    state_.has_speed = true;
    state_.speed_raw = u16_be(d[5], d[6]);
    state_.speed_kmh = static_cast<double>(state_.speed_raw) * 0.01;
    state_.speed_mps = state_.speed_kmh / 3.6;
  }

  void decode_steering(const uint8_t * d, const uint8_t dlc)
  {
    if (dlc < 2) {return;}

    const uint16_t raw16 = u16_be(d[0], d[1]);
    int16_t raw12 = static_cast<int16_t>(raw16 & 0x0FFF);
    if (raw12 >= 0x0800) {
      raw12 = static_cast<int16_t>(raw12 - 0x1000);
    }

    const double deg = static_cast<double>(raw12) * 1.5 - steering_center_offset_deg_;

    state_.has_steering = true;
    state_.steering_raw = raw12;
    state_.steering_wheel_deg = deg;
    state_.steering_wheel_rad = deg * M_PI / 180.0;
  }

  void decode_wheel_speeds(const uint8_t * d, const uint8_t dlc)
  {
    if (dlc < 8) {return;}

    const auto convert = [this](const uint16_t raw) -> double {
      return (static_cast<int>(raw) - wheel_speed_baseline_raw_) * 0.01;
    };

    const uint16_t w01_raw = u16_be(d[0], d[1]);
    const uint16_t w23_raw = u16_be(d[2], d[3]);
    const uint16_t w45_raw = u16_be(d[4], d[5]);
    const uint16_t w67_raw = u16_be(d[6], d[7]);

    const double w01_kmh = convert(w01_raw);
    const double w23_kmh = convert(w23_raw);
    const double w45_kmh = convert(w45_raw);
    const double w67_kmh = convert(w67_raw);

    state_.has_wheel_speeds = true;
    state_.wheel_01_raw = w01_raw;
    state_.wheel_23_raw = w23_raw;
    state_.wheel_45_raw = w45_raw;
    state_.wheel_67_raw = w67_raw;

    state_.wheel_01_kmh = w01_kmh;
    state_.wheel_23_kmh = w23_kmh;
    state_.wheel_45_kmh = w45_kmh;
    state_.wheel_67_kmh = w67_kmh;

    // Inferred from road-test data:
    // positive steering angle = left turn, right wheels move faster.
    state_.wheel_fr_kmh = w01_kmh;
    state_.wheel_fl_kmh = w23_kmh;
    state_.wheel_rr_kmh = w45_kmh;
    state_.wheel_rl_kmh = w67_kmh;

    state_.wheel_fr_mps = state_.wheel_fr_kmh / 3.6;
    state_.wheel_fl_mps = state_.wheel_fl_kmh / 3.6;
    state_.wheel_rr_mps = state_.wheel_rr_kmh / 3.6;
    state_.wheel_rl_mps = state_.wheel_rl_kmh / 3.6;
  }

  void decode_brake(const uint8_t * d, const uint8_t dlc)
  {
    if (dlc < 5) {return;}
    state_.has_brake = true;
    state_.brake_pressure_candidate = u16_le(d[1], d[2]);
    state_.brake_position_raw = d[3];
    state_.brake_status_byte4 = d[4];
    state_.brake_pressed = (d[4] & 0x04) != 0U;
  }

  void decode_gas(const uint8_t * d, const uint8_t dlc)
  {
    if (dlc < 4) {return;}
    state_.has_gas = true;
    state_.gas_pedal_raw = d[2];
    state_.engine_status_code = static_cast<uint8_t>((d[3] >> 4) & 0x0F);
    state_.engine_status_text = engine_status_text(state_.engine_status_code);
  }

  void decode_wire_status(const uint8_t * d, const uint8_t dlc)
  {
    if (dlc < 8) {return;}
    state_.has_wire_status = true;
    state_.wire_status_byte4 = d[4];
    state_.wire_status_byte7 = d[7];
    state_.wire_control_enabled = (d[7] & 0x80) != 0U;
  }

  void decode_steering_levers(const uint8_t * d, const uint8_t dlc)
  {
    if (dlc < 4) {return;}
    state_.has_turn_signal = true;
    state_.turn_signal_code = static_cast<uint8_t>((d[3] >> 4) & 0x0F);
    state_.turn_signal_text = turn_signal_text(state_.turn_signal_code);
    state_.turn_blink_bit = (d[1] & 0x80) != 0U;
  }

  void decode_abs(const uint8_t * d, const uint8_t dlc)
  {
    if (dlc < 4) {return;}
    state_.has_abs = true;
    state_.abs_fl = (d[2] & 0x40) != 0U;
    state_.abs_fr = (d[2] & 0x04) != 0U;
    state_.abs_rl = (d[3] & 0x40) != 0U;
    state_.abs_rr = (d[3] & 0x04) != 0U;
  }

  void publish_timer_callback()
  {
    state_.stamp = now();
    status_pub_->publish(state_);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "speed=%.2f km/h steer=%.1f deg wheels=[%.2f %.2f %.2f %.2f] brake=%s gas=%u turn=%s wire=%s rx=%lu",
      state_.speed_kmh,
      state_.steering_wheel_deg,
      state_.wheel_fr_kmh,
      state_.wheel_fl_kmh,
      state_.wheel_rr_kmh,
      state_.wheel_rl_kmh,
      state_.brake_pressed ? "pressed" : "released",
      state_.gas_pedal_raw,
      state_.turn_signal_text.c_str(),
      state_.wire_control_enabled ? "enabled" : "disabled",
      static_cast<unsigned long>(state_.rx_total_count));
  }

  int socket_fd_{-1};
  std::string interface_;
  double publish_hz_{50.0};
  int read_timer_ms_{1};
  int max_frames_per_cycle_{500};
  double steering_center_offset_deg_{0.0};
  int wheel_speed_baseline_raw_{0x1A6F};
  bool warn_unknown_ids_{false};

  prius_can_interface::msg::PriusCanStatus state_;
  std::unordered_map<uint32_t, uint64_t> id_counts_;

  rclcpp::Publisher<prius_can_interface::msg::PriusCanStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<PriusCanStatusNode>());
  } catch (const std::exception & e) {
    std::cerr << "Fatal error: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
