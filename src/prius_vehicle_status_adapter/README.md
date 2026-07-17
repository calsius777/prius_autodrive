# prius_vehicle_status_adapter

Adapter from `/prius/can/status` to vehicle-status topics.

## Nodes

### `prius_basic_vehicle_status_adapter_node`

Builds without Autoware message packages. Publishes generic ROS 2 topics:

- `/prius/vehicle/twist` (`geometry_msgs/msg/TwistStamped`)
- `/prius/vehicle/speed_mps` (`std_msgs/msg/Float32`)
- `/prius/vehicle/steering_wheel_angle_rad` (`std_msgs/msg/Float32`)
- `/prius/vehicle/steering_tire_angle_rad` (`std_msgs/msg/Float32`)
- `/prius/vehicle/brake_pressed` (`std_msgs/msg/Bool`)
- `/prius/vehicle/gas_pedal_raw` (`std_msgs/msg/UInt8`)
- `/prius/vehicle/control_mode_text` (`std_msgs/msg/String`)
- `/prius/vehicle/turn_signal_text` (`std_msgs/msg/String`)

### `prius_autoware_vehicle_status_adapter_node`

Built only when `autoware_vehicle_msgs` is installed. Publishes:

- `/vehicle/status/velocity_status`
- `/vehicle/status/steering_status`
- `/vehicle/status/control_mode`
- `/vehicle/status/turn_indicators_status`
- `/vehicle/status/hazard_lights_status`

Gear/reverse is intentionally not implemented yet.

## Build

```bash
cd /colcon_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select prius_vehicle_status_adapter
source install/setup.bash
```

If you want the Autoware adapter target, install or source a workspace that contains `autoware_vehicle_msgs` before building.

On a ROS Humble system that has the package in apt:

```bash
sudo apt update
sudo apt install -y ros-humble-autoware-vehicle-msgs
```

## Run basic adapter

```bash
ros2 launch prius_vehicle_status_adapter prius_basic_vehicle_status_adapter.launch.py
```

## Run Autoware adapter

```bash
ros2 launch prius_vehicle_status_adapter prius_autoware_vehicle_status_adapter.launch.py
```

## Important parameters

- `steering_ratio`: converts steering wheel angle to tire angle.
- `wheel_base_m`: used only when estimating heading rate from steering.
- `speed_source`: `speed` or `wheel_average`.
- `estimate_heading_rate_from_steering`: keep `false` until steering ratio is calibrated; IMU yaw rate is preferred.
