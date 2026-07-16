# prius_sensor_bringup

ROS 2 Humble sensor bringup package for the Prius project.

Current contents:

- Velodyne VLP-16 driver
- Velodyne PointCloud2 conversion
- `base_link -> velodyne` static TF
- Optional RViz2 startup

## Requirements

The Docker image must include:

```bash
apt update
apt install -y \
  ros-humble-velodyne \
  ros-humble-rviz2 \
  ros-humble-tf2-ros
```

## Build

Copy this package to:

```text
/colcon_ws/src/prius_sensor_bringup
```

Then:

```bash
cd /colcon_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select prius_sensor_bringup --symlink-install
source install/setup.bash
```

## Launch LiDAR only

```bash
ros2 launch prius_sensor_bringup lidar.launch.py
```

## Launch LiDAR + RViz2

```bash
ros2 launch prius_sensor_bringup lidar.launch.py rviz:=true
```

## Override LiDAR mounting pose

The default static transform is identity and is intended only for bench testing.
Replace the following values with the measured sensor pose relative to `base_link`:

```bash
ros2 launch prius_sensor_bringup lidar.launch.py \
  lidar_x:=0.80 \
  lidar_y:=0.00 \
  lidar_z:=1.65 \
  lidar_roll:=0.0 \
  lidar_pitch:=0.0 \
  lidar_yaw:=0.0 \
  rviz:=true
```

## Key topics

```text
/velodyne_packets
/velodyne_points
/tf_static
```

## Notes

The parameter file assumes:

```text
VLP-16 IP: 192.168.1.201
UDP port: 2368
RPM: 600
Frame: velodyne
```

Change `config/vlp16.yaml` if the sensor configuration differs.
