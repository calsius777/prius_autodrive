# prius_mti630_driver

ROS 2 Humble bringup package for a Movella/Xsens MTi-630 AHRS in the Prius project.

This first version is intentionally simple and robust:

- reads MTData2 packets over Xbus from a serial port;
- publishes `sensor_msgs/Imu`;
- publishes `sensor_msgs/MagneticField` when magnetic field is present;
- publishes MTi status word;
- publishes ROS diagnostics;
- publishes a static TF from `base_link` to `mti630_link` through launch.

It does **not** require the Xsens MT SDK. Configure the MTi-630 with MT Manager first so that it streams Xbus/MTData2.

## Recommended MT Manager output configuration

Set output protocol to **Xbus / MTData2** and enable:

- PacketCounter `0x1020`
- SampleTimeFine `0x1060`
- Quaternion `0x2010`
- Acceleration `0x4020` in m/s²
- RateOfTurn `0x8020` in rad/s
- MagneticField `0xC020`, optional
- StatusWord `0xE020`

Recommended first test rate: **100 Hz**.

## Install into the Prius workspace

```bash
cd /colcon_ws/src
cp -r /path/to/prius_mti630_driver .

cd /colcon_ws
rosdep update || true
sudo apt update
sudo apt install -y python3-serial ros-humble-diagnostic-msgs ros-humble-tf2-ros
colcon build --packages-select prius_mti630_driver
source install/setup.bash
```

## Host udev rule

Run this on the Ubuntu host so the device has a stable name:

```bash
cd /colcon_ws/src/prius_mti630_driver
sudo bash scripts/setup_mti630_udev.sh
```

Reconnect USB, then check:

```bash
ls -l /dev/mti630 /dev/serial/by-id/
```

## Docker run note

Your existing Prius Docker run already uses `--privileged -v /dev:/dev`, so `/dev/mti630` or `/dev/ttyUSB0` should appear inside the container.

Add this to the Dockerfile if missing:

```dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-serial \
    ros-humble-diagnostic-msgs \
    ros-humble-tf2-ros \
  && rm -rf /var/lib/apt/lists/*
```

## Quick serial sniff test

```bash
cd /colcon_ws
source install/setup.bash
ros2 run prius_mti630_driver xbus_sniffer --port /dev/mti630 --baudrate 115200 --duration 10
```

If no packets are printed, check:

1. port name;
2. baudrate;
3. MT Manager output is Xbus/MTData2, not NMEA;
4. USB cable and permissions.

## Launch driver

```bash
ros2 launch prius_mti630_driver mti630.launch.py \
  port:=/dev/mti630 \
  baudrate:=115200 \
  base_frame:=base_link \
  imu_frame:=mti630_link \
  x:=0.0 y:=0.0 z:=0.0 roll:=0.0 pitch:=0.0 yaw:=0.0
```

Replace the transform values with measured Prius installation extrinsics.

## Check topics

```bash
ros2 topic list | grep mti630
ros2 topic hz /sensing/imu/mti630/imu_raw
ros2 topic echo /sensing/imu/mti630/imu_raw --once
ros2 topic echo /diagnostics --once
ros2 run tf2_ros tf2_echo base_link mti630_link
```

## Topic output

| Topic | Type | Meaning |
|---|---|---|
| `/sensing/imu/mti630/imu_raw` | `sensor_msgs/msg/Imu` | orientation, angular velocity, acceleration |
| `/sensing/imu/mti630/mag` | `sensor_msgs/msg/MagneticField` | magnetic field if present |
| `/sensing/imu/mti630/status_word` | `std_msgs/msg/UInt32` | raw MTi status word |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | connection and packet health |

## Important frame note

The vector `axis_map` / `axis_sign` parameters only remap acceleration, gyro and magnetic vectors.
Quaternion remapping is not implemented. For orientation publishing, mount or configure the MTi coordinate frame so it matches your ROS frame convention, or set `publish_orientation: false` until the frame is verified.

## Autoware use

Use this raw topic as the input side for Autoware IMU correction:

```text
/sensing/imu/mti630/imu_raw
```

Then remap/copy the corrected output into your Autoware sensing pipeline after `imu_corrector`.
