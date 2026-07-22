# Prius Calibration Tools

Tools/templates for camera intrinsic and sensor extrinsic calibration.

```bash
ros2 launch prius_bringup sensors_autostart.launch.py can_interface:=can0
python3 scripts/capture_camera_lidar_calib.py --topics config/camera_topics.yaml --output /colcon_ws/calibration_data/test
```

See `docs/calibration_workflow.md`.
