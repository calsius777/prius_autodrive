# Prius 內外參校正流程

## 建議順序

```text
Camera intrinsic -> base_link 定義 -> 粗量測 sensor_poses.yaml -> LiDAR-camera extrinsic -> RViz/projection validation
```

## Camera intrinsic

每顆相機各自標定，輸出 `camera_info.yaml`。建議棋盤內角點 8x6，方格邊長使用實際量測值，例如 0.030 m。

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.030 \
  --no-service-check \
  image:=/camera/device_0/image_raw \
  camera:=/camera/device_0
```

## 擷取 LiDAR-camera 外參資料

```bash
ros2 launch prius_bringup sensors_autostart.launch.py can_interface:=can0
python3 scripts/capture_camera_lidar_calib.py \
  --topics config/camera_topics.yaml \
  --output /colcon_ws/calibration_data/camera_lidar_$(date +%Y%m%d_%H%M%S)
```

## 外參結果

先填 `templates/extrinsics/sensor_poses.yaml`，再複製到：

```text
/colcon_ws/src/prius_description/config/sensor_poses.yaml
```

## 品質檢查

- intrinsic：rectified image 的直線不能彎曲，棋盤姿態需覆蓋中心、四角、遠近、傾斜。
- extrinsic：LiDAR 投影到影像時，棋盤、車道線、路緣、明顯邊界要對齊。
- 遠處左右偏差多半是 yaw；上下偏差多半是 pitch；近處整體偏移多半是 x/y/z translation。
