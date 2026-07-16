# Six-camera + VLP-16 snapshot tool

每按一次 Enter，工具會保存一個資料夾，內容為：

```text
capture_0001_<timestamp>/
├── camera_0.png
├── camera_1.png
├── camera_2.png
├── camera_3.png
├── camera_4.png
├── camera_5.png
├── velodyne.pcd
└── metadata.json
```

預設 topics：

```text
/camera/device_0/image_raw
/camera/device_1/image_raw
/camera/device_2/image_raw
/camera/device_3/image_raw
/camera/device_4/image_raw
/camera/device_5/image_raw
/velodyne_points
```

同步策略：按 Enter 記錄觸發時刻，選最接近該時刻的 LiDAR frame，再以 LiDAR timestamp 為 anchor，從六路相機各選最近 timestamp 的影像。每路 camera-LiDAR 時差與整體 capture span 都寫入 `metadata.json`。

## 放置位置

將整個資料夾放到：

```text
prius_project/tools/sensor_snapshot/
```

你的 `start_vehicle.sh` 已將 Host `tools/` 掛載到 Container `/colcon_ws/tools`，因此資料會保留在 Host。

## 相依檢查

```bash
python3 -c "import cv2, numpy, rclpy"
python3 -c "from sensor_msgs_py import point_cloud2"
```

## 執行

```bash
source /opt/ros/humble/setup.bash
source /colcon_ws/install/setup.bash 2>/dev/null || true

python3 /colcon_ws/tools/sensor_snapshot/capture_six_cam_lidar.py
```

按 Enter 保存一組；輸入 `q` 再按 Enter 結束。

預設資料位置：

```text
/colcon_ws/tools/sensor_snapshot/dataset/session_<timestamp>/
```

因為 `tools/` 是 bind mount，所以 Host 位置為：

```text
prius_project/tools/sensor_snapshot/dataset/session_<timestamp>/
```

## 調整同步容許值

```bash
python3 /colcon_ws/tools/sensor_snapshot/capture_six_cam_lidar.py \
  --tolerance-ms 60
```

預設為 ±120 ms，適合作為你目前相機約 10 Hz、LiDAR 約 9.92 Hz 的初始測試值；正式校正前應依 `metadata.json` 中實際時間差再縮緊。
