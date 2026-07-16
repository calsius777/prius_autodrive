# camera_calibration_capture

六相機校正影像採集工具。

## 用途

此工具不是校正演算法本身，而是建立可供後續校正使用的六相機影像資料集。

預設相機：

```text
/dev/video0
/dev/video2
/dev/video4
/dev/video6
/dev/video8
/dev/video10
```

預設擷取模式：

```text
1920×1280
UYVY 4:2:2
30 FPS
8 mmap buffers
blocking VIDIOC_DQBUF
```

## 校正時是否一定要六顆一起拍？

### 單目內參

不需要。

每一顆相機可以分別拍攝棋盤格、ChArUco 或 AprilGrid，獨立求：

- fx
- fy
- cx
- cy
- distortion coefficients

### 雙目或多相機外參

建議使用「同一姿態、同一組編號」的影像集。

例如：

```text
set_0001/
├── camera_0.png
├── camera_1.png
├── camera_2.png
├── camera_3.png
├── camera_4.png
└── camera_5.png
```

這表示六顆相機在同一次軟體 snapshot 下保存的最新 frame。

注意：這不是硬體同步。工具會在 `manifest.csv` 中記錄 sequence、frame age 與軟體 snapshot skew。

## Build

```bash
cd /colcon_ws/tools/camera_calibration_capture

mkdir -p build
cd build

cmake ..
make -j$(nproc)
```

## Interactive mode

```bash
cd /colcon_ws/tools/camera_calibration_capture

chmod +x run_capture.sh
./run_capture.sh
```

控制：

```text
ENTER       儲存一組六相機影像
s + ENTER   儲存一組六相機影像
q + ENTER   離開
```

## Auto mode

例如自動拍 30 組，每 2 秒一組：

```bash
./build/camera_calibration_capture \
  --output calibration_dataset \
  --auto-count 30 \
  --interval 2.0
```

## 輸出

```text
calibration_dataset/
└── session_YYYYMMDD_HHMMSS_mmm/
    ├── session_info.txt
    ├── set_0001/
    │   ├── camera_0.png
    │   ├── camera_1.png
    │   ├── camera_2.png
    │   ├── camera_3.png
    │   ├── camera_4.png
    │   ├── camera_5.png
    │   ├── manifest.csv
    │   └── set_info.txt
    ├── set_0002/
    └── ...
```

## 拍攝建議

內參校正每顆相機建議至少準備多組不同姿態：

- 校正板靠近畫面中心
- 校正板靠近四角
- 左右傾斜
- 上下傾斜
- 不同距離
- 避免所有照片都正對鏡頭

多相機外參時，要確保需要建立外參關係的相機能觀測到共同標定板或具有可連接的重疊觀測鏈。

## 重要限制

1. 本工具是軟體 snapshot，不是硬體 trigger。
2. 六顆相機各自持續擷取，按下儲存時複製每顆的 latest frame。
3. `max_software_snapshot_skew_ms` 只是軟體接收時間差，不代表真實 sensor exposure time 差。
4. 如果要做高精度多相機外參或時序敏感融合，建議使用硬體同步或可提供 sensor timestamp 的方案。
