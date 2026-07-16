#!/usr/bin/env python3
"""Save six camera images and one VLP-16 PCD every time ENTER is pressed."""

from __future__ import annotations

import argparse
import json
import math
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Deque, Dict, List, Sequence

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2

CAMERA_TOPICS = [f"/camera/device_{i}/image_raw" for i in range(6)]
LIDAR_TOPIC = "/velodyne_points"


@dataclass(frozen=True)
class StampedMessage:
    stamp_ns: int
    msg: object


@dataclass
class PendingTrigger:
    sequence: int
    target_ros_ns: int
    ready_monotonic: float


@dataclass
class SnapshotJob:
    sequence: int
    request_ros_ns: int
    lidar_stamp_ns: int
    lidar_msg: PointCloud2
    camera_items: Dict[str, StampedMessage]


def stamp_ns(msg: object) -> int:
    stamp = msg.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def nearest(buffer: Sequence[StampedMessage], target_ns: int):
    if not buffer:
        return None
    return min(buffer, key=lambda item: abs(item.stamp_ns - target_ns))


def image_to_bgr(msg: Image) -> np.ndarray:
    enc = msg.encoding.lower()
    raw = np.frombuffer(msg.data, dtype=np.uint8)

    if enc in ("bgr8", "rgb8"):
        row_bytes = msg.width * 3
        rows = raw.reshape(msg.height, msg.step)
        image = rows[:, :row_bytes].reshape(msg.height, msg.width, 3)
        if enc == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return np.ascontiguousarray(image)

    if enc in ("bgra8", "rgba8"):
        row_bytes = msg.width * 4
        rows = raw.reshape(msg.height, msg.step)
        image = rows[:, :row_bytes].reshape(msg.height, msg.width, 4)
        code = cv2.COLOR_BGRA2BGR if enc == "bgra8" else cv2.COLOR_RGBA2BGR
        return cv2.cvtColor(image, code)

    if enc == "mono8":
        rows = raw.reshape(msg.height, msg.step)
        mono = rows[:, :msg.width]
        return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

    if enc in ("yuv422", "uyvy", "yuv422_uyvy"):
        row_bytes = msg.width * 2
        rows = raw.reshape(msg.height, msg.step)
        yuv = rows[:, :row_bytes].reshape(msg.height, msg.width, 2)
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_UYVY)

    if enc in ("yuyv", "yuy2", "yuv422_yuy2"):
        row_bytes = msg.width * 2
        rows = raw.reshape(msg.height, msg.step)
        yuv = rows[:, :row_bytes].reshape(msg.height, msg.width, 2)
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUY2)

    raise ValueError(f"Unsupported image encoding: {msg.encoding}")


PCD_TYPES = {
    PointField.INT8: (1, "I"),
    PointField.UINT8: (1, "U"),
    PointField.INT16: (2, "I"),
    PointField.UINT16: (2, "U"),
    PointField.INT32: (4, "I"),
    PointField.UINT32: (4, "U"),
    PointField.FLOAT32: (4, "F"),
    PointField.FLOAT64: (8, "F"),
}


def _point_value(record, field_name: str, index: int):
    dtype = getattr(record, "dtype", None)
    names = getattr(dtype, "names", None)
    if names and field_name in names:
        return record[field_name]
    return record[index]


def write_ascii_pcd(msg: PointCloud2, path: Path) -> int:
    available = {field.name: field for field in msg.fields}
    fields = [name for name in ("x", "y", "z", "intensity", "ring") if name in available]
    for required in ("x", "y", "z"):
        if required not in fields:
            raise ValueError(f"PointCloud2 missing required field: {required}")

    sizes, types, counts = [], [], []
    for name in fields:
        field = available[name]
        size, pcd_type = PCD_TYPES[field.datatype]
        sizes.append(str(size))
        types.append(pcd_type)
        counts.append(str(field.count))

    rows = []
    for record in point_cloud2.read_points(msg, field_names=fields, skip_nans=True):
        row = [_point_value(record, name, i) for i, name in enumerate(fields)]
        if all(math.isfinite(float(row[i])) for i in range(3)):
            rows.append(row)

    with path.open("w", encoding="utf-8") as fp:
        fp.write("# .PCD v0.7 - Point Cloud Data file format\n")
        fp.write("VERSION 0.7\n")
        fp.write(f"FIELDS {' '.join(fields)}\n")
        fp.write(f"SIZE {' '.join(sizes)}\n")
        fp.write(f"TYPE {' '.join(types)}\n")
        fp.write(f"COUNT {' '.join(counts)}\n")
        fp.write(f"WIDTH {len(rows)}\n")
        fp.write("HEIGHT 1\n")
        fp.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        fp.write(f"POINTS {len(rows)}\n")
        fp.write("DATA ascii\n")
        for row in rows:
            out = []
            for i, name in enumerate(fields):
                _, pcd_type = PCD_TYPES[available[name].datatype]
                out.append(str(int(row[i])) if pcd_type in ("I", "U") else f"{float(row[i]):.7g}")
            fp.write(" ".join(out) + "\n")
    return len(rows)


class SnapshotNode(Node):
    def __init__(self, output: Path, tolerance_ms: float, settle_ms: float, cache_size: int):
        super().__init__("sensor_snapshot_capture")
        self.camera_topics = CAMERA_TOPICS
        self.lidar_topic = LIDAR_TOPIC
        self.tolerance_ns = int(tolerance_ms * 1_000_000)
        self.settle_sec = settle_ms / 1000.0

        session = datetime.now().strftime("session_%Y%m%d_%H%M%S")
        self.session_dir = output / session
        self.session_dir.mkdir(parents=True, exist_ok=True)

        self.lock = threading.Lock()
        self.camera_buffers: Dict[str, Deque[StampedMessage]] = {
            topic: deque(maxlen=cache_size) for topic in self.camera_topics
        }
        self.lidar_buffer: Deque[StampedMessage] = deque(maxlen=cache_size)

        self.trigger_queue = queue.Queue()
        self.save_queue = queue.Queue()
        self.pending: List[PendingTrigger] = []
        self.sequence = 0
        self.shutdown_requested = threading.Event()

        self.camera_subscriptions = []
        for topic in self.camera_topics:
            self.camera_subscriptions.append(
                self.create_subscription(
                    Image,
                    topic,
                    lambda msg, t=topic: self._camera_cb(t, msg),
                    qos_profile_sensor_data,
                )
            )

        self.lidar_sub = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self._lidar_cb,
            qos_profile_sensor_data,
        )

        self.poll_timer = self.create_timer(0.02, self._poll)
        self.status_timer = self.create_timer(2.0, self._status)

        self.input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self.worker_thread = threading.Thread(target=self._save_worker, daemon=False)
        self.input_thread.start()
        self.worker_thread.start()

        self.get_logger().info(f"Output session: {self.session_dir}")
        self.get_logger().info("Press ENTER to save one synchronized set; q + ENTER to quit.")

    def _camera_cb(self, topic: str, msg: Image):
        with self.lock:
            self.camera_buffers[topic].append(StampedMessage(stamp_ns(msg), msg))

    def _lidar_cb(self, msg: PointCloud2):
        with self.lock:
            self.lidar_buffer.append(StampedMessage(stamp_ns(msg), msg))

    def _input_loop(self):
        while rclpy.ok() and not self.shutdown_requested.is_set():
            try:
                line = input()
            except (EOFError, KeyboardInterrupt):
                return
            if line.strip().lower() in ("q", "quit", "exit"):
                self.shutdown_requested.set()
                return
            self.trigger_queue.put(True)

    def _poll(self):
        while True:
            try:
                self.trigger_queue.get_nowait()
            except queue.Empty:
                break
            self.sequence += 1
            self.pending.append(
                PendingTrigger(
                    sequence=self.sequence,
                    target_ros_ns=self.get_clock().now().nanoseconds,
                    ready_monotonic=time.monotonic() + self.settle_sec,
                )
            )
            self.get_logger().info(f"[CAPTURE {self.sequence:04d}] trigger received")

        now = time.monotonic()
        ready = [item for item in self.pending if item.ready_monotonic <= now]
        self.pending = [item for item in self.pending if item.ready_monotonic > now]
        for trigger in ready:
            job = self._select_job(trigger)
            if job is not None:
                self.save_queue.put(job)

        if self.shutdown_requested.is_set():
            rclpy.shutdown()

    def _select_job(self, trigger: PendingTrigger):
        with self.lock:
            lidar = list(self.lidar_buffer)
            cameras = {topic: list(buf) for topic, buf in self.camera_buffers.items()}

        lidar_item = nearest(lidar, trigger.target_ros_ns)
        if lidar_item is None:
            self.get_logger().error(f"[CAPTURE {trigger.sequence:04d}] no LiDAR data")
            return None
        if abs(lidar_item.stamp_ns - trigger.target_ros_ns) > self.tolerance_ns:
            self.get_logger().error(f"[CAPTURE {trigger.sequence:04d}] LiDAR too far from trigger")
            return None

        selected = {}
        for topic in self.camera_topics:
            item = nearest(cameras[topic], lidar_item.stamp_ns)
            if item is None:
                self.get_logger().error(f"[CAPTURE {trigger.sequence:04d}] no data: {topic}")
                return None
            delta = abs(item.stamp_ns - lidar_item.stamp_ns)
            if delta > self.tolerance_ns:
                self.get_logger().error(
                    f"[CAPTURE {trigger.sequence:04d}] sync fail {topic}: {delta/1e6:.1f} ms"
                )
                return None
            selected[topic] = item

        return SnapshotJob(
            sequence=trigger.sequence,
            request_ros_ns=trigger.target_ros_ns,
            lidar_stamp_ns=lidar_item.stamp_ns,
            lidar_msg=lidar_item.msg,
            camera_items=selected,
        )

    def _save_worker(self):
        while True:
            job = self.save_queue.get()
            try:
                if job is None:
                    return
                try:
                    self._save(job)
                except Exception as exc:
                    self.get_logger().error(f"[CAPTURE {job.sequence:04d}] save failed: {exc}")
            finally:
                self.save_queue.task_done()

    def _save(self, job: SnapshotJob):
        folder = self.session_dir / f"capture_{job.sequence:04d}_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"
        folder.mkdir(parents=True)

        camera_meta = {}
        for i, topic in enumerate(self.camera_topics):
            item = job.camera_items[topic]
            msg: Image = item.msg
            filename = f"camera_{i}.png"
            image = image_to_bgr(msg)
            if not cv2.imwrite(str(folder / filename), image):
                raise RuntimeError(f"Failed to write {filename}")
            camera_meta[f"camera_{i}"] = {
                "topic": topic,
                "file": filename,
                "stamp_ns": item.stamp_ns,
                "delta_to_lidar_ms": (item.stamp_ns - job.lidar_stamp_ns) / 1e6,
                "frame_id": msg.header.frame_id,
                "encoding": msg.encoding,
                "width": int(msg.width),
                "height": int(msg.height),
            }

        points = write_ascii_pcd(job.lidar_msg, folder / "velodyne.pcd")
        stamps = [item.stamp_ns for item in job.camera_items.values()] + [job.lidar_stamp_ns]

        metadata = {
            "capture_sequence": job.sequence,
            "request_ros_time_ns": job.request_ros_ns,
            "lidar": {
                "topic": self.lidar_topic,
                "file": "velodyne.pcd",
                "stamp_ns": job.lidar_stamp_ns,
                "frame_id": job.lidar_msg.header.frame_id,
                "source_width": int(job.lidar_msg.width),
                "source_height": int(job.lidar_msg.height),
                "points_saved": points,
                "fields": [f.name for f in job.lidar_msg.fields],
            },
            "cameras": camera_meta,
            "sync": {
                "anchor": "lidar",
                "tolerance_ms": self.tolerance_ns / 1e6,
                "capture_span_ms": (max(stamps) - min(stamps)) / 1e6,
            },
        }
        (folder / "metadata.json").write_text(
            json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8"
        )

        max_delta = max(abs(item.stamp_ns - job.lidar_stamp_ns) for item in job.camera_items.values()) / 1e6
        self.get_logger().info(
            f"[CAPTURE {job.sequence:04d}] SAVED {folder} | points={points} | max delta={max_delta:.1f} ms"
        )

    def _status(self):
        with self.lock:
            ready = sum(bool(buf) for buf in self.camera_buffers.values())
            lidar_ok = bool(self.lidar_buffer)
        self.get_logger().info(
            f"[STATUS] cameras={ready}/6 lidar={'OK' if lidar_ok else 'WAIT'} writer_queue={self.save_queue.qsize()}"
        )

    def stop(self):
        self.shutdown_requested.set()
        self.save_queue.put(None)
        self.save_queue.join()
        self.worker_thread.join(timeout=5.0)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        default="/colcon_ws/tools/sensor_snapshot/dataset",
        help="Output root directory",
    )
    parser.add_argument("--tolerance-ms", type=float, default=120.0)
    parser.add_argument("--settle-ms", type=float, default=180.0)
    parser.add_argument("--cache-size", type=int, default=120)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = SnapshotNode(
        output=Path(args.output),
        tolerance_ms=args.tolerance_ms,
        settle_ms=args.settle_ms,
        cache_size=args.cache_size,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
