#!/usr/bin/env python3
from __future__ import annotations

import math
import threading
import time
from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import UInt32

try:
    import serial
    from serial import SerialException
except Exception as exc:  # pragma: no cover
    serial = None
    SerialException = Exception
    _SERIAL_IMPORT_ERROR = exc
else:
    _SERIAL_IMPORT_ERROR = None

from .xbus import MID_MTDATA2, XbusParser, parse_mtdata2_payload, MTData2Packet


def _declare_list_param(node: Node, name: str, default: Sequence[float]) -> List[float]:
    node.declare_parameter(name, list(default))
    value = node.get_parameter(name).value
    return [float(v) for v in value]


def _diag_cov(stddev: Sequence[float]) -> List[float]:
    sx, sy, sz = [float(x) for x in stddev]
    return [
        sx * sx, 0.0, 0.0,
        0.0, sy * sy, 0.0,
        0.0, 0.0, sz * sz,
    ]


def _remap_vec(vec: Tuple[float, float, float], axis_map: Sequence[int], axis_sign: Sequence[float]) -> Tuple[float, float, float]:
    src = [float(vec[0]), float(vec[1]), float(vec[2])]
    return (
        float(axis_sign[0]) * src[int(axis_map[0])],
        float(axis_sign[1]) * src[int(axis_map[1])],
        float(axis_sign[2]) * src[int(axis_map[2])],
    )


def _normalize_quat_wxyz(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    w, x, y, z = [float(v) for v in q]
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1e-12:
        return 1.0, 0.0, 0.0, 0.0
    return w / norm, x / norm, y / norm, z / norm


class MTi630Node(Node):
    def __init__(self) -> None:
        super().__init__('mti630_node')

        if _SERIAL_IMPORT_ERROR is not None:
            raise RuntimeError(
                'pyserial is not available. Install python3-serial or pip install pyserial.'
            ) from _SERIAL_IMPORT_ERROR

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'mti630_link')
        self.declare_parameter('imu_topic', '/sensing/imu/mti630/imu_raw')
        self.declare_parameter('mag_topic', '/sensing/imu/mti630/mag')
        self.declare_parameter('status_topic', '/sensing/imu/mti630/status_word')
        self.declare_parameter('diagnostics_topic', '/diagnostics')
        self.declare_parameter('publish_magnetic_field', True)
        self.declare_parameter('publish_status_word', True)
        self.declare_parameter('publish_diagnostics', True)
        self.declare_parameter('use_device_timestamp', False)
        self.declare_parameter('read_chunk_size', 512)
        self.declare_parameter('serial_timeout_sec', 0.05)
        self.declare_parameter('reconnect_sec', 1.0)
        self.declare_parameter('warn_if_no_packet_sec', 1.0)
        self.declare_parameter('publish_rate_limit_hz', 0.0)
        self.declare_parameter('allow_free_acceleration_fallback', False)

        # Axis remap only affects vector data. Quaternion remap is intentionally not
        # implemented; mount/configure the MTi frame to match ROS base frame when
        # publishing orientation.
        self.declare_parameter('axis_map', [0, 1, 2])
        self.declare_parameter('axis_sign', [1.0, 1.0, 1.0])
        self.declare_parameter('publish_orientation', True)
        self.declare_parameter('disable_orientation_when_axis_remap', True)

        self.orientation_stddev = _declare_list_param(self, 'orientation_stddev', [0.02, 0.02, 0.05])
        self.angular_velocity_stddev = _declare_list_param(self, 'angular_velocity_stddev', [0.003, 0.003, 0.003])
        self.linear_acceleration_stddev = _declare_list_param(self, 'linear_acceleration_stddev', [0.05, 0.05, 0.05])
        self.magnetic_field_stddev = _declare_list_param(self, 'magnetic_field_stddev', [1.0e-6, 1.0e-6, 1.0e-6])

        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.mag_topic = str(self.get_parameter('mag_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.diag_topic = str(self.get_parameter('diagnostics_topic').value)
        self.read_chunk_size = int(self.get_parameter('read_chunk_size').value)
        self.serial_timeout_sec = float(self.get_parameter('serial_timeout_sec').value)
        self.reconnect_sec = float(self.get_parameter('reconnect_sec').value)
        self.warn_if_no_packet_sec = float(self.get_parameter('warn_if_no_packet_sec').value)
        self.publish_rate_limit_hz = float(self.get_parameter('publish_rate_limit_hz').value)
        self.allow_free_accel_fallback = bool(self.get_parameter('allow_free_acceleration_fallback').value)
        self.axis_map = [int(v) for v in self.get_parameter('axis_map').value]
        self.axis_sign = [float(v) for v in self.get_parameter('axis_sign').value]
        self.publish_orientation = bool(self.get_parameter('publish_orientation').value)
        self.disable_orientation_when_axis_remap = bool(self.get_parameter('disable_orientation_when_axis_remap').value)

        if len(self.axis_map) != 3 or sorted(self.axis_map) != [0, 1, 2]:
            raise ValueError('axis_map must be a permutation of [0, 1, 2]')
        if len(self.axis_sign) != 3:
            raise ValueError('axis_sign must contain exactly 3 values')

        self.axis_remap_is_identity = self.axis_map == [0, 1, 2] and all(abs(s - 1.0) < 1e-9 for s in self.axis_sign)
        if not self.axis_remap_is_identity and self.disable_orientation_when_axis_remap:
            self.get_logger().warn(
                'axis remap/sign is not identity. Quaternion remapping is not implemented, '
                'so orientation will be marked unavailable unless publish_orientation is false.'
            )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, qos)
        self.mag_pub = self.create_publisher(MagneticField, self.mag_topic, qos)
        self.status_pub = self.create_publisher(UInt32, self.status_topic, 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, self.diag_topic, 10)

        self.parser = XbusParser()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._serial = None

        self._last_packet_wall = 0.0
        self._last_publish_wall = 0.0
        self._packets_total = 0
        self._imu_total = 0
        self._mag_total = 0
        self._status_total = 0
        self._last_packet_counter: Optional[int] = None
        self._packet_counter_gap_total = 0
        self._last_status_word: Optional[int] = None
        self._last_error = ''
        self._connected = False
        self._free_accel_warned = False

        self._diagnostic_timer = self.create_timer(1.0, self._publish_diagnostics)
        self._thread = threading.Thread(target=self._reader_loop, name='mti630_serial_reader', daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'MTi-630 Xbus driver starting: port={self.port}, baudrate={self.baudrate}, frame_id={self.frame_id}, topic={self.imu_topic}'
        )

    def destroy_node(self) -> bool:
        self._stop_event.set()
        try:
            if self._serial is not None:
                self._serial.close()
        except Exception:
            pass
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        return super().destroy_node()

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._connect_and_read()
            except Exception as exc:
                self._connected = False
                self._last_error = str(exc)
                self.get_logger().warn(f'MTi serial read failed: {exc}. Reconnecting in {self.reconnect_sec:.1f}s')
                try:
                    if self._serial is not None:
                        self._serial.close()
                except Exception:
                    pass
                self._serial = None
                self._stop_event.wait(self.reconnect_sec)

    def _connect_and_read(self) -> None:
        self.get_logger().info(f'Opening MTi serial port {self.port} @ {self.baudrate}')
        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.serial_timeout_sec,
            write_timeout=self.serial_timeout_sec,
        )
        self._connected = True
        self._last_error = ''

        while not self._stop_event.is_set():
            data = self._serial.read(self.read_chunk_size)
            now = time.time()
            frames = self.parser.feed(data)
            for frame in frames:
                if frame.mid != MID_MTDATA2:
                    continue
                pkt = parse_mtdata2_payload(frame.payload, rx_time=now)
                self._handle_packet(pkt)

            if self.warn_if_no_packet_sec > 0.0:
                if self._last_packet_wall > 0 and now - self._last_packet_wall > self.warn_if_no_packet_sec:
                    self.get_logger().warn(
                        f'No valid MTData2 packet for {now - self._last_packet_wall:.2f}s. '
                        'Check baudrate, Xbus MTData2 output, and cable.',
                        throttle_duration_sec=2.0,
                    )

    def _stamp(self, pkt: MTData2Packet):
        # The first version uses ROS receive time by default. Device timestamp is
        # intentionally not converted to ROS time until a time-sync policy is fixed.
        return self.get_clock().now().to_msg()

    def _rate_limited(self, now_wall: float) -> bool:
        if self.publish_rate_limit_hz <= 0.0:
            return False
        min_dt = 1.0 / self.publish_rate_limit_hz
        return (now_wall - self._last_publish_wall) < min_dt

    def _handle_packet(self, pkt: MTData2Packet) -> None:
        now_wall = time.time()
        self._last_packet_wall = now_wall
        self._packets_total += 1

        if pkt.packet_counter is not None:
            if self._last_packet_counter is not None:
                expected = (self._last_packet_counter + 1) & 0xFFFF
                if pkt.packet_counter != expected:
                    self._packet_counter_gap_total += 1
            self._last_packet_counter = pkt.packet_counter

        if pkt.status_word is not None:
            self._last_status_word = pkt.status_word
            if bool(self.get_parameter('publish_status_word').value):
                msg = UInt32()
                msg.data = int(pkt.status_word)
                self.status_pub.publish(msg)
                self._status_total += 1

        if pkt.magnetic_field_xyz is not None and bool(self.get_parameter('publish_magnetic_field').value):
            mag = MagneticField()
            mag.header.stamp = self._stamp(pkt)
            mag.header.frame_id = self.frame_id
            mx, my, mz = _remap_vec(pkt.magnetic_field_xyz, self.axis_map, self.axis_sign)
            mag.magnetic_field.x = mx
            mag.magnetic_field.y = my
            mag.magnetic_field.z = mz
            mag.magnetic_field_covariance = _diag_cov(self.magnetic_field_stddev)
            self.mag_pub.publish(mag)
            self._mag_total += 1

        if not pkt.has_minimum_imu:
            return
        if self._rate_limited(now_wall):
            return

        accel = pkt.acceleration_xyz
        if accel is None and self.allow_free_accel_fallback:
            accel = pkt.free_acceleration_xyz
            if accel is not None and not self._free_accel_warned:
                self.get_logger().warn(
                    'Publishing FreeAcceleration as Imu.linear_acceleration. For Autoware raw IMU, prefer Acceleration 0x4020.'
                )
                self._free_accel_warned = True
        if accel is None or pkt.rate_of_turn_xyz is None:
            return

        imu = Imu()
        imu.header.stamp = self._stamp(pkt)
        imu.header.frame_id = self.frame_id

        publish_orientation = self.publish_orientation and pkt.quaternion_wxyz is not None
        if not self.axis_remap_is_identity and self.disable_orientation_when_axis_remap:
            publish_orientation = False

        if publish_orientation and pkt.quaternion_wxyz is not None:
            qw, qx, qy, qz = _normalize_quat_wxyz(pkt.quaternion_wxyz)
            imu.orientation.w = qw
            imu.orientation.x = qx
            imu.orientation.y = qy
            imu.orientation.z = qz
            imu.orientation_covariance = _diag_cov(self.orientation_stddev)
        else:
            # Per sensor_msgs/Imu convention, -1 in covariance[0] means no estimate.
            imu.orientation_covariance[0] = -1.0

        gx, gy, gz = _remap_vec(pkt.rate_of_turn_xyz, self.axis_map, self.axis_sign)
        ax, ay, az = _remap_vec(accel, self.axis_map, self.axis_sign)

        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        imu.angular_velocity_covariance = _diag_cov(self.angular_velocity_stddev)

        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.linear_acceleration_covariance = _diag_cov(self.linear_acceleration_stddev)

        self.imu_pub.publish(imu)
        self._last_publish_wall = now_wall
        self._imu_total += 1

    def _publish_diagnostics(self) -> None:
        if not bool(self.get_parameter('publish_diagnostics').value):
            return
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        now = time.time()
        age = now - self._last_packet_wall if self._last_packet_wall > 0.0 else float('inf')
        status = DiagnosticStatus()
        status.name = 'prius_mti630_driver'
        status.hardware_id = self.port
        if not self._connected:
            status.level = DiagnosticStatus.ERROR
            status.message = 'serial disconnected'
        elif age > max(self.warn_if_no_packet_sec, 1.0):
            status.level = DiagnosticStatus.WARN
            status.message = 'no recent MTData2 packet'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'ok'

        status.values = [
            KeyValue(key='port', value=self.port),
            KeyValue(key='baudrate', value=str(self.baudrate)),
            KeyValue(key='connected', value=str(self._connected)),
            KeyValue(key='last_packet_age_sec', value=f'{age:.3f}' if math.isfinite(age) else 'inf'),
            KeyValue(key='packets_total', value=str(self._packets_total)),
            KeyValue(key='imu_messages_total', value=str(self._imu_total)),
            KeyValue(key='mag_messages_total', value=str(self._mag_total)),
            KeyValue(key='status_messages_total', value=str(self._status_total)),
            KeyValue(key='xbus_frames_seen', value=str(self.parser.frames_seen)),
            KeyValue(key='xbus_checksum_errors', value=str(self.parser.checksum_errors)),
            KeyValue(key='xbus_bytes_dropped', value=str(self.parser.bytes_dropped)),
            KeyValue(key='packet_counter_gap_total', value=str(self._packet_counter_gap_total)),
            KeyValue(key='last_packet_counter', value=str(self._last_packet_counter)),
            KeyValue(key='last_status_word', value=str(self._last_status_word)),
            KeyValue(key='last_error', value=self._last_error),
        ]
        msg.status.append(status)
        self.diag_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MTi630Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
