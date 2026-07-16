#!/usr/bin/env python3
"""Small Xsens Xbus / MTData2 parser.

This parser intentionally implements only the pieces needed by the Prius MTi-630
bringup path:
  * Xbus frame synchronization and checksum validation
  * MTData2 payload parsing
  * common calibrated data identifiers used by MTi-630

It does not depend on the Xsens MT SDK. Configure the MTi device with MT Manager
or another Xsens tool to stream MTData2 over Xbus before running this driver.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import struct
import time
from typing import Dict, Iterable, Optional, Tuple


PREAMBLE = 0xFA
MID_MTDATA2 = 0x36
LEN_EXTENDED = 0xFF

# Common MTData2 data identifiers. The lower nibble may encode format details,
# so the parser compares using `data_id & 0xFFF0`.
DID_PACKET_COUNTER = 0x1020
DID_SAMPLE_TIME_FINE = 0x1060
DID_SAMPLE_TIME_COARSE = 0x1070
DID_QUATERNION = 0x2010
DID_EULER = 0x2030
DID_ACCELERATION = 0x4020
DID_FREE_ACCELERATION = 0x4030
DID_ACCELERATION_HR = 0x4040
DID_RATE_OF_TURN = 0x8020
DID_RATE_OF_TURN_HR = 0x8040
DID_MAGNETIC_FIELD = 0xC020
DID_STATUS_BYTE = 0xE010
DID_STATUS_WORD = 0xE020
DID_TEMPERATURE = 0x0810


def _base_id(data_id: int) -> int:
    return data_id & 0xFFF0


def _unpack_f32_vec(data: bytes, count: int) -> Tuple[float, ...]:
    if len(data) < 4 * count:
        raise ValueError(f"need {4 * count} bytes, got {len(data)}")
    return struct.unpack(">" + "f" * count, data[: 4 * count])


def _unpack_u16(data: bytes) -> int:
    if len(data) < 2:
        raise ValueError(f"need 2 bytes, got {len(data)}")
    return struct.unpack(">H", data[:2])[0]


def _unpack_u32(data: bytes) -> int:
    if len(data) < 4:
        raise ValueError(f"need 4 bytes, got {len(data)}")
    return struct.unpack(">I", data[:4])[0]


@dataclass
class XbusFrame:
    bid: int
    mid: int
    payload: bytes
    rx_time: float = field(default_factory=time.time)


@dataclass
class MTData2Packet:
    rx_time: float
    packet_counter: Optional[int] = None
    sample_time_fine: Optional[int] = None
    sample_time_coarse: Optional[int] = None
    quaternion_wxyz: Optional[Tuple[float, float, float, float]] = None
    acceleration_xyz: Optional[Tuple[float, float, float]] = None
    free_acceleration_xyz: Optional[Tuple[float, float, float]] = None
    rate_of_turn_xyz: Optional[Tuple[float, float, float]] = None
    magnetic_field_xyz: Optional[Tuple[float, float, float]] = None
    status_word: Optional[int] = None
    status_byte: Optional[int] = None
    temperature_c: Optional[float] = None
    unknown_items: Dict[int, bytes] = field(default_factory=dict)

    @property
    def has_minimum_imu(self) -> bool:
        return self.rate_of_turn_xyz is not None and (
            self.acceleration_xyz is not None or self.free_acceleration_xyz is not None
        )


class XbusParser:
    """Incremental Xbus frame parser.

    Feed bytes with :meth:`feed` and receive validated :class:`XbusFrame` items.
    """

    def __init__(self, max_payload_len: int = 4096) -> None:
        self._buf = bytearray()
        self.max_payload_len = int(max_payload_len)
        self.frames_seen = 0
        self.checksum_errors = 0
        self.bytes_dropped = 0

    @staticmethod
    def checksum_ok(frame_without_preamble: bytes) -> bool:
        return (sum(frame_without_preamble) & 0xFF) == 0

    def feed(self, data: bytes) -> Iterable[XbusFrame]:
        if not data:
            return []
        self._buf.extend(data)
        out = []

        while True:
            # Search for preamble.
            try:
                idx = self._buf.index(PREAMBLE)
            except ValueError:
                self.bytes_dropped += len(self._buf)
                self._buf.clear()
                break

            if idx > 0:
                self.bytes_dropped += idx
                del self._buf[:idx]

            # Need at least preamble + bid + mid + len + checksum.
            if len(self._buf) < 5:
                break

            bid = self._buf[1]
            mid = self._buf[2]
            length_byte = self._buf[3]

            if length_byte == LEN_EXTENDED:
                if len(self._buf) < 7:
                    break
                payload_len = (self._buf[4] << 8) | self._buf[5]
                header_len = 6
            else:
                payload_len = length_byte
                header_len = 4

            if payload_len > self.max_payload_len:
                # Not a sane frame. Drop the preamble and resync.
                self.bytes_dropped += 1
                del self._buf[0]
                continue

            total_len = header_len + payload_len + 1  # + checksum, includes preamble
            if len(self._buf) < total_len:
                break

            raw = bytes(self._buf[:total_len])

            # Checksum excludes preamble but includes BID, MID, LEN, payload and CS.
            # On checksum failure, drop ONLY the current preamble and resynchronise.
            # Dropping the whole candidate frame is risky when the stream contains
            # non-Xbus bytes or a false 0xFA inside another protocol.
            if not self.checksum_ok(raw[1:]):
                self.checksum_errors += 1
                self.bytes_dropped += 1
                del self._buf[0]
                continue

            del self._buf[:total_len]
            payload = raw[header_len : header_len + payload_len]
            self.frames_seen += 1
            out.append(XbusFrame(bid=bid, mid=mid, payload=payload, rx_time=time.time()))

        return out


def parse_mtdata2_payload(payload: bytes, rx_time: Optional[float] = None) -> MTData2Packet:
    pkt = MTData2Packet(rx_time=time.time() if rx_time is None else rx_time)
    i = 0
    n = len(payload)

    while i + 3 <= n:
        data_id = (payload[i] << 8) | payload[i + 1]
        size = payload[i + 2]
        i += 3
        if i + size > n:
            # Truncated item; keep the raw remainder for debugging and stop.
            pkt.unknown_items[data_id] = payload[i:]
            break

        data = bytes(payload[i : i + size])
        i += size
        bid = _base_id(data_id)

        try:
            if bid == DID_PACKET_COUNTER:
                pkt.packet_counter = _unpack_u16(data)
            elif bid == DID_SAMPLE_TIME_FINE:
                pkt.sample_time_fine = _unpack_u32(data)
            elif bid == DID_SAMPLE_TIME_COARSE:
                pkt.sample_time_coarse = _unpack_u32(data)
            elif bid == DID_QUATERNION:
                # Xsens order is q0, q1, q2, q3 = w, x, y, z.
                pkt.quaternion_wxyz = _unpack_f32_vec(data, 4)  # type: ignore[assignment]
            elif bid == DID_ACCELERATION:
                pkt.acceleration_xyz = _unpack_f32_vec(data, 3)  # type: ignore[assignment]
            elif bid == DID_FREE_ACCELERATION:
                pkt.free_acceleration_xyz = _unpack_f32_vec(data, 3)  # type: ignore[assignment]
            elif bid == DID_RATE_OF_TURN:
                pkt.rate_of_turn_xyz = _unpack_f32_vec(data, 3)  # type: ignore[assignment]
            elif bid == DID_MAGNETIC_FIELD:
                pkt.magnetic_field_xyz = _unpack_f32_vec(data, 3)  # type: ignore[assignment]
            elif bid == DID_STATUS_BYTE:
                pkt.status_byte = int(data[0]) if data else None
            elif bid == DID_STATUS_WORD:
                pkt.status_word = _unpack_u32(data)
            elif bid == DID_TEMPERATURE:
                pkt.temperature_c = _unpack_f32_vec(data, 1)[0]
            else:
                pkt.unknown_items[data_id] = data
        except Exception:
            pkt.unknown_items[data_id] = data

    return pkt


def make_xbus_message(mid: int, payload: bytes = b"", bid: int = 0xFF) -> bytes:
    """Create a standard-length Xbus message.

    This helper is included for future configuration commands. The first version of
    the node does not send configuration by default.
    """
    if len(payload) > 254:
        raise ValueError("standard Xbus message payload must be <= 254 bytes")
    body = bytes([bid & 0xFF, mid & 0xFF, len(payload) & 0xFF]) + payload
    cs = (-sum(body)) & 0xFF
    return bytes([PREAMBLE]) + body + bytes([cs])
