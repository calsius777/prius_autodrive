#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Prius CAN Probe - READ ONLY

Purpose:
  - Receive CAN frames through Linux SocketCAN.
  - Decode selected Toyota Prius / ACE428 vehicle signals.
  - Log both raw and decoded CSV for later analysis.
  - Never transmit CAN frames.

Tested workflow:
  python3 prius_can_probe.py --interface can0 --steer-format i12_old \
      --duration 100 --summary-period 1.0 --log-dir ./logs

Important:
  This program is read-only. It does not call send(), cansend, or any TX API.
"""

from __future__ import annotations

import argparse
import csv
import datetime as _dt
import math
import os
import select
import signal
import socket
import struct
import sys
import time
from typing import Any, Dict, Iterable, List, Optional, Tuple


# Linux CAN constants ---------------------------------------------------------
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF
CAN_RAW = 1

# struct can_frame: can_id(uint32), len(uint8), padding(3), data(8)
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)


# Frame names -----------------------------------------------------------------
FRAME_NAMES: Dict[int, str] = {
    # Vehicle feedback / status
    0x0B4: "SPEED",
    0x226: "BRAKE_MODULE",
    0x3B7: "ESP_CONTROL",
    0x025: "STEER_ANGLE_SENSOR_D",
    0x245: "GAS_PEDAL",
    0x1AA: "ABS_CONTROL",
    0x614: "STEERING_LEVERS",
    0x0AA: "WHEEL_SPEEDS",
    0x203: "WIRE_CONTROLLER_STATUS",

    # IMU IDs from the Prius CAN teaching material. Scaling is not finalized.
    0x022: "IMU_EULERANGLES_RAW",
    0x032: "IMU_RATEOFTURN_RAW",
    0x034: "IMU_ACCELERATION_RAW",

    # Seen often in your bus captures, not finalized yet.
    0x021: "UNKNOWN_0x021",
    0x041: "UNKNOWN_0x041",
    0x3BC: "UNKNOWN_0x3BC",

    # Control command IDs. This probe only monitors them if already present.
    0x065: "EPS_COMMAND_MONITOR_ONLY",
    0x075: "THROTTLE_COMMAND_MONITOR_ONLY",
    0x068: "BRAKE_COMMAND_MONITOR_ONLY",
    0x086: "BRAKE_COMMAND_0x086_MONITOR_ONLY",
}

# Object detection range from protocol table
for _cid in range(0x0E1, 0x0F0):
    FRAME_NAMES.setdefault(_cid, f"OBJECT_DETECTION_0x{_cid:03X}")

# Suspension range from protocol table
for _cid in range(0x0F1, 0x0F6):
    FRAME_NAMES.setdefault(_cid, f"SUSPENSION_0x{_cid:03X}")


# Utility ---------------------------------------------------------------------
def now_iso() -> str:
    return _dt.datetime.now().isoformat(timespec="milliseconds")


def data_hex(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def can_id_to_str(can_id: int) -> str:
    return f"0x{can_id:03X}" if can_id <= 0x7FF else f"0x{can_id:08X}"


def s16_be(data: bytes, idx: int = 0) -> int:
    value = (data[idx] << 8) | data[idx + 1]
    if value >= 0x8000:
        value -= 0x10000
    return value


def s16_le(data: bytes, idx: int = 0) -> int:
    value = data[idx] | (data[idx + 1] << 8)
    if value >= 0x8000:
        value -= 0x10000
    return value


def u16_be(data: bytes, idx: int = 0) -> int:
    return (data[idx] << 8) | data[idx + 1]


def u16_le(data: bytes, idx: int = 0) -> int:
    return data[idx] | (data[idx + 1] << 8)


def safe_fmt(value: Any, fmt: str = ".2f", none_text: str = "--") -> str:
    if value is None:
        return none_text
    try:
        return format(float(value), fmt)
    except Exception:
        return str(value)


# Decoders --------------------------------------------------------------------
def decode_speed_0x0b4(data: bytes) -> Dict[str, Any]:
    """0x0B4 SPEED: Byte5~6 big-endian, scale 0.01 km/h."""
    if len(data) < 7:
        return {}
    speed_raw = u16_be(data, 5)
    speed_kmh = speed_raw * 0.01
    return {
        "speed_raw": speed_raw,
        "speed_kmh": speed_kmh,
        "speed_mps": speed_kmh / 3.6,
    }


def decode_steer_0x025(data: bytes, steer_format: str) -> Dict[str, Any]:
    """
    0x025 steering wheel angle feedback.

    Valid modes:
      - i12_old: observed on current ACE428 Prius CAN logs.
                  raw = signed 12-bit from Byte0~1, scale 1.5 deg.
      - i16_be : signed 16-bit big-endian, scale 1.5 deg.
      - i16_le : signed 16-bit little-endian, scale 1.5 deg.
    """
    if len(data) < 2:
        return {}

    if steer_format == "i12_old":
        raw16 = u16_be(data, 0)
        raw = raw16 & 0x0FFF
        if raw >= 0x800:
            raw -= 0x1000
    elif steer_format == "i16_be":
        raw = s16_be(data, 0)
    elif steer_format == "i16_le":
        raw = s16_le(data, 0)
    else:
        raise ValueError(f"Unsupported steer format: {steer_format}")

    deg = raw * 1.5
    return {
        "steer_raw": raw,
        "steer_wheel_deg": deg,
        "steer_wheel_rad": math.radians(deg),
        "steer_format": steer_format,
    }


def decode_wheel_speeds_0x0aa(data: bytes) -> Dict[str, Any]:
    """
    0x0AA wheel speeds.

    Current best observed format from your 2026-07-14 logs:
      Byte0~1, Byte2~3, Byte4~5, Byte6~7 are four big-endian u16 values.
      Zero baseline is 0x1A6F.
      physical km/h = (raw - 0x1A6F) * 0.01

    Wheel order is intentionally not finalized. Keep W01/W23/W45/W67 until a
    controlled left/right turning dataset confirms FR/FL/RR/RL mapping.
    """
    if len(data) < 8:
        return {}

    baseline = 0x1A6F
    raw01 = u16_be(data, 0)
    raw23 = u16_be(data, 2)
    raw45 = u16_be(data, 4)
    raw67 = u16_be(data, 6)

    kmh01 = (raw01 - baseline) * 0.01
    kmh23 = (raw23 - baseline) * 0.01
    kmh45 = (raw45 - baseline) * 0.01
    kmh67 = (raw67 - baseline) * 0.01

    return {
        "wheel_01_raw": raw01,
        "wheel_23_raw": raw23,
        "wheel_45_raw": raw45,
        "wheel_67_raw": raw67,
        "wheel_01_kmh": kmh01,
        "wheel_23_kmh": kmh23,
        "wheel_45_kmh": kmh45,
        "wheel_67_kmh": kmh67,
        "wheel_01_mps": kmh01 / 3.6,
        "wheel_23_mps": kmh23 / 3.6,
        "wheel_45_mps": kmh45 / 3.6,
        "wheel_67_mps": kmh67 / 3.6,
        "wheel_fr_kmh": w01_kmh,
        "wheel_fl_kmh": w23_kmh,
        "wheel_rr_kmh": w45_kmh,
        "wheel_rl_kmh": w67_kmh,
    }


def decode_gas_0x245(data: bytes) -> Dict[str, Any]:
    """0x245 gas pedal and engine status candidate."""
    if len(data) < 4:
        return {}
    gas_raw = data[2]
    engine_code = (data[3] >> 4) & 0x0F
    engine_text = {
        0: "engine_off",
        1: "engine_stop",
        2: "engine_starting",
        3: "engine_running",
    }.get(engine_code, "unknown")
    return {
        "gas_pedal_raw": gas_raw,
        "engine_status_code": engine_code,
        "engine_status_text": engine_text,
    }


def decode_wire_status_0x203(data: bytes) -> Dict[str, Any]:
    """0x203 wire controller status candidate."""
    if len(data) < 8:
        return {}
    b7 = data[7]
    return {
        "wire_enable_candidate": (b7 & 0x80) != 0,
        "wire_status_byte7": b7,
        "wire_status_byte7_hex": f"0x{b7:02X}",
        "wire_status_byte4": data[4] if len(data) > 4 else None,
        "wiper_candidate_byte0": data[0] if len(data) > 0 else None,
    }


def decode_brake_0x226(data: bytes) -> Dict[str, Any]:
    """
    0x226 brake module candidate, revised from your raw logs.

    Current best observed candidate:
      Byte1~2 little-endian = brake pressure raw
      Byte3                = brake position raw
      Byte4 bit2           = brake pressed/status candidate
    """
    if len(data) < 5:
        return {}
    brake_pressure_raw = u16_le(data, 1)
    brake_position_raw = data[3]
    brake_pressed = (data[4] & 0x04) != 0
    
    return {
        "brake_pressure_raw": brake_pressure_raw,
        "brake_position_raw": brake_position_raw,
        "brake_pressed": brake_pressed,
        "brake_status_byte4": data[4],
    }


def decode_abs_0x1aa(data: bytes) -> Dict[str, Any]:
    """0x1AA ABS activation candidates."""
    if len(data) < 4:
        return {}
    return {
        "abs_fl_active_candidate": (data[2] & 0x40) != 0,
        "abs_fr_active_candidate": (data[2] & 0x04) != 0,
        "abs_rl_active_candidate": (data[3] & 0x40) != 0,
        "abs_rr_active_candidate": (data[3] & 0x04) != 0,
        "abs_byte2_hex": f"0x{data[2]:02X}",
        "abs_byte3_hex": f"0x{data[3]:02X}",
    }


def decode_esp_0x3b7(data: bytes) -> Dict[str, Any]:
    """0x3B7 ESP/TC/VSC raw candidates. Bit mapping is not finalized."""
    if len(data) < 8:
        return {}
    return {
        "esp_byte0_hex": f"0x{data[0]:02X}",
        "esp_byte1_hex": f"0x{data[1]:02X}",
        "esp_byte2_hex": f"0x{data[2]:02X}",
        "esp_byte3_hex": f"0x{data[3]:02X}",
        "esp_byte4_hex": f"0x{data[4]:02X}",
        "esp_byte5_hex": f"0x{data[5]:02X}",
        "esp_byte6_hex": f"0x{data[6]:02X}",
        "esp_byte7_hex": f"0x{data[7]:02X}",
    }


def decode_steering_levers_0x614(data: bytes) -> Dict[str, Any]:
    """
    0x614 steering levers.

    Observed correction from logs:
      Byte3 high nibble gives 1/2/3 turn code.
      Byte1 bit7 toggles during blinking, likely blink state.
    """
    if len(data) < 4:
        return {}
    turn_code = (data[3] >> 4) & 0x0F
    turn_text = {
        1: "left",
        2: "right",
        3: "none",
    }.get(turn_code, "unknown")
    return {
        "turn_signal_code": turn_code,
        "turn_signal_text": turn_text,
        "turn_blink_bit": (data[1] & 0x80) != 0,
        "steering_lever_byte1_hex": f"0x{data[1]:02X}",
        "steering_lever_byte3_hex": f"0x{data[3]:02X}",
    }


def decode_imu_raw_6bytes(data: bytes, prefix: str) -> Dict[str, Any]:
    """Generic 3-axis raw words for 6-byte IMU frames. Scaling not finalized."""
    if len(data) < 6:
        return {}
    return {
        f"{prefix}_raw_be_x": s16_be(data, 0),
        f"{prefix}_raw_be_y": s16_be(data, 2),
        f"{prefix}_raw_be_z": s16_be(data, 4),
        f"{prefix}_raw_le_x": s16_le(data, 0),
        f"{prefix}_raw_le_y": s16_le(data, 2),
        f"{prefix}_raw_le_z": s16_le(data, 4),
    }


def decode_command_monitor(can_id: int, data: bytes) -> Dict[str, Any]:
    """Decode command frames if they are already present on bus. No TX."""
    out: Dict[str, Any] = {
        "command_monitor_only": True,
        "command_enable_byte7": data[7] if len(data) >= 8 else None,
        "command_enable_candidate": (len(data) >= 8 and (data[7] & 0x80) != 0),
    }
    if can_id == 0x065 and len(data) >= 2:
        raw = s16_le(data, 0)
        out.update({
            "eps_command_raw_le": raw,
            "eps_command_steer_wheel_deg_candidate": raw * 1.5,
        })
    elif can_id == 0x075 and len(data) >= 1:
        out["throttle_command_raw"] = data[0]
    elif can_id in (0x068, 0x086) and len(data) >= 1:
        out["brake_command_raw"] = data[0]
    return out


def decode_frame(can_id: int, data: bytes, steer_format: str) -> Tuple[str, str, Dict[str, Any]]:
    """Return (frame_name, group, decoded_dict)."""
    name = FRAME_NAMES.get(can_id, "UNKNOWN")

    if can_id == 0x0B4:
        return name, "speed", decode_speed_0x0b4(data)
    if can_id == 0x025:
        return name, "steer", decode_steer_0x025(data, steer_format)
    if can_id == 0x0AA:
        return name, "wheel_speeds", decode_wheel_speeds_0x0aa(data)
    if can_id == 0x245:
        return name, "gas", decode_gas_0x245(data)
    if can_id == 0x203:
        return name, "wire_status", decode_wire_status_0x203(data)
    if can_id == 0x226:
        return name, "brake", decode_brake_0x226(data)
    if can_id == 0x1AA:
        return name, "abs", decode_abs_0x1aa(data)
    if can_id == 0x3B7:
        return name, "esp", decode_esp_0x3b7(data)
    if can_id == 0x614:
        return name, "steering_levers", decode_steering_levers_0x614(data)
    if can_id == 0x022:
        return name, "imu_euler_raw", decode_imu_raw_6bytes(data, "euler")
    if can_id == 0x032:
        return name, "imu_rate_raw", decode_imu_raw_6bytes(data, "rate")
    if can_id == 0x034:
        return name, "imu_accel_raw", decode_imu_raw_6bytes(data, "accel")
    if can_id in (0x065, 0x075, 0x068, 0x086):
        return name, "command_monitor", decode_command_monitor(can_id, data)

    return name, "unknown", {}


# State and summary -----------------------------------------------------------
def make_initial_state() -> Dict[str, Any]:
    return {
        "speed_kmh": None,
        "speed_mps": None,
        "steer_wheel_deg": None,
        "steer_wheel_rad": None,
        "steer_format": None,
        "wheel_01_kmh": None,
        "wheel_23_kmh": None,
        "wheel_45_kmh": None,
        "wheel_67_kmh": None,
        "wheel_01_mps": None,
        "wheel_23_mps": None,
        "wheel_45_mps": None,
        "wheel_67_mps": None,
        "gas_pedal_raw": None,
        "engine_status_text": None,
        "engine_status_code": None,
        "brake_pressure_raw": None,
        "brake_position_raw": None,
        "brake_pressed": None,
        "brake_status_byte4": None,
        "wire_enable_candidate": None,
        "wire_status_byte7": None,
        "wire_status_byte7_hex": None,
        "abs_fl_active_candidate": None,
        "abs_fr_active_candidate": None,
        "abs_rl_active_candidate": None,
        "abs_rr_active_candidate": None,
        "turn_signal_code": None,
        "turn_signal_text": None,
        "turn_blink_bit": None,
    }


def update_state(state: Dict[str, Any], group: str, decoded: Dict[str, Any]) -> None:
    """Update summary state. Uses .get() only; never throws KeyError."""
    if group == "speed":
        state["speed_kmh"] = decoded.get("speed_kmh")
        state["speed_mps"] = decoded.get("speed_mps")

    elif group == "steer":
        state["steer_wheel_deg"] = decoded.get("steer_wheel_deg")
        state["steer_wheel_rad"] = decoded.get("steer_wheel_rad")
        state["steer_format"] = decoded.get("steer_format")

    elif group == "wheel_speeds":
        state["wheel_01_kmh"] = decoded.get("wheel_01_kmh")
        state["wheel_23_kmh"] = decoded.get("wheel_23_kmh")
        state["wheel_45_kmh"] = decoded.get("wheel_45_kmh")
        state["wheel_67_kmh"] = decoded.get("wheel_67_kmh")
        state["wheel_01_mps"] = decoded.get("wheel_01_mps")
        state["wheel_23_mps"] = decoded.get("wheel_23_mps")
        state["wheel_45_mps"] = decoded.get("wheel_45_mps")
        state["wheel_67_mps"] = decoded.get("wheel_67_mps")

    elif group == "gas":
        state["gas_pedal_raw"] = decoded.get("gas_pedal_raw")
        state["engine_status_code"] = decoded.get("engine_status_code")
        state["engine_status_text"] = decoded.get("engine_status_text")

    elif group == "brake":
        state["brake_pressure_raw"] = decoded.get("brake_pressure_raw")
        state["brake_position_raw"] = decoded.get("brake_position_raw")
        state["brake_pressed"] = decoded.get("brake_pressed")
        state["brake_status_byte4"] = decoded.get("brake_status_byte4")

    elif group == "wire_status":
        state["wire_enable_candidate"] = decoded.get("wire_enable_candidate")
        state["wire_status_byte7"] = decoded.get("wire_status_byte7")
        state["wire_status_byte7_hex"] = decoded.get("wire_status_byte7_hex")

    elif group == "abs":
        state["abs_fl_active_candidate"] = decoded.get("abs_fl_active_candidate")
        state["abs_fr_active_candidate"] = decoded.get("abs_fr_active_candidate")
        state["abs_rl_active_candidate"] = decoded.get("abs_rl_active_candidate")
        state["abs_rr_active_candidate"] = decoded.get("abs_rr_active_candidate")

    elif group == "steering_levers":
        state["turn_signal_code"] = decoded.get("turn_signal_code")
        state["turn_signal_text"] = decoded.get("turn_signal_text")
        state["turn_blink_bit"] = decoded.get("turn_blink_bit")


def print_decoded_frame(ts: str, can_id: int, name: str, dlc: int, data: bytes, group: str, decoded: Dict[str, Any]) -> None:
    print(f"[{ts}] {can_id_to_str(can_id)} {name} dlc={dlc} data=[{data_hex(data)}]")
    if decoded:
        for key, value in decoded.items():
            print(f"    {group}.{key}: {value}")


def print_summary(state: Dict[str, Any], counts: Dict[int, int], first_seen: Dict[int, float], last_seen: Dict[int, float]) -> None:
    print("=" * 90)
    print(f"[{now_iso()}] Prius CAN Probe Summary")

    speed_kmh = state.get("speed_kmh")
    speed_mps = state.get("speed_mps")
    if speed_kmh is None:
        print("  Speed        : --")
    else:
        print(f"  Speed        : {speed_kmh:.2f} km/h  ({speed_mps:.3f} m/s)")

    steer_deg = state.get("steer_wheel_deg")
    steer_rad = state.get("steer_wheel_rad")
    if steer_deg is None:
        print("  Steering     : --")
    else:
        print(f"  Steering     : {steer_deg:.2f} deg  ({steer_rad:.4f} rad)  [steering wheel]")

    w01 = state.get("wheel_01_kmh")
    w23 = state.get("wheel_23_kmh")
    w45 = state.get("wheel_45_kmh")
    w67 = state.get("wheel_67_kmh")
    if w01 is None or w23 is None or w45 is None or w67 is None:
        print("  Wheel speeds : --")
    else:
        print(f"  Wheel speeds : W01={w01:.2f}, W23={w23:.2f}, W45={w45:.2f}, W67={w67:.2f} km/h")

    gas = state.get("gas_pedal_raw")
    engine = state.get("engine_status_text")
    if gas is None:
        print("  Gas / Engine : --")
    else:
        print(f"  Gas / Engine : gas_raw={gas}  engine={engine}")

    brake_pressure = state.get("brake_pressure_raw")
    brake_position = state.get("brake_position_raw")
    brake_pressed = state.get("brake_pressed")
    if brake_pressure is None and brake_position is None and brake_pressed is None:
        print("  Brake        : --")
    else:
        print(f"  Brake        : pressed={brake_pressed}  pressure_raw={brake_pressure}  position_raw={brake_position}")

    wire_enable = state.get("wire_enable_candidate")
    wire_hex = state.get("wire_status_byte7_hex")
    if wire_enable is None:
        print("  Wire status  : --")
    else:
        print(f"  Wire status  : enable_candidate={wire_enable}  byte7={wire_hex}")

    afl = state.get("abs_fl_active_candidate")
    afr = state.get("abs_fr_active_candidate")
    arl = state.get("abs_rl_active_candidate")
    arr = state.get("abs_rr_active_candidate")
    if afl is not None or afr is not None or arl is not None or arr is not None:
        print(f"  ABS          : FL={afl} FR={afr} RL={arl} RR={arr}")

    turn_text = state.get("turn_signal_text")
    if turn_text is not None:
        print(f"  Turn signal  : {turn_text}  code={state.get('turn_signal_code')}  blink={state.get('turn_blink_bit')}")

    print("  Observed ID rates:")
    for cid in sorted(counts):
        count = counts[cid]
        dt = max(last_seen.get(cid, 0.0) - first_seen.get(cid, 0.0), 1e-9)
        hz = count / dt if count > 1 else 0.0
        name = FRAME_NAMES.get(cid, "UNKNOWN")
        print(f"    {can_id_to_str(cid):<6} {name:<42} count={count:8d} approx_hz={hz:8.2f}")


# SocketCAN -------------------------------------------------------------------
def open_can_socket(interface: str) -> socket.socket:
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, CAN_RAW)
    sock.bind((interface,))
    sock.setblocking(False)
    return sock


def parse_can_frame(frame: bytes) -> Tuple[int, int, bytes, bool, bool, bool]:
    can_id_raw, dlc, data = struct.unpack(CAN_FRAME_FMT, frame)
    is_extended = (can_id_raw & CAN_EFF_FLAG) != 0
    is_rtr = (can_id_raw & CAN_RTR_FLAG) != 0
    is_error = (can_id_raw & CAN_ERR_FLAG) != 0
    if is_extended:
        can_id = can_id_raw & CAN_EFF_MASK
    else:
        can_id = can_id_raw & CAN_SFF_MASK
    dlc = min(int(dlc), 8)
    return can_id, dlc, data[:dlc], is_extended, is_rtr, is_error


# Logging ---------------------------------------------------------------------
RAW_FIELDS = [
    "time_iso",
    "monotonic",
    "interface",
    "can_id",
    "can_id_dec",
    "frame_name",
    "dlc",
    "data_hex",
    "is_extended",
    "is_rtr",
    "is_error",
]

DECODED_FIELDS = [
    "time_iso",
    "monotonic",
    "interface",
    "can_id",
    "can_id_dec",
    "frame_name",
    "group",
    "signal",
    "value",
    "data_hex",
]


def setup_loggers(base_log_dir: str, enable_logging: bool) -> Tuple[Optional[Any], Optional[Any], Optional[csv.DictWriter], Optional[csv.DictWriter], Optional[str]]:
    if not enable_logging:
        return None, None, None, None, None

    stamp = _dt.datetime.now().strftime("can_probe_%Y%m%d_%H%M%S")
    log_dir = os.path.join(base_log_dir, stamp)
    os.makedirs(log_dir, exist_ok=True)

    raw_path = os.path.join(log_dir, "raw_can.csv")
    decoded_path = os.path.join(log_dir, "decoded_can.csv")

    raw_file = open(raw_path, "w", newline="", encoding="utf-8")
    decoded_file = open(decoded_path, "w", newline="", encoding="utf-8")

    raw_writer = csv.DictWriter(raw_file, fieldnames=RAW_FIELDS)
    decoded_writer = csv.DictWriter(decoded_file, fieldnames=DECODED_FIELDS)
    raw_writer.writeheader()
    decoded_writer.writeheader()

    print(f"Logging raw CAN to     : {raw_path}")
    print(f"Logging decoded CAN to : {decoded_path}")
    return raw_file, decoded_file, raw_writer, decoded_writer, log_dir


# Main ------------------------------------------------------------------------
RUNNING = True


def _handle_signal(signum: int, frame: Any) -> None:
    global RUNNING
    RUNNING = False


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Prius CAN Probe - READ ONLY SocketCAN logger/decoder")
    parser.add_argument("--interface", "-i", default="can0", help="SocketCAN interface, e.g. can0, can1, vcan0")
    parser.add_argument("--duration", type=float, default=0.0, help="Run duration in seconds. 0 means run until Ctrl+C.")
    parser.add_argument("--summary-period", type=float, default=1.0, help="Summary print period in seconds.")
    parser.add_argument("--steer-format", choices=["i12_old", "i16_be", "i16_le"], default="i12_old", help="Steering decoder format")
    parser.add_argument("--print-each", action="store_true", help="Print every known decoded frame")
    parser.add_argument("--print-unknown", action="store_true", help="Print unknown frames too; can be very noisy")
    parser.add_argument("--log-dir", default="./can_logs", help="Base directory for log folders")
    parser.add_argument("--no-log", action="store_true", help="Disable CSV logging")
    parser.add_argument("--self-test", action="store_true", help="Run decoder self-test and exit")
    return parser


def run_self_test() -> int:
    print("Running decoder self-test...")

    tests = [
        ("speed", decode_speed_0x0b4(bytes.fromhex("00 00 00 00 00 04 F1 00"))["speed_kmh"], 12.65),
        ("steer_i12", decode_steer_0x025(bytes.fromhex("00 64 00 00 00 00 00 00"), "i12_old")["steer_wheel_deg"], 150.0),
        ("steer_i12_neg", decode_steer_0x025(bytes.fromhex("0F FA 00 00 00 00 00 00"), "i12_old")["steer_wheel_deg"], -9.0),
        ("wheel_zero", decode_wheel_speeds_0x0aa(bytes.fromhex("1A 6F 1A 6F 1A 6F 1A 6F"))["wheel_01_kmh"], 0.0),
        ("wheel_10", decode_wheel_speeds_0x0aa(bytes.fromhex("1E 57 1E 57 1E 57 1E 57"))["wheel_01_kmh"], 10.0),
        ("turn_left", decode_steering_levers_0x614(bytes.fromhex("00 80 00 10 00 00 00 00"))["turn_signal_code"], 1),
        ("turn_right", decode_steering_levers_0x614(bytes.fromhex("00 80 00 20 00 00 00 00"))["turn_signal_code"], 2),
        ("turn_none", decode_steering_levers_0x614(bytes.fromhex("00 00 00 30 00 00 00 00"))["turn_signal_code"], 3),
        ("brake", decode_brake_0x226(bytes.fromhex("00 A0 01 BC 04 00 00 00"))["brake_pressure_raw"], 416),
    ]

    ok = True
    for name, got, expected in tests:
        if isinstance(expected, float):
            passed = abs(float(got) - expected) < 1e-6
        else:
            passed = got == expected
        print(f"  {name:<16} got={got!r:<12} expected={expected!r:<12} {'OK' if passed else 'FAIL'}")
        ok = ok and passed

    return 0 if ok else 1


def main() -> int:
    global RUNNING

    args = build_arg_parser().parse_args()

    if args.self_test:
        return run_self_test()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    print("Prius CAN Probe - READ ONLY")
    print("This program will NOT send any CAN frames.")
    print(f"Opening SocketCAN interface: {args.interface}")

    sock = open_can_socket(args.interface)

    raw_file, decoded_file, raw_writer, decoded_writer, log_dir = setup_loggers(args.log_dir, not args.no_log)

    state = make_initial_state()
    counts: Dict[int, int] = {}
    first_seen: Dict[int, float] = {}
    last_seen: Dict[int, float] = {}

    start_t = time.monotonic()
    next_summary_t = start_t + max(args.summary_period, 0.1)

    try:
        while RUNNING:
            now_mono = time.monotonic()
            if args.duration and (now_mono - start_t) >= args.duration:
                break

            timeout = 0.05
            if args.duration:
                timeout = max(0.0, min(timeout, args.duration - (now_mono - start_t)))

            readable, _, _ = select.select([sock], [], [], timeout)

            if readable:
                try:
                    frame = sock.recv(CAN_FRAME_SIZE)
                except BlockingIOError:
                    frame = b""

                if frame:
                    ts = now_iso()
                    rx_t = time.monotonic()
                    can_id, dlc, payload, is_extended, is_rtr, is_error = parse_can_frame(frame)
                    name, group, decoded = decode_frame(can_id, payload, args.steer_format)

                    counts[can_id] = counts.get(can_id, 0) + 1
                    first_seen.setdefault(can_id, rx_t)
                    last_seen[can_id] = rx_t

                    hex_data = data_hex(payload)

                    if raw_writer is not None:
                        raw_writer.writerow({
                            "time_iso": ts,
                            "monotonic": f"{rx_t:.9f}",
                            "interface": args.interface,
                            "can_id": can_id_to_str(can_id),
                            "can_id_dec": can_id,
                            "frame_name": name,
                            "dlc": dlc,
                            "data_hex": hex_data,
                            "is_extended": is_extended,
                            "is_rtr": is_rtr,
                            "is_error": is_error,
                        })

                    if decoded_writer is not None and decoded:
                        for signal_name, value in decoded.items():
                            decoded_writer.writerow({
                                "time_iso": ts,
                                "monotonic": f"{rx_t:.9f}",
                                "interface": args.interface,
                                "can_id": can_id_to_str(can_id),
                                "can_id_dec": can_id,
                                "frame_name": name,
                                "group": group,
                                "signal": signal_name,
                                "value": value,
                                "data_hex": hex_data,
                            })

                    update_state(state, group, decoded)

                    if args.print_each and decoded:
                        print_decoded_frame(ts, can_id, name, dlc, payload, group, decoded)
                    elif args.print_unknown and not decoded:
                        print(f"[{ts}] {can_id_to_str(can_id)} {name} dlc={dlc} data=[{hex_data}]")

            now_mono = time.monotonic()
            if args.summary_period > 0 and now_mono >= next_summary_t:
                print_summary(state, counts, first_seen, last_seen)
                next_summary_t = now_mono + args.summary_period
                if raw_file is not None:
                    raw_file.flush()
                if decoded_file is not None:
                    decoded_file.flush()

    finally:
        print("\nStopping Prius CAN Probe...")
        print_summary(state, counts, first_seen, last_seen)
        try:
            sock.close()
        except Exception:
            pass
        if raw_file is not None:
            raw_file.flush()
            raw_file.close()
        if decoded_file is not None:
            decoded_file.flush()
            decoded_file.close()

    if log_dir:
        print(f"\nLog directory: {log_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
