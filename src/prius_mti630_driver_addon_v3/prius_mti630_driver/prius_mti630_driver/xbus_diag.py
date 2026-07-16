#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
import string
import time
from collections import Counter

from .xbus import MID_MTDATA2, XbusParser, parse_mtdata2_payload

COMMON_BAUDS = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000, 2000000]


def ascii_preview(data: bytes, limit: int = 512) -> str:
    shown = data[:limit]
    out = []
    printable = set(bytes(string.printable, 'ascii'))
    for b in shown:
        if b in (10, 13, 9):
            out.append(chr(b))
        elif b in printable and b not in (11, 12):
            out.append(chr(b))
        else:
            out.append('.')
    return ''.join(out)


def hex_preview(data: bytes, limit: int = 192) -> str:
    shown = data[:limit]
    lines = []
    for i in range(0, len(shown), 16):
        chunk = shown[i:i+16]
        hx = ' '.join(f'{b:02X}' for b in chunk)
        asc = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in chunk)
        lines.append(f'{i:04X}: {hx:<47}  {asc}')
    return '\n'.join(lines)


def classify(data: bytes) -> list[str]:
    notes = []
    if not data:
        return ['no bytes read']
    printable = sum(1 for b in data if b in (9, 10, 13) or 32 <= b <= 126)
    ratio = printable / max(1, len(data))
    if ratio > 0.80:
        notes.append(f'looks like ASCII/text stream, printable_ratio={ratio:.2f}')
    else:
        notes.append(f'looks binary/noisy, printable_ratio={ratio:.2f}')
    if b'$' in data or re.search(rb'\$[A-Z]{2,}', data):
        notes.append('NMEA-like "$" sentences detected; device may be in ASCII/NMEA output mode, not Xbus MTData2')
    if b'\xFA' in data:
        notes.append(f'0xFA preamble-like bytes seen: {data.count(bytes([0xFA]))}')
    else:
        notes.append('no 0xFA Xbus preamble byte seen in sample')
    top = Counter(data).most_common(8)
    notes.append('top bytes: ' + ', '.join(f'0x{k:02X}({v})' for k, v in top))
    return notes


def read_once(port: str, baud: int, seconds: float, chunk_size: int) -> tuple[bytes, XbusParser, int, int]:
    import serial
    xp = XbusParser()
    raw = bytearray()
    mtdata2 = 0
    other = 0
    end = time.time() + seconds
    with serial.Serial(port, baud, timeout=0.05) as ser:
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        while time.time() < end:
            data = ser.read(chunk_size)
            if data:
                raw.extend(data)
                for frame in xp.feed(data):
                    if frame.mid == MID_MTDATA2:
                        mtdata2 += 1
                    else:
                        other += 1
    return bytes(raw), xp, mtdata2, other


def main() -> None:
    ap = argparse.ArgumentParser(description='Diagnose Xsens MTi serial stream and Xbus MTData2 framing.')
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--baudrate', type=int, default=115200, help='Baudrate for single diagnostic run.')
    ap.add_argument('--duration', type=float, default=3.0)
    ap.add_argument('--chunk-size', type=int, default=512)
    ap.add_argument('--scan-baud', action='store_true', help='Try common baudrates and report Xbus frame counts.')
    args = ap.parse_args()

    bauds = COMMON_BAUDS if args.scan_baud else [args.baudrate]
    best = None
    for baud in bauds:
        try:
            raw, xp, mtdata2, other = read_once(args.port, baud, args.duration, args.chunk_size)
        except Exception as e:
            print(f'baud={baud}: ERROR: {e}')
            continue
        score = (mtdata2, xp.frames_seen, -xp.checksum_errors, len(raw))
        if best is None or score > best[0]:
            best = (score, baud, raw, xp, mtdata2, other)
        print(f'baud={baud}: bytes={len(raw)} frames_seen={xp.frames_seen} mtdata2={mtdata2} other_frames={other} checksum_errors={xp.checksum_errors} bytes_dropped={xp.bytes_dropped}')
        if args.scan_baud:
            print('  ' + ' | '.join(classify(raw)[:3]))

    if best is None:
        return
    _, baud, raw, xp, mtdata2, other = best
    print('\n=== best/selected raw preview ===')
    print(f'baud={baud}, bytes={len(raw)}, frames_seen={xp.frames_seen}, mtdata2={mtdata2}, checksum_errors={xp.checksum_errors}, bytes_dropped={xp.bytes_dropped}')
    print('\n'.join(classify(raw)))
    print('\n--- HEX preview ---')
    print(hex_preview(raw))
    print('\n--- ASCII preview ---')
    print(ascii_preview(raw))

    # If frames are present, print a few decoded MTData2 items.
    if xp.frames_seen:
        xp2 = XbusParser()
        printed = 0
        for frame in xp2.feed(raw):
            if frame.mid != MID_MTDATA2:
                continue
            pkt = parse_mtdata2_payload(frame.payload, frame.rx_time)
            print(f'packet_counter={pkt.packet_counter} sample_time_fine={pkt.sample_time_fine} q={pkt.quaternion_wxyz} gyr={pkt.rate_of_turn_xyz} acc={pkt.acceleration_xyz} status={pkt.status_word}')
            printed += 1
            if printed >= 5:
                break

if __name__ == '__main__':
    main()
