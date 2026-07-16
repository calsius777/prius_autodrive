#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time

from .xbus import MID_MTDATA2, XbusParser, parse_mtdata2_payload


def main() -> None:
    parser = argparse.ArgumentParser(description='Print decoded Xsens Xbus MTData2 packets from a serial port.')
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--hex', action='store_true', help='Print raw payload hex for each MTData2 frame.')
    args = parser.parse_args()

    import serial

    xp = XbusParser()
    start = time.time()
    count = 0
    with serial.Serial(args.port, args.baudrate, timeout=0.05) as ser:
        print(f'Reading {args.port} @ {args.baudrate} for {args.duration:.1f}s')
        while time.time() - start < args.duration:
            frames = xp.feed(ser.read(512))
            for frame in frames:
                if frame.mid != MID_MTDATA2:
                    continue
                count += 1
                pkt = parse_mtdata2_payload(frame.payload)
                parts = [f'#{count}']
                if pkt.packet_counter is not None:
                    parts.append(f'pc={pkt.packet_counter}')
                if pkt.sample_time_fine is not None:
                    parts.append(f'stf={pkt.sample_time_fine}')
                if pkt.quaternion_wxyz is not None:
                    parts.append(f'q(wxyz)={tuple(round(v, 5) for v in pkt.quaternion_wxyz)}')
                if pkt.rate_of_turn_xyz is not None:
                    parts.append(f'gyr={tuple(round(v, 5) for v in pkt.rate_of_turn_xyz)} rad/s')
                if pkt.acceleration_xyz is not None:
                    parts.append(f'acc={tuple(round(v, 5) for v in pkt.acceleration_xyz)} m/s^2')
                if pkt.magnetic_field_xyz is not None:
                    parts.append(f'mag={tuple(round(v, 8) for v in pkt.magnetic_field_xyz)}')
                if pkt.status_word is not None:
                    parts.append(f'status=0x{pkt.status_word:08X}')
                if args.hex:
                    parts.append(frame.payload.hex(' '))
                print(' | '.join(parts))

    print(f'frames_seen={xp.frames_seen} checksum_errors={xp.checksum_errors} bytes_dropped={xp.bytes_dropped}')


if __name__ == '__main__':
    main()
