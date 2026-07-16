# MTi-630 serial debug flow

If `xbus_sniffer` shows `frames_seen=0` but `bytes_dropped` increases, the serial port is receiving data but it is not valid Xbus MTData2 at the selected baudrate.

Run:

```bash
ros2 run prius_mti630_driver xbus_diag \
  --port /dev/serial/by-id/usb-Xsens_MTi_USB_Converter_DB6C4QWP-if00-port0 \
  --baudrate 115200 \
  --duration 3
```

Scan common baudrates:

```bash
ros2 run prius_mti630_driver xbus_diag \
  --port /dev/serial/by-id/usb-Xsens_MTi_USB_Converter_DB6C4QWP-if00-port0 \
  --scan-baud \
  --duration 2
```

Expected good result:

```text
frames_seen > 0
mtdata2 > 0
checksum_errors low or zero
```

If the ASCII preview contains `$...` NMEA sentences or readable text, configure the MTi in Xsens MT Manager / Movella DOT? Device Manager to output Xbus MTData2, not NMEA/ASCII.

Recommended output configuration:

- Protocol: Xbus / MTData2
- Rate: 100 Hz
- Baudrate: 115200 first; 921600 later if needed
- PacketCounter 0x1020
- SampleTimeFine 0x1060
- Quaternion 0x2010
- Acceleration 0x4020
- RateOfTurn 0x8020
- MagneticField 0xC020 optional
- StatusWord 0xE020
