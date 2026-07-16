#!/usr/bin/env bash
set -euo pipefail

# Creates a stable /dev/mti630 symlink for common USB-serial adapters.
# Run on the HOST, not only inside Docker:
#   sudo bash setup_mti630_udev.sh

RULE=/etc/udev/rules.d/99-prius-mti630.rules

cat <<'RULE_EOF' | sudo tee "$RULE" >/dev/null
# Prius project MTi-630 USB/serial aliases.
# FTDI adapters are common on Xsens development/starter kits.
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", MODE="0666", SYMLINK+="mti630"
# Silicon Labs CP210x fallback.
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", MODE="0666", SYMLINK+="mti630"
# QinHeng CH340 fallback.
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", MODE="0666", SYMLINK+="mti630"
RULE_EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Installed $RULE"
echo "Reconnect the MTi-630 USB cable, then check: ls -l /dev/mti630 /dev/serial/by-id/"
