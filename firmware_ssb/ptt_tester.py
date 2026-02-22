#!/usr/bin/env python3
"""
Simple PTT control for DDX over USB serial.

Usage:
  python3 ptt_control.py on
  python3 ptt_control.py off
  python3 ptt_control.py on  /dev/ttyACM0
"""

import sys
import time

import serial
import serial.tools.list_ports


BAUD_RATE = 115200


def find_ddx_port():
    """Auto-detect the DDX radio serial port (same logic as transceiver_server.py)."""
    for p in serial.tools.list_ports.comports():
        hwid = p.hwid.lower()
        if "cafe" in hwid or "4011" in hwid:
            return p.device
    return None


def send_cmd(port, cmd_bytes):
    with serial.Serial(port, BAUD_RATE, timeout=1) as ser:
        # Small delay after opening to let the device settle
        time.sleep(0.2)
        ser.write(cmd_bytes)
        ser.flush()


def main():
    if len(sys.argv) < 2 or sys.argv[1].lower() not in ("on", "off"):
        print("Usage: ptt_control.py on|off [serial_port]")
        sys.exit(1)

    action = sys.argv[1].lower()
    if len(sys.argv) >= 3:
        port = sys.argv[2]
    else:
        port = find_ddx_port()
        if not port:
            print("Could not auto-detect DDX radio port. Please specify it explicitly.")
            sys.exit(1)
        print(f"Using detected port: {port}")

    if action == "on":
        cmd = b"*p*"
        print("Sending PTT ON...")
    else:
        cmd = b"*z*"
        print("Sending PTT OFF...")

    try:
        send_cmd(port, cmd)
        print("Done.")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
