#!/usr/bin/env python3
"""Send CAT commands or a saved audio-tuning preset to the ESP32-S3."""

import argparse
import glob
import os
import sys
import time

import serial


PRESETS = {
    # Selected by the joint phase-only sweep for maximum similarity to the
    # source voice while retaining the existing speech-DSP topology.
    "natural": ["*A150*", "*G15*", "*C400*", "*E0*", "*B2700*",
                "*L850*", "*R40*", "*e8*", "*j2*"],
    "default": ["*A150*", "*G15*", "*C400*", "*E0*", "*B2700*",
                "*L850*", "*R40*", "*e8*", "*j2*"],
    # Previous firmware defaults, retained for instant on-air rollback.
    "legacy": ["*A350*", "*G30*", "*C400*", "*E70*", "*B2700*",
               "*R10*", "*e20*", "*j20*"],
    "baseline": ["*A350*", "*G30*", "*C400*", "*E70*", "*B2700*",
                 "*R10*", "*e20*", "*j20*"],
    "optimized": ["*t0*", "*o1*", "*t1*"],
    "live": ["*t0*", "*o0*", "*t1*"],
}


def find_port(explicit):
    if explicit:
        return explicit
    candidates = []
    for pattern in ("/dev/tty.usbmodem*", "/dev/cu.usbmodem*", "/dev/ttyACM*"):
        candidates.extend(glob.glob(pattern))
    candidates = sorted(set(candidates))
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise SystemExit("No ESP32-S3 serial port found; pass --port PORT")
    raise SystemExit("Multiple serial ports found; pass --port PORT:\n  " +
                     "\n  ".join(candidates))


def normalize_commands(items):
    commands = []
    for item in items:
        if item in PRESETS:
            commands.extend(PRESETS[item])
        else:
            if not (item.startswith("*") and item.endswith("*")):
                raise SystemExit(f"CAT command must be wrapped in '*': {item}")
            commands.append(item)
    return commands


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("commands", nargs="+",
                        help="optimized/live, natural/default, legacy/baseline, or literal CAT commands")
    parser.add_argument("--port", default=os.environ.get("POLAR_PORT"))
    parser.add_argument("--baud", type=int, default=115200,
                        help="ignored by native USB, retained for compatibility")
    parser.add_argument("--response-wait", type=float, default=0.25)
    parser.add_argument("--command-gap", type=float, default=0.15,
                        help="delay between commands so TX stop/start sequences settle")
    args = parser.parse_args()

    port = find_port(args.port)
    commands = normalize_commands(args.commands)
    print(f"Sending {' '.join(commands)} to {port}")
    with serial.Serial(port, args.baud, timeout=0.1) as device:
        device.reset_input_buffer()
        for index, command in enumerate(commands):
            device.write(command.encode("ascii"))
            device.flush()
            if index + 1 < len(commands):
                time.sleep(args.command_gap)
        time.sleep(args.response_wait)
        response = device.read(device.in_waiting).decode("utf-8", errors="replace")
    if response:
        print(response, end="" if response.endswith("\n") else "\n")


if __name__ == "__main__":
    try:
        main()
    except serial.SerialException as error:
        print(f"Serial error: {error}", file=sys.stderr)
        raise SystemExit(1)
