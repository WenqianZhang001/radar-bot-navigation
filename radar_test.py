#!/usr/bin/env python3
"""Compatibility helpers for the legacy viewer; no standalone ROS driver.

Hardware operation uses radar_navigation.driver through system.launch.py.
"""
import argparse
import glob
import struct
import sys
import time

import serial

MAGIC = b'\x02\x01\x04\x03\x06\x05\x08\x07'
HEADER_LEN = 40          # 8-byte magic + 8 x uint32
TLV_DETECTED_POINTS = 1  # x, y, z, velocity (float32 each)
PROMPT = ':/>'           # end of the CLI prompt "mmwDemo:/>"; printed when ready for input


def find_ports():
    cli = glob.glob('/dev/serial/by-id/*XDS110*if00*')
    data = glob.glob('/dev/serial/by-id/*XDS110*if03*')
    if not cli or not data:
        sys.exit("Radar ports not found. Check the USB cable, then run: ls -l /dev/serial/by-id/")
    if len(cli) != 1 or len(data) != 1:
        sys.exit("Multiple radars found. Use radar record or ROS launch with explicit CLI/data paths.")
    return cli[0], data[0]


def send_command(cli, cmd, timeout=3.0):
    """Type one CLI command and return the radar's full reply (up to the next prompt)."""
    # The radar's CLI drops characters when a whole line arrives at once
    # (e.g. "flushCfg" arrived as "lushCfg"), so type it one character at a time.
    for ch in (cmd + '\n').encode():
        cli.write(bytes([ch]))
        time.sleep(0.001)
    resp, deadline = b'', time.time() + timeout
    while PROMPT.encode() not in resp and time.time() < deadline:
        resp += cli.read(cli.in_waiting or 1)
    return resp.decode(errors='ignore')


def send_cfg(cli, path):
    """Send each config line and print the radar's reply."""
    with open(path) as f:
        lines = [l.strip() for l in f if l.strip() and not l.startswith('%')]
    # An empty line flushes anything half-typed in the CLI and checks that it answers.
    cli.reset_input_buffer()
    if PROMPT not in send_command(cli, '', timeout=1.0):
        sys.exit("Radar CLI is not answering. Press the reset button on the radar board and try again.")
    ok = True
    for line in lines:
        text = send_command(cli, line).replace('mmwDemo:/>', '').strip()
        status = 'OK ' if 'Done' in text else 'ERR'
        if status == 'ERR' and line not in ('sensorStop', 'flushCfg'):
            ok = False
        print(f"[{status}] {line}" + ('' if status == 'OK ' else f"   -> {text}"))
    return ok


def read_frames(data):
    """Yield (frame_number, [(x, y, z, v), ...]) for each complete radar frame."""
    from radar_navigation.protocol import Parser
    parser = Parser()
    while True:
        for frame in parser.feed(data.read(data.in_waiting or 1)):
            yield frame.number, [tuple(p) for p in frame.points]


def main():
    raise SystemExit('Standalone hardware/ROS test retired. Use: ros2 launch radar_navigation system.launch.py')


if __name__ == '__main__':
    main()
