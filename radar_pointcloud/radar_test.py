#!/usr/bin/env python3
"""
AWR1843BOOST quick test: send a config, read point clouds, print them,
and optionally publish them to ROS 2 as sensor_msgs/PointCloud2.

Usage:
  python3 radar_test.py --cfg awr1843_profile.cfg          # print points only
  python3 radar_test.py --cfg awr1843_profile.cfg --ros    # also publish /radar/points
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
    buf = bytearray()
    while True:
        buf += data.read(data.in_waiting or 1)
        idx = buf.find(MAGIC)
        if idx < 0:
            del buf[:-8]
            continue
        if len(buf) - idx < HEADER_LEN:
            continue
        (_ver, total_len, _plat, frame_num, _t, _nobj, num_tlv, _sub) = \
            struct.unpack_from('<8I', buf, idx + 8)
        if total_len < HEADER_LEN or total_len > 100000:
            del buf[:idx + 8]          # corrupt header, resync
            continue
        if len(buf) - idx < total_len:
            continue
        frame = bytes(buf[idx:idx + total_len])
        del buf[:idx + total_len]

        points, off = [], HEADER_LEN
        for _ in range(num_tlv):
            if off + 8 > len(frame):
                break
            tlv_type, tlv_len = struct.unpack_from('<2I', frame, off)
            off += 8
            if tlv_type == TLV_DETECTED_POINTS:
                for k in range(tlv_len // 16):
                    points.append(struct.unpack_from('<4f', frame, off + 16 * k))
            off += tlv_len
        yield frame_num, points


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cfg', required=True, help='radar config file')
    ap.add_argument('--ros', action='store_true', help='publish /radar/points to ROS 2')
    args = ap.parse_args()

    cli_port, data_port = find_ports()
    print(f"CLI port : {cli_port}\nData port: {data_port}\n")
    cli = serial.Serial(cli_port, 115200, timeout=0.1)
    data = serial.Serial(data_port, 921600, timeout=0.1)

    if not send_cfg(cli, args.cfg):
        print("\nSome config lines failed. Paste the [ERR] lines to troubleshoot.")
    print("\nStreaming... press Ctrl+C to stop.\n")

    pub = node = None
    if args.ros:
        import rclpy
        from sensor_msgs.msg import PointCloud2  # noqa: F401
        from sensor_msgs_py import point_cloud2
        from std_msgs.msg import Header
        rclpy.init()
        node = rclpy.create_node('awr1843_test')
        pub = node.create_publisher(PointCloud2, '/radar/points', 10)

    try:
        for frame_num, pts in read_frames(data):
            print(f"Frame {frame_num}: {len(pts)} points")
            for x, y, z, v in pts[:5]:
                r = (x * x + y * y + z * z) ** 0.5
                print(f"    range {r:5.2f} m   x {x:+.2f}  y {y:+.2f}  z {z:+.2f}  vel {v:+.2f} m/s")
            if pub:
                header = Header()
                header.stamp = node.get_clock().now().to_msg()
                header.frame_id = 'radar_link'
                # TI: y forward, x right, z up  ->  ROS: x forward, y left, z up
                ros_pts = [(y, -x, z) for x, y, z, _ in pts]
                pub.publish(point_cloud2.create_cloud_xyz32(header, ros_pts))
    except KeyboardInterrupt:
        pass
    finally:
        send_command(cli, 'sensorStop')
        cli.close()
        data.close()
        if node:
            node.destroy_node()
            import rclpy
            rclpy.shutdown()
        print("\nRadar stopped.")


if __name__ == '__main__':
    main()
