"""Lossless JSONL frame recording, timed replay and serial acquisition."""
import json
import time
from contextlib import contextmanager
from .protocol import Frame, Parser


def write_frame(stream, frame):
    stream.write(json.dumps(frame.to_dict(), allow_nan=False) + "\n")
    stream.flush()


def replay(path, speed=1.):
    if speed < 0:
        raise ValueError("speed must be nonnegative; zero means unpaced")
    first = None
    start = time.monotonic()
    with open(path) as source:
        for line in source:
            frame = Frame(**json.loads(line))
            if first is None:
                first = frame.stamp_ns
            if speed:
                time.sleep(max(0., (frame.stamp_ns-first)/1e9/speed-(time.monotonic()-start)))
            yield frame


@contextmanager
def acquire(cli_path, data_path, cfg, sensor, stop=None):
    import serial
    from radar_test import send_cfg, send_command
    with serial.Serial(cli_path, 115200, timeout=.1, exclusive=True) as cli:
        try:
            with serial.Serial(data_path, 921600, timeout=.1, exclusive=True) as data:
                if not send_cfg(cli, cfg):
                    raise RuntimeError("Radar configuration rejected")
                parser = Parser(sensor)
                def frames():
                    last = time.monotonic()
                    while stop is None or not stop.is_set():
                        chunk = data.read(data.in_waiting or 1)
                        if chunk:
                            last = time.monotonic()
                        elif time.monotonic()-last > 3:
                            raise TimeoutError("No radar bytes for 3 seconds")
                        yield from parser.feed(chunk)
                yield frames()
        finally:
            send_command(cli, 'sensorStop')
