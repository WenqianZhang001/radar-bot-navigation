import argparse
import json
from .io import acquire, replay, write_frame


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    record = sub.add_parser("record")
    for name in ("cli", "data", "cfg", "output"):
        record.add_argument("--"+name, required=True)
    record.add_argument("--sensor", default="radar_front")
    play = sub.add_parser("replay")
    play.add_argument("path")
    play.add_argument("--speed", type=float, default=1.)
    args = parser.parse_args()
    try:
        if args.command == "record":
            with open(args.output, "x") as out:
                with acquire(args.cli, args.data, args.cfg, args.sensor) as frames:
                    for frame in frames:
                        write_frame(out, frame)
        else:
            for frame in replay(args.path, args.speed):
                print(json.dumps(frame.to_dict()))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
