"""Seeded stationary synthetic perception benchmark; not physical navigation evidence."""
import argparse
import csv
import json
import math
import random
import time
from pathlib import Path
from .core import Filter, Grid, Pose
from .protocol import Frame


def run(seed, sensors=1, accumulation=True, doppler=True, lidar=False):
    rng = random.Random(seed)
    truth = [(3., y/10) for y in range(-20, 21)]
    filt = Filter(window=.5 if accumulation else 0., doppler=doppler)
    grid = Grid()
    predicted = []
    latencies = []
    for k in range(100):
        for sensor in range(sensors):
            points = []
            for x, y in truth:
                if rng.random() < (1. if lidar else .25):
                    noise = .01 if lidar else .05
                    # Sensor coordinates: right=-world y; forward=world x.
                    points.append([-y+rng.gauss(0, noise), x+rng.gauss(0, noise), .3, 0.])
            if not lidar:
                points.append([rng.uniform(-3, 3), rng.uniform(1, 6), .3, .4])
            frame = Frame(str(sensor), k*100000000+sensor, k, points)
            before = time.perf_counter()
            output = filt.update(frame, Pose())
            grid.update(output)
            latencies.append((time.perf_counter()-before)*1000)
            predicted = output
    distances = [min(math.hypot(p[0]-x, p[1]-y) for x,y in truth) for p in predicted]
    precision = sum(d < .2 for d in distances)/max(1,len(distances))
    recall = sum(any(math.hypot(p[0]-x,p[1]-y)<.2 for p in predicted) for x,y in truth)/len(truth)
    return dict(seed=seed, precision=precision, recall=recall,
                endpoint_error_m=sum(distances)/max(1,len(distances)),
                processing_ms=sum(latencies)/len(latencies),
                throughput_frames_s=1000/(sum(latencies)/len(latencies)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', default='results/synthetic')
    ap.add_argument('--seeds', type=int, default=5)
    args = ap.parse_args()
    target = Path(args.output)
    target.mkdir(parents=True, exist_ok=True)
    modes = dict(lidar=dict(lidar=True), single={}, dual=dict(sensors=2),
                 no_accumulation=dict(accumulation=False), no_doppler=dict(doppler=False))
    rows = [dict(mode=name, **run(seed, **options)) for name, options in modes.items()
            for seed in range(args.seeds)]
    with (target/'metrics.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary = {name: {key: sum(row[key] for row in rows if row['mode']==name)/args.seeds
                     for key in ('precision','recall','endpoint_error_m','processing_ms')}
               for name in modes}
    (target/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    # Standalone SVG plot without a plotting dependency.
    bars = []
    for i, (name, metrics) in enumerate(summary.items()):
        y = 30+i*45
        bars.append(f'<text x="5" y="{y+15}">{name}</text><rect x="160" y="{y}" width="{metrics["recall"]*300}" height="25" fill="steelblue"/>')
    (target/'recall.svg').write_text('<svg xmlns="http://www.w3.org/2000/svg" width="500" height="280"><rect width="100%" height="100%" fill="white"/>'+''.join(bars)+'<text x="160" y="270">Synthetic endpoint recall (0–1)</text></svg>')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
