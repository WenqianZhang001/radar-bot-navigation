"""Closed-loop grid navigation experiment with noisy first-return sensing.

This deliberately simple simulator measures its own collision geometry; it is
not a replacement for Gazebo, Nav2 integration, or physical trials.
"""
import argparse
import json
import math
import random
from pathlib import Path
from .core import Grid
from .planning import astar


def trial(seed=0, beams=61):
    rng=random.Random(seed)
    grid=Grid(60,40,.1,(0.,0.),clear=True)
    obstacles={(30,y) for y in range(8,30)}
    obstacles |= {(x,y) for x in range(60) for y in (0,39)}
    obstacles |= {(x,y) for y in range(40) for x in (0,59)}
    position, goal=(8,20),(50,20)
    path=[position]
    collisions=0
    for step in range(200):
        origin=((position[0]+.5)*.1,(position[1]+.5)*.1)
        points=[]
        # Ideal 360-degree coverage is explicitly assumed in this benchmark.
        for i in range(beams):
            angle=2*math.pi*i/beams
            for j in range(1,81):
                x,y=origin[0]+j*.05*math.cos(angle),origin[1]+j*.05*math.sin(angle)
                cell=grid.cell(x,y)
                if cell in obstacles:
                    if rng.random() > .1:
                        points.append((x,y,0,0))
                    break
        grid.update(points,origin)
        route=astar(grid,position,goal,radius=.22,allow_unknown=True)
        if not route:
            break
        if len(route)==1:
            break
        proposed=route[1]
        # Independent ground truth disk collision check, never hidden from metrics.
        if any(math.hypot(proposed[0]-x,proposed[1]-y)*.1 <= .22 for x,y in obstacles):
            collisions+=1
            break
        position=proposed
        path.append(position)
        if position==goal:
            break
    length=(len(path)-1)*.1
    # Ground truth reference planner with the identical robot radius.
    oracle=Grid(60,40,.1,(0.,0.))
    oracle.seen=[True]*2400
    oracle.odds=[-4.]*2400
    for x,y in obstacles:
        oracle.odds[y*60+x]=4.
    optimal=astar(oracle,path[0],goal,radius=.22)
    return dict(seed=seed,beams=beams,success=position==goal,collisions=collisions,
                interventions=0,steps=len(path)-1,completion_time_s=(len(path)-1)*.5,
                path_length_m=length,
                path_efficiency=((len(optimal)-1)*.1/length if position==goal and length else None),
                path=path)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--output',default='results/navigation.json')
    args=ap.parse_args()
    rows=[trial(seed,beams) for beams in (31,61,181) for seed in range(5)]
    Path(args.output).parent.mkdir(parents=True,exist_ok=True)
    Path(args.output).write_text(json.dumps(rows,indent=2)+'\n')
    print(json.dumps({str(beams):dict(successes=sum(r['success'] for r in rows if r['beams']==beams),
                                    collisions=sum(r['collisions'] for r in rows if r['beams']==beams))
                      for beams in (31,61,181)},indent=2))


if __name__=='__main__':
    main()
