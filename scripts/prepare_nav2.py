#!/usr/bin/env python3
"""Merge radar local perception into a validated Humble Nav2 configuration."""
import argparse
from pathlib import Path
import yaml

parser=argparse.ArgumentParser()
parser.add_argument('base')
parser.add_argument('output')
args=parser.parse_args()
base=yaml.safe_load(Path(args.base).read_text())
overlay=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/nav2_radar.yaml').read_text())
def merge(dst,src):
    for key,value in src.items():
        if isinstance(value,dict) and isinstance(dst.get(key),dict):
            merge(dst[key],value)
        else:
            dst[key]=value
merge(base,overlay)
# Global route follows the surveyed prior map. Live LiDAR is for AMCL only.
global_params=base['global_costmap']['global_costmap']['ros__parameters']
global_params['plugins']=['static_layer','inflation_layer']
global_params['static_layer']={'plugin':'nav2_costmap_2d::StaticLayer',
                               'map_subscribe_transient_local':True}
with open(args.output,'x') as stream:
    yaml.safe_dump(base,stream,sort_keys=False)
