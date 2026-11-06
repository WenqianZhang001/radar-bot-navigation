"""Collect real ROS pipeline rates, receipt age, frame gaps and map statistics."""
import argparse
import json
import math
import statistics
import time
from pathlib import Path


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String
    parser=argparse.ArgumentParser()
    parser.add_argument('--seconds',type=float,default=30.)
    parser.add_argument('--namespace',default='/radar_front')
    parser.add_argument('--output',default='jetson_metrics.json')
    args=parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    rclpy.init()
    node=Node('radar_probe')
    counts={'raw':0,'processed':0,'maps':0,'frame_gaps':0,'frame_resets':0}
    ages=[]
    previous=None
    last_map=None
    def cloud(kind,msg):
        counts[kind]+=1
        if kind=='processed':
            ages.append((node.get_clock().now().nanoseconds-Time.from_msg(msg.header.stamp).nanoseconds)/1e6)
    def metadata(msg):
        nonlocal previous
        number=json.loads(msg.data)['number']
        if previous is not None:
            delta=(number-previous) % (2**32)
            if delta > 2**31:
                counts['frame_resets']+=1
            elif delta > 1:
                counts['frame_gaps']+=delta-1
        previous=number
    def occupancy(msg):
        nonlocal last_map
        counts['maps']+=1
        last_map=msg
    subs=[
        node.create_subscription(PointCloud2,args.namespace+'/points_raw',lambda m:cloud('raw',m),10),
        node.create_subscription(PointCloud2,args.namespace+'/points',lambda m:cloud('processed',m),10),
        node.create_subscription(String,args.namespace+'/metadata',metadata,10),
        node.create_subscription(OccupancyGrid,args.namespace+'/occupancy',occupancy,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
    started=time.monotonic()
    try:
        while time.monotonic()-started < args.seconds:
            rclpy.spin_once(node,timeout_sec=.05)
    finally:
        elapsed=time.monotonic()-started
        result=dict(counts,seconds=elapsed,raw_hz=counts['raw']/elapsed,
                    processed_hz=counts['processed']/elapsed,
                    receipt_to_probe_ms_p50=statistics.median(ages) if ages else None,
                    receipt_to_probe_ms_p95=sorted(ages)[max(0,math.ceil(.95*len(ages))-1)] if ages else None,
                    occupied_cells=sum(v>=65 for v in last_map.data) if last_map else None,
                    unknown_cells=sum(v<0 for v in last_map.data) if last_map else None,
                    note='Host serial receipt to probe, not radar acquisition latency.')
        Path(args.output).write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
        node.destroy_node()
        rclpy.shutdown()
    if not counts['raw'] or not counts['processed']:
        raise SystemExit('FAIL: raw and processed messages are both required')


if __name__=='__main__':
    main()
