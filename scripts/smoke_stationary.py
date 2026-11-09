#!/usr/bin/env python3
"""Operator-authorized stationary real-radar smoke check; no robot command topics.

Run only with an idle radar and stationary robot. Starts/stops its own canonical
launch and bag, writes measured JSON, and leaves existing processes untouched.
"""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import time
import urllib.request


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from ament_index_python.packages import get_package_prefix

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.seconds < 5: parser.error('Use at least five seconds of settled data')
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init(); node = Node('stationary_smoke_observer')
    processes = []; logs = []
    received = collections.defaultdict(list); schemas = {}; transforms = []; maps = []; health = []
    def launch(name, command):
        log = (args.output/(name+'.log')).open('w'); logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
        processes.append((name, process)); return process
    def cloud(kind, msg):
        received[kind].append(dict(monotonic=time.monotonic(),stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,width=msg.width))
        schemas[kind] = dict(frame=msg.header.frame_id,fields=[f.name for f in msg.fields],point_step=msg.point_step)
    subscriptions = [node.create_subscription(PointCloud2, topic, lambda m,k=kind:cloud(k,m),qos_profile_sensor_data)
        for kind,topic in [('source','/radar/points'),('raw','/radar_front/points_raw'),('processed','/radar_front/points')]]
    subscriptions += [
        node.create_subscription(OccupancyGrid,'/radar_front/occupancy',lambda m:maps.append((time.monotonic(),sum(v>=65 for v in m.data))),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(String,'/radar_front/status',lambda m:health.append((time.monotonic(),json.loads(m.data))),10),
        node.create_subscription(TFMessage,'/tf_static',transforms.append,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
    report = dict(passed=False,kind='real stationary radar smoke, unsurveyed room scene',robot_motion_commanded=False)
    try:
        for _ in range(15): rclpy.spin_once(node, timeout_sec=.1)
        if node.get_publishers_info_by_topic('/radar/points') or node.get_publishers_info_by_topic('/tf_static'):
            raise RuntimeError('Existing radar/TF publisher: coordinate instead of starting a duplicate')
        ports = list(Path('/dev/serial/by-id').glob('*XDS110*'))
        if not any('-if00' in p.name for p in ports) or not any('-if03' in p.name for p in ports):
            raise RuntimeError('XDS110 command/data interfaces required; never select OpenCR')
        for port in ports:
            owner = subprocess.run(['fuser',str(port.resolve())],capture_output=True)
            if owner.returncode != 1: raise RuntimeError('Radar serial port owned or ownership check failed')
        topics=['/radar/points','/radar_front/points_raw','/radar_front/points','/radar_front/metadata','/radar_front/status','/radar_front/occupancy','/tf_static','/tf']
        launch('bag',['ros2','bag','record','-o',str(args.output/'bag'),*topics])
        system = launch('system',['ros2','launch','radar_navigation','system.launch.py','viewer:=true'])
        deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if len(received['processed'])>=10 and transforms and maps: break
            if system.poll() is not None: raise RuntimeError('System exited before measurement')
        else: raise RuntimeError('No settled physical source/processed/map/TF input within 20 seconds')
        start=time.monotonic(); end=start+args.seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.03)
            if system.poll() is not None: raise RuntimeError('System stopped during measurement')
        report.update(seconds=time.monotonic()-start,schemas=schemas,driver_prefix=get_package_prefix('radar_navigation'),rates={},counts={})
        for kind,samples in received.items():
            rr=[r for r in samples if r['monotonic']>=start]
            report['counts'][kind]=len(rr)
            report['rates'][kind]=(len(rr)-1)/(rr[-1]['monotonic']-rr[0]['monotonic']) if len(rr)>1 else None
        mm=[r for r in maps if r[0]>=start]
        report.update(map_messages=len(mm),map_hz=(len(mm)-1)/(mm[-1][0]-mm[0][0]) if len(mm)>1 else None,
            occupied_cells_last=mm[-1][1] if mm else None,stale_messages=sum(t>=start and m['stale'] for t,m in health))
        pubs=node.get_publishers_info_by_topic('/tf_static')
        report['mount_publishers']=[dict(name=p.node_name,namespace=p.node_namespace) for p in pubs]
        mounts=[t for msg in transforms for t in msg.transforms if t.child_frame_id=='radar_link']
        report['mounts']=[dict(parent=t.header.frame_id,child=t.child_frame_id,translation=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],rotation=[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w]) for t in mounts]
        report['nodes']=node.get_node_names_and_namespaces()
        report['command_publishers']=[p.node_name for p in node.get_publishers_info_by_topic('/cmd_vel')]
        with urllib.request.urlopen('http://127.0.0.1:8765',timeout=3) as response:
            report['viewer_http_status']=response.status
        if len(pubs)!=1 or len(mounts)!=1 or report['mounts'][0]['translation']!=[.15,.06,.19]:
            raise RuntimeError('Expected exactly one default top-plate mount')
        if report['command_publishers']: raise RuntimeError('Unexpected command publisher in smoke environment')
        if not all(report['counts'].get(k,0)>0 and 8<report['rates'][k]<12 for k in ('source','raw','processed')):
            raise RuntimeError('Physical clouds missing or outside smoke rate bounds')
        if not report['map_messages'] or report['stale_messages']: raise RuntimeError('Missing maps or stale perception')
        report['passed']=True
    finally:
        for name, process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try: process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait(); report['passed']=False
            report[name+'_exit_code']=process.returncode
        for log in logs: log.close()
        node.destroy_node(); rclpy.shutdown()
        system_log=args.output/'system.log'
        if system_log.exists():
            output=system_log.read_text()
            report['clean_shutdown']=('radar stopped' in output and 'escalating to' not in output and 'Traceback' not in output)
            report['passed']=report['passed'] and report['clean_shutdown']
        (args.output/'metrics.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))
    if not report['passed']: raise SystemExit(1)


if __name__=='__main__':
    main()
