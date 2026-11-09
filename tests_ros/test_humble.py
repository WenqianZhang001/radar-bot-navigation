"""Real Humble middleware + pseudo-terminal sensor integration; never opens robot devices."""
import json
import os
import pty
import select
import signal
import struct
import subprocess
import tempfile
import threading
import time
import unittest

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import Bool
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from rclpy.time import Time
from radar_navigation.protocol import MAGIC


class HumbleIntegration(unittest.TestCase):
    def test_serial_to_mapping_and_watchdog(self):
        rclpy.init()
        node=Node('integration_observer')
        base_tf=StaticTransformBroadcaster(node)
        joint=TransformStamped()
        joint.header.frame_id='base_footprint'
        joint.child_frame_id='base_link'
        joint.transform.translation.z=.01
        joint.transform.rotation.w=1.
        base_tf.sendTransform(joint)
        odom_tf=TransformBroadcaster(node)
        def delayed_odometry():
            msg=TransformStamped()
            msg.header.frame_id='odom'
            msg.child_frame_id='base_footprint'
            msg.header.stamp=Time(nanoseconds=node.get_clock().now().nanoseconds-50000000).to_msg()
            msg.transform.rotation.w=1.
            odom_tf.sendTransform(msg)
        timer=node.create_timer(.02,delayed_odometry)
        clouds,maps,commands=[],[],[]
        subs=[
            node.create_subscription(PointCloud2,'/radar_front/points',clouds.append,10),
            node.create_subscription(OccupancyGrid,'/radar_front/occupancy',maps.append,
                                     QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            node.create_subscription(Twist,'/radar/cmd_vel_safe',commands.append,10)]
        arm=node.create_publisher(Bool,'/radar/enable',10)
        request=node.create_publisher(Twist,'/radar/cmd_vel_request',10)
        cli_master,cli_slave=pty.openpty()
        data_master,data_slave=pty.openpty()
        running=threading.Event()
        quit_event=threading.Event()
        transmitting=threading.Event()
        transmitting.set()
        stop_seen=threading.Event()
        distance=[2.]
        def emulator():
            line=b''
            frame=0
            last=0.
            while not quit_event.is_set():
                ready,_,_=select.select([cli_master],[],[],.01)
                if ready:
                    chunk=os.read(cli_master,4096)
                    for char in chunk:
                        if char==10:
                            cmd=line.strip()
                            if cmd==b'sensorStart':
                                running.set()
                            if cmd==b'sensorStop':
                                stop_seen.set()
                                running.clear()
                            os.write(cli_master,b'Done\nmmwDemo:/>')
                            line=b''
                        else:
                            line+=bytes([char])
                if running.is_set() and transmitting.is_set() and time.monotonic()-last>.1:
                    packet=MAGIC+struct.pack('<8I',1,64,0,frame,123,1,1,0)
                    packet+=struct.pack('<2I4f',1,16,0.,distance[0],.3,.1)
                    os.write(data_master,packet)
                    frame+=1
                    last=time.monotonic()
        worker=threading.Thread(target=emulator,daemon=True)
        worker.start()
        processes=[]
        logs=[]
        try:
            with tempfile.TemporaryDirectory() as directory:
                cfg=os.path.join(directory,'radar.cfg')
                with open(cfg,'w') as stream:
                    stream.write('sensorStart\n')
                mount_path=os.path.join(directory,'mount.yaml')
                with open(mount_path,'w') as stream:
                    stream.write('measured: true\nx: 0.0\ny: 0.0\nz: 0.0\nroll_deg: 0.0\npitch_deg: 0.0\nyaw_deg: 0.0\n')
                for command in (
                    ['ros2','launch','radar_navigation','system.launch.py',
                     'cli_port:='+os.ttyname(cli_slave),'data_port:='+os.ttyname(data_slave),
                     'cfg_file:='+cfg,'mount_file:='+mount_path,'fixed_frame:=odom'],
                    ['ros2','run','radar_navigation','radar_control']):
                    log=tempfile.TemporaryFile(mode='w+')
                    logs.append(log)
                    processes.append(subprocess.Popen(command,stdout=log,stderr=log,
                                                       start_new_session=True))
                def spin_until(predicate,timeout=10.,enable=False):
                    end=time.monotonic()+timeout
                    while time.monotonic()<end:
                        if enable:
                            arm.publish(Bool(data=True))
                            cmd=Twist()
                            cmd.linear.x=.08
                            request.publish(cmd)
                        rclpy.spin_once(node,timeout_sec=.03)
                        if predicate():
                            return
                    self.fail('ROS condition timed out')
                spin_until(lambda: len(clouds)>=10 and bool(maps))
                self.assertEqual([f.name for f in clouds[-1].fields],['x','y','z','doppler'])
                point=list(read_points(clouds[-1],field_names=('x','y','z','doppler')))[0]
                self.assertAlmostEqual(float(point[0]),2.)
                self.assertAlmostEqual(float(point[3]),.1,places=5)
                self.assertEqual(clouds[-1].header.frame_id,'odom')
                spin_until(lambda: any(v>=65 for v in maps[-1].data))
                report=os.path.join(directory,'metrics.json')
                probe_log=tempfile.TemporaryFile(mode='w+')
                logs.append(probe_log)
                probe=subprocess.Popen(['ros2','run','radar_navigation','radar_probe',
                    '--seconds','2','--output',report],stdout=probe_log,stderr=probe_log,
                    start_new_session=True)
                processes.append(probe)
                spin_until(lambda: probe.poll() is not None)
                self.assertEqual(probe.returncode,0)
                with open(report) as stream:
                    metrics=json.load(stream)
                self.assertGreater(metrics['processed'],0)
                self.assertGreater(metrics['raw'],0)
                spin_until(lambda: bool(commands))
                self.assertTrue(all(c.linear.x==0 and c.angular.z==0 for c in commands))
                commands.clear()
                spin_until(lambda: any(c.linear.x>0 for c in commands),enable=True)
                distance[0]=.4
                commands.clear()
                spin_until(lambda: bool(commands) and commands[-1].linear.x==0,
                           timeout=1.,enable=True)
                distance[0]=2.
                commands.clear()
                spin_until(lambda: any(c.linear.x>0 for c in commands),enable=True)
                # Releasing the operator lease must stop even with healthy perception.
                commands.clear()
                spin_until(lambda: bool(commands) and commands[-1].linear.x==0,timeout=1.)
                commands.clear()
                spin_until(lambda: any(c.linear.x>0 for c in commands),enable=True)
                # Keep command and enable alive while disconnecting only perception.
                transmitting.clear()
                start=time.monotonic()
                commands.clear()
                spin_until(lambda: bool(commands) and commands[-1].linear.x==0,
                           timeout=1.,enable=True)
                stop_latency=time.monotonic()-start
                self.assertLess(stop_latency,.6)
                spin_until(lambda: all(v==-1 for v in maps[-1].data),timeout=3.,enable=True)
                print(json.dumps({'test':'humble_serial_pipeline',
                    'clouds':len(clouds),'maps':len(maps),'stale_stop_seconds':stop_latency}))
        finally:
            for process in processes:
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGINT)
            for process in processes:
                try:
                    process.wait(timeout=8.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    process.wait()
            quit_event.set()
            worker.join(timeout=1.)
            outputs=[]
            for log in logs:
                log.seek(0)
                output=log.read()
                print(output[-5000:])
                outputs.append(output)
                log.close()
            for fd in (cli_master,cli_slave,data_master,data_slave):
                os.close(fd)
            node.destroy_node()
            rclpy.shutdown()
            self.assertTrue(stop_seen.is_set(), "sensorStop must reach the CLI on shutdown")
            self.assertNotIn("Traceback",''.join(outputs))


if __name__=='__main__':
    unittest.main()
