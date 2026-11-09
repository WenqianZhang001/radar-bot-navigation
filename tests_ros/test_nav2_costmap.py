"""Validate the shipped aging-map overlay against the real Humble StaticLayer."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_prefix, PackageNotFoundError
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import ChangeState
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import yaml


class Nav2CostmapIntegration(unittest.TestCase):
    def test_occupied_then_expired(self):
        try:
            get_package_prefix('nav2_costmap_2d')
        except PackageNotFoundError:
            self.skipTest('Nav2 costmap runtime not installed')
        rclpy.init()
        node=Node('costmap_test')
        tf=StaticTransformBroadcaster(node)
        transform=TransformStamped()
        transform.header.frame_id='odom'
        transform.child_frame_id='base_link'
        transform.transform.rotation.w=1.
        tf.sendTransform(transform)
        pub=node.create_publisher(OccupancyGrid,'/radar_front/occupancy',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        received=[]
        sub=node.create_subscription(OccupancyGrid,'/costmap/costmap',received.append,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        grid=OccupancyGrid()
        grid.header.frame_id='odom'
        grid.info.width=grid.info.height=200
        grid.info.resolution=.1
        grid.info.origin.position.x=grid.info.origin.position.y=-10.
        grid.info.origin.orientation.w=1.
        grid.data=[-1]*40000
        grid.data[100*200+120]=90
        params=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/nav2_radar.yaml').read_text())
        params=params['local_costmap']['local_costmap']['ros__parameters']
        params.update(publish_frequency=10.,update_frequency=10.,always_send_full_costmap=True)
        log=tempfile.TemporaryFile(mode='w+')
        process=None
        try:
            with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml') as stream:
                yaml.safe_dump({'/**':{'ros__parameters':params}},stream)
                stream.flush()
                process=subprocess.Popen(['ros2','run','nav2_costmap_2d','nav2_costmap_2d',
                    '--ros-args','--params-file',stream.name],stdout=log,stderr=log,start_new_session=True)
                client=node.create_client(ChangeState,'/costmap/costmap/change_state')
                self.assertTrue(client.wait_for_service(timeout_sec=15.))
                def transition(identifier):
                    req=ChangeState.Request()
                    req.transition.id=identifier
                    future=client.call_async(req)
                    rclpy.spin_until_future_complete(node,future,timeout_sec=10.)
                    self.assertTrue(future.done())
                    self.assertTrue(future.result().success)
                transition(1)
                pub.publish(grid)
                transition(3)
                def wait_for(predicate):
                    deadline=time.monotonic()+8.
                    while time.monotonic()<deadline:
                        grid.header.stamp=node.get_clock().now().to_msg()
                        pub.publish(grid)
                        rclpy.spin_once(node,timeout_sec=.05)
                        if received and predicate(received[-1]):
                            return
                    self.fail('Costmap did not reflect processed occupancy')
                wait_for(lambda m:any(v==100 for v in m.data))
                grid.data=[-1]*40000
                received.clear()
                wait_for(lambda m:all(v==0 for v in m.data))
                transition(4)
                transition(2)
                transition(5)
        finally:
            if process:
                os.killpg(process.pid,signal.SIGINT)
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    process.wait()
            log.seek(0)
            output=log.read()
            print(output[-4000:])
            log.close()
            node.destroy_node()
            rclpy.shutdown()
            self.assertNotIn('Segmentation fault', output)
            self.assertEqual(process.returncode, 0)


if __name__=='__main__':
    unittest.main()
