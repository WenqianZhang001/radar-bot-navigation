"""Verify localization loss stops external commands despite fresh radar messages."""
import os
import signal
import subprocess
import tempfile
import time
import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class LocalizationGateTest(unittest.TestCase):
    def test_missing_and_stale_localization(self):
        rclpy.init()
        node=Node('localization_gate_test')
        static=StaticTransformBroadcaster(node)
        dynamic=TransformBroadcaster(node)
        base=TransformStamped()
        base.header.frame_id='odom'
        base.child_frame_id='base_link'
        base.transform.rotation.w=1.
        static.sendTransform(base)
        localization=TransformStamped()
        localization.header.frame_id='map'
        localization.child_frame_id='odom'
        localization.transform.rotation.w=1.
        arm=node.create_publisher(Bool,'/radar/enable',10)
        request=node.create_publisher(Twist,'/radar/cmd_vel_request',10)
        cloud=node.create_publisher(PointCloud2,'/radar_front/points',10)
        output=[]
        sub=node.create_subscription(Twist,'/radar/cmd_vel_safe',output.append,10)
        fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1)
                for i,n in enumerate(('x','y','z','doppler'))]
        log=tempfile.TemporaryFile(mode='w+')
        process=subprocess.Popen(['ros2','run','radar_navigation','radar_control',
            '--ros-args','-p','require_localization:=true'],stdout=log,stderr=log,start_new_session=True)
        try:
            def exercise(duration,localize):
                output.clear()
                deadline=time.monotonic()+duration
                while time.monotonic()<deadline:
                    now=node.get_clock().now().to_msg()
                    if localize:
                        localization.header.stamp=now
                        dynamic.sendTransform(localization)
                    cloud.publish(create_cloud(Header(stamp=now,frame_id='odom'),
                                               fields,[(2.,0.,.3,0.)]))
                    arm.publish(Bool(data=True))
                    cmd=Twist()
                    cmd.linear.x=.08
                    request.publish(cmd)
                    rclpy.spin_once(node,timeout_sec=.02)
                    time.sleep(.01)
            exercise(1.5,False)
            self.assertTrue(output)
            self.assertTrue(all(m.linear.x==0 for m in output))
            exercise(1.,True)
            self.assertTrue(any(m.linear.x>0 for m in output))
            exercise(1.,False)
            self.assertTrue(output)
            self.assertEqual(output[-1].linear.x,0.)
        finally:
            os.killpg(process.pid,signal.SIGINT)
            process.wait(timeout=5.)
            log.seek(0)
            text=log.read()
            print(text[-1000:])
            self.assertNotIn('Traceback',text)
            log.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__=='__main__':
    unittest.main()
