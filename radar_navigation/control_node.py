"""Processed-perception command gate; dry output and disarmed by default."""
import json
import math
import time
from .control import SafetyGate, reactive


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool, String
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py.point_cloud2 import read_points
    from tf2_ros import Buffer, TransformListener

    class Control(Node):
        def __init__(self):
            super().__init__('radar_control')
            self.gate = SafetyGate()
            self.mode = self.declare_parameter('mode','external').value
            if self.mode not in ('external','reactive'):
                raise ValueError('mode must be external or reactive')
            cloud = self.declare_parameter('points_topic','/radar_front/points').value
            self.base = self.declare_parameter('base_frame','base_link').value
            self.localization = self.declare_parameter('require_localization',False).value
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer,self)
            self.pub = self.create_publisher(Twist,'/radar/cmd_vel_safe',10)
            self.status = self.create_publisher(String,'/radar/control_status',10)
            self.create_subscription(Bool,'/radar/enable',self.arm,10)
            self.create_subscription(Twist,'/radar/cmd_vel_request',self.command,10)
            self.create_subscription(PointCloud2,cloud,self.cloud,10)
            self.create_timer(.05,self.tick)

        def arm(self,msg):
            self.gate.armed_at = time.monotonic() if msg.data else -math.inf

        def command(self,msg):
            if self.mode != 'external':
                return
            if any(v != 0 for v in (msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y)):
                self.gate.command_at = -math.inf
                return
            self.gate.command = msg.linear.x,msg.angular.z
            self.gate.command_at = time.monotonic()

        def cloud(self,msg):
            self.gate.cloud_at = -math.inf
            try:
                age = (self.get_clock().now().nanoseconds-
                       Time.from_msg(msg.header.stamp).nanoseconds)/1e9
                if not 0 <= age <= self.gate.timeout:
                    return
                if self.localization:
                    localization = self.buffer.lookup_transform('map','odom',Time())
                    localization_age=(self.get_clock().now().nanoseconds-
                        Time.from_msg(localization.header.stamp).nanoseconds)/1e9
                    # AMCL deliberately future-dates transforms by its tolerance.
                    if not -1.5 <= localization_age <= .5:
                        return
                # Fixed-world accumulated points transformed to current robot pose.
                tf = self.buffer.lookup_transform(self.base,msg.header.frame_id,Time())
                tf_ns=Time.from_msg(tf.header.stamp).nanoseconds
                if tf_ns and not 0 <= (self.get_clock().now().nanoseconds-tf_ns)/1e9 <= .3:
                    return
                q,t = tf.transform.rotation,tf.transform.translation
                if abs(q.x) > .01 or abs(q.y) > .01:
                    return
                yaw = 2*math.atan2(q.z,q.w)
                c,s = math.cos(yaw),math.sin(yaw)
                points=[]
                for p in read_points(msg,field_names=('x','y','z'),skip_nans=False):
                    x,y,z=map(float,p)
                    if not all(map(math.isfinite,(x,y,z))):
                        return
                    points.append((t.x+c*x-s*y,t.y+s*x+c*y))
                self.gate.points=points
                # Include message age in the monotonic deadline.
                self.gate.cloud_at=time.monotonic()-age
            except Exception as exc:
                self.get_logger().warning('Control rejects perception: '+str(exc), throttle_duration_sec=2.)

        def tick(self):
            now=time.monotonic()
            if self.mode == 'reactive':
                self.gate.command=reactive(self.gate.points)
                self.gate.command_at=now
            linear,angular=self.gate.output(now)
            msg=Twist()
            msg.linear.x, msg.angular.z=linear,angular
            self.pub.publish(msg)
            self.status.publish(String(data=json.dumps({'reason':self.gate.reason,
                'linear':linear,'angular':angular,'mode':self.mode})))

    from .lifecycle import initialize, begin_cleanup
    shutdown = initialize()
    node=Control()
    try:
        while rclpy.ok() and not shutdown.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        begin_cleanup()
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
