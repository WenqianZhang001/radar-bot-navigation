"""Expose radar-facing canonical odometry while retaining native base geometry.

Native ROS odometry/driver are reused. Explicit fixed half-turns connect world
and body frames; no competing dynamic odometry TF is published here.
"""
import copy
import math
from .base_coordinates import canonical_pose,canonical_vector,canonical_covariance,canonical_wheels


def main():
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState,Imu
    from geometry_msgs.msg import TransformStamped
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from .lifecycle import initialize,begin_cleanup
    requested=initialize()

    class Adapter(Node):
        def __init__(self):
            super().__init__('base_frame_adapter')
            self.odom_pub=self.create_publisher(Odometry,'/odom',10)
            self.joint_pub=self.create_publisher(JointState,'/joint_states',10)
            self.native_joint_pub=self.create_publisher(JointState,'/base/native_joint_states',10)
            self.imu_pub=self.create_publisher(Imu,'/base/native_imu',10)
            self.create_subscription(Odometry,'/base/native_odom',self.odom,10)
            self.create_subscription(JointState,'/base/native_joint_states_raw',self.joints,10)
            self.create_subscription(Imu,'/base/native_imu_raw',self.imu,10)
            self.static=StaticTransformBroadcaster(self)
            transforms=[]
            for parent,child,half_turn,z in [('odom','native_odom',True,0.),
                    ('native_base_footprint','base_footprint',True,0.),
                    ('base_footprint','base_link',False,.010)]:
                tr=TransformStamped();tr.header.stamp=self.get_clock().now().to_msg()
                tr.header.frame_id=parent;tr.child_frame_id=child
                tr.transform.translation.z=z
                tr.transform.rotation.z=1. if half_turn else 0.
                tr.transform.rotation.w=0. if half_turn else 1.
                transforms.append(tr)
            self.static.sendTransform(transforms)

        def odom(self,msg):
            if msg.header.frame_id!='native_odom' or msg.child_frame_id!='native_base_footprint':
                self.get_logger().error('Rejected unexpected native odometry frames',throttle_duration_sec=2.)
                return
            p,q=msg.pose.pose.position,msg.pose.pose.orientation
            vectors=[p.x,p.y,p.z,q.x,q.y,q.z,q.w,
                msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z,
                msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z]
            if not all(math.isfinite(v) for v in vectors):return
            out=copy.deepcopy(msg);out.header.frame_id='odom';out.child_frame_id='base_footprint'
            position,rotation=canonical_pose((p.x,p.y,p.z),(q.x,q.y,q.z,q.w))
            out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=position
            out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=rotation
            for vector in [out.twist.twist.linear,out.twist.twist.angular]:vector.x,vector.y,vector.z=canonical_vector((vector.x,vector.y,vector.z))
            out.pose.covariance=canonical_covariance(msg.pose.covariance)
            out.twist.covariance=canonical_covariance(msg.twist.covariance)
            self.odom_pub.publish(out)

        def joints(self,msg):
            if set(msg.name)!={'wheel_left_joint','wheel_right_joint'} or len(msg.position)!=2 or len(msg.velocity)!=2:return
            left,right=msg.name.index('wheel_left_joint'),msg.name.index('wheel_right_joint')
            values=[*msg.position,*msg.velocity]
            if not all(math.isfinite(v) for v in values):return
            native=copy.deepcopy(msg);native.header.frame_id='native_base_link';self.native_joint_pub.publish(native)
            out=copy.deepcopy(msg);out.header.frame_id='base_link'
            out.name=['wheel_left_joint','wheel_right_joint']
            out.position=list(canonical_wheels(msg.position[left],msg.position[right]))
            out.velocity=list(canonical_wheels(msg.velocity[left],msg.velocity[right]))
            # Effort is unused by stock driver; do not invent transformed torque.
            out.effort=[];self.joint_pub.publish(out)

        def imu(self,msg):
            # Keep native orientation/vectors and label the preserved native URDF frame.
            out=copy.deepcopy(msg);out.header.frame_id='native_imu_link';self.imu_pub.publish(out)

    node=Adapter()
    try:
        while rclpy.ok() and not requested.is_set():rclpy.spin_once(node,timeout_sec=.05)
    finally:begin_cleanup();node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
