"""Fuse fixed-frame ROS clouds without synchronizing unrelated radar clocks."""
from .fusion import Fusion


def main():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py.point_cloud2 import read_points, create_cloud
    from std_msgs.msg import Header

    class FusionNode(Node):
        def __init__(self):
            super().__init__('radar_fusion')
            self.fixed=self.declare_parameter('fixed_frame','odom').value
            topics=self.declare_parameter('topics',['/radar_front/points','/radar_rear/points']).value
            self.fusion=Fusion()
            self.pub=self.create_publisher(PointCloud2,'/radar/fused',10)
            self.subs=[self.create_subscription(PointCloud2,topic,
                       lambda msg,sensor=topic:self.receive(sensor,msg),10) for topic in topics]

        def receive(self,sensor,msg):
            if msg.header.frame_id != self.fixed:
                self.get_logger().error('Fusion input must already be in '+self.fixed)
                return
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            now=self.get_clock().now().nanoseconds/1e9
            if abs(now-stamp) > .3:
                return
            points=[tuple(float(v) for v in p) for p in read_points(
                msg,field_names=('x','y','z','doppler'),skip_nans=True)]
            points=self.fusion.update(sensor,stamp,points)
            self.pub.publish(create_cloud(Header(stamp=msg.header.stamp,frame_id=self.fixed),
                                               msg.fields,points))
    from .lifecycle import initialize, begin_cleanup
    shutdown = initialize()
    node=FusionNode()
    try:
        while rclpy.ok() and not shutdown.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        begin_cleanup()
        node.destroy_node()
        rclpy.shutdown()
