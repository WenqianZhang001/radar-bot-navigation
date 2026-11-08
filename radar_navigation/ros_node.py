"""ROS adapter. Serial work is isolated from executor callbacks."""
import json
import math
import queue
import threading
import time

from .core import Filter, Grid, Pose, ros_points
from .io import acquire, replay
from .cloud_input import decode_cloud


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.duration import Duration
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, PointField
    from sensor_msgs_py.point_cloud2 import create_cloud
    from std_msgs.msg import Header, String
    from nav_msgs.msg import OccupancyGrid
    from tf2_ros import Buffer, TransformListener

    class RadarNode(Node):
        def __init__(self):
            super().__init__('radar_pipeline')
            defaults = dict(cli='', data='', cfg='', sensor='radar_front',
                            input_topic='', replay='', frame_id='radar_front', fixed_frame='odom',
                            min_snr=0., confirmations=1, clear_rays=False,
                            obstacle_ttl=1.5, max_frame_age=.3)
            self.settings = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
            self.raw = self.create_publisher(PointCloud2, 'points_raw', 10)
            self.filtered = self.create_publisher(PointCloud2, 'points', 10)
            self.metadata = self.create_publisher(String, 'metadata', 10)
            self.map_pub = self.create_publisher(OccupancyGrid, 'occupancy',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.status = self.create_publisher(String, 'status', 10)
            self.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                           for i, n in enumerate(('x', 'y', 'z', 'doppler'))]
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.filter = Filter(min_snr=self.settings['min_snr'],
                                 confirmations=self.settings['confirmations'])
            self.grid = Grid(clear=self.settings['clear_rays'])
            self.inbox = queue.Queue(maxsize=20)
            self.stop = threading.Event()
            self.pending = None
            self.last = -math.inf
            self.create_timer(.02, self.process)
            self.create_timer(.1, self.health)
            self.create_timer(.2, self.publish_map)
            self.thread = None
            self.sequence = 0
            if self.settings['input_topic']:
                if any(self.settings[k] for k in ('cli', 'data', 'replay')):
                    raise ValueError('Cloud input cannot be combined with serial or replay')
                self.subscription = self.create_subscription(PointCloud2,
                    self.settings['input_topic'], self.receive_cloud, qos_profile_sensor_data)
            else:
                self.thread = threading.Thread(target=self.worker, daemon=True)
                self.thread.start()

        def receive_cloud(self, msg):
            try:
                frame = decode_cloud(msg, self.settings['sensor'],
                                     self.settings['frame_id'], self.sequence)
                self.sequence += 1
                self.inbox.put_nowait(frame)
            except (ValueError, queue.Full) as exc:
                self.get_logger().warning('Cloud rejected: '+str(exc), throttle_duration_sec=2.)

        def worker(self):
            def consume(frames):
                offset = None
                for frame in frames:
                    if self.stop.is_set():
                        break
                    if self.settings['replay']:
                        if offset is None:
                            offset = self.get_clock().now().nanoseconds-frame.stamp_ns
                        frame.stamp_ns += offset
                    try:
                        self.inbox.put(frame, timeout=.2)
                    except queue.Full:
                        self.get_logger().error('Input queue full: frame dropped')
            try:
                if self.settings['replay']:
                    consume(replay(self.settings['replay']))
                else:
                    with acquire(self.settings['cli'], self.settings['data'],
                                 self.settings['cfg'], self.settings['sensor'], self.stop) as frames:
                        consume(frames)
            except Exception as exc:
                self.get_logger().error(str(exc))

        def health(self):
            stale = time.monotonic()-self.last > .5
            self.status.publish(String(data=json.dumps({'stale': stale})))

        def process(self):
            for _ in range(5):
                first_attempt = self.pending is None
                if first_attempt:
                    try:
                        frame = self.inbox.get_nowait()
                    except queue.Empty:
                        return
                else:
                    frame = self.pending
                    self.pending = None
                age = (self.get_clock().now().nanoseconds-frame.stamp_ns)/1e9
                if not 0 <= age <= self.settings['max_frame_age']:
                    continue
                stamp = Time(nanoseconds=frame.stamp_ns).to_msg()
                header = Header(stamp=stamp, frame_id=self.settings['frame_id'])
                if first_attempt:
                    self.raw.publish(create_cloud(header, self.fields, ros_points(frame)))
                    self.metadata.publish(String(data=json.dumps(frame.to_dict())))
                try:
                    tf = self.buffer.lookup_transform(self.settings['fixed_frame'],
                            self.settings['frame_id'], Time.from_msg(stamp),
                            timeout=Duration(seconds=0.))
                except Exception as exc:
                    self.get_logger().warning('Waiting for measurement-time TF: '+str(exc), throttle_duration_sec=2.)
                    # Let the executor receive newer odometry before retrying.
                    # The max_frame_age deadline above bounds this wait.
                    self.pending = frame
                    return
                t, q = tf.transform.translation, tf.transform.rotation
                # Planar mounting is required: reject roll/pitch rather than silently misproject.
                if abs(q.x) > .01 or abs(q.y) > .01:
                    self.get_logger().error('Only planar radar extrinsics supported')
                    continue
                pose = Pose(t.x, t.y, 2*math.atan2(q.z, q.w), t.z)
                points = self.filter.update(frame, pose)
                self.last = time.monotonic()-age
                header.frame_id = self.settings['fixed_frame']
                self.filtered.publish(create_cloud(header, self.fields, points))
                self.grid.update(self.filter.fresh, (t.x, t.y), frame.stamp_ns/1e9)

        def publish_map(self):
            now = self.get_clock().now()
            self.grid.expire(now.nanoseconds/1e9, self.settings['obstacle_ttl'])
            grid = OccupancyGrid()
            grid.header = Header(stamp=now.to_msg(), frame_id=self.settings['fixed_frame'])
            grid.info.resolution = self.grid.resolution
            grid.info.width, grid.info.height = self.grid.width, self.grid.height
            grid.info.origin.position.x, grid.info.origin.position.y = self.grid.origin
            grid.info.origin.orientation.w = 1.
            grid.data = self.grid.values()
            self.map_pub.publish(grid)

    from .lifecycle import initialize, begin_cleanup
    shutdown = initialize()
    node = RadarNode()
    try:
        while rclpy.ok() and not shutdown.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        begin_cleanup()
        node.stop.set()
        if node.thread is not None:
            node.thread.join(timeout=5.)
        node.destroy_node()
        rclpy.shutdown()
