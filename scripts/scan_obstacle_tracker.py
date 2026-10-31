#!/usr/bin/env python3
"""Map-frame scan-only tracks; original scan stamp survives added latency."""
from collections import deque
import math
import signal

from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from scan_tracker_core import detections, rotation, StaticMap, Tracker
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


class ScanObstacleTracker(Node):
    """Publish delayed scan-only tracks using observation-time TF."""

    def __init__(self):
        """Connect scan, map, TF and the standard point cloud output."""
        super().__init__('scan_obstacle_tracker')
        self.delay = self.declare_parameter('additional_delay', 0.).value
        self.noise = self.declare_parameter('velocity_noise_std', 0.).value
        self.map_exclusion_distance = self.declare_parameter('map_exclusion_distance', .20).value
        if not math.isfinite(self.map_exclusion_distance) or self.map_exclusion_distance < 0:
            raise ValueError('Map exclusion distance must be finite and nonnegative')
        if not all(math.isfinite(v) and v >= 0 for v in (self.delay, self.noise)):
            raise ValueError('Delay and noise standard deviation must be finite and nonnegative')
        self.rng = np.random.default_rng(0)
        self.map = None
        self.tracker = Tracker()
        self.scans, self.output = deque(maxlen=30), deque()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.publisher = self.create_publisher(PointCloud2, '/sim/obstacle_tracks', 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.scan_sub = self.create_subscription(
            LaserScan, '/sim/scan', lambda message: self.scans.append(message),
            qos_profile_sensor_data)
        self.timer = self.create_timer(.02, self.tick)
        self.fields = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                       for i, name in enumerate(('x', 'y', 'vx', 'vy', 'radius'))]
        self.announced = False

    def on_map(self, message):
        """Cache occupancy distances from a map-frame static map."""
        if message.header.frame_id != 'map':
            self.get_logger().error('Static map must have frame_id=map')
            return
        self.map = StaticMap(message, self.map_exclusion_distance)

    def tick(self):
        """Process scans with valid TF and release delayed outputs."""
        now = self.get_clock().now().nanoseconds / 1.e9
        if self.map is not None:
            while self.scans:
                scan = self.scans[0]
                stamp = Time.from_msg(scan.header.stamp)
                try:
                    tf = self.tf.lookup_transform('map', scan.header.frame_id, stamp,
                                                  timeout=Duration(seconds=0.))
                except TransformException:
                    if now - stamp.nanoseconds / 1.e9 > .5:
                        self.scans.popleft()
                        self.get_logger().warning('Scan dropped: no TF at observation stamp',
                                                  throttle_duration_sec=5.)
                        continue
                    break
                self.scans.popleft()
                t = tf.transform
                obs = detections(scan, (t.translation.x, t.translation.y,
                                        rotation(t.rotation)), self.map)
                tracks = self.tracker.update(stamp.nanoseconds / 1.e9, obs)
                rows = []
                for x, y, vx, vy, radius in tracks:
                    vx, vy = np.array([vx, vy]) + self.rng.normal(0., self.noise, 2)
                    rows.append((x, y, vx, vy, radius))
                message = point_cloud2.create_cloud(
                    Header(stamp=scan.header.stamp, frame_id='map'), self.fields, rows)
                self.output.append((now + self.delay, message))
                if rows and not self.announced:
                    self.get_logger().info(
                        'Confirmed scan tracks published in map; no truth inputs')
                    self.announced = True
        while self.output and self.output[0][0] <= now:
            self.publisher.publish(self.output.popleft()[1])


def main():
    # Stop spinning before destroying the context. Repeated launch SIGINTs must
    # not invalidate a subscription while its message is being taken.
    """Run the tracker until orderly shutdown is requested."""
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ScanObstacleTracker()
    stopping = False

    def request_stop(_signum, _frame):
        """Request shutdown without invalidating active subscriptions."""
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
