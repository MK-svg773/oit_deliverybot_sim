"""Synthetic laser beams through map filtering, clustering and CV tracking."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from scan_tracker_core import detections, StaticMap, Tracker  # noqa: E402


def test_synthetic_scans_remove_wall_and_estimate_velocity():
    info = NS(width=200, height=100, resolution=.05,
              origin=NS(position=NS(x=-2., y=-2.5), orientation=NS(x=0., y=0., z=0., w=1.)))
    grid = np.zeros((100, 200), dtype=int)
    grid[88:, :] = 100
    static = StaticMap(NS(info=info, data=grid.ravel()))
    tracker = Tracker()
    for n in range(31):
        cx, cy, radius = 3. - .03 * n, .2, .25
        ranges = []
        for index in range(720):
            a = -math.pi + index * math.pi / 360
            projection = cx * math.cos(a) + cy * math.sin(a)
            discriminant = projection**2 - (cx**2 + cy**2 - radius**2)
            distance = projection - \
                math.sqrt(discriminant) if discriminant >= 0 and projection > 0 else math.inf
            wall = 1.9 / math.sin(a) if math.sin(a) > .01 else math.inf
            ranges.append(min(distance, wall))
        scan = NS(ranges=ranges, range_min=.1, range_max=30.,
                  angle_min=-math.pi, angle_increment=math.pi / 360)
        obs = detections(scan, (0., 0., 0.), static)
        assert len(obs) == 1
        output = tracker.update(n * .1, obs)
    assert len(output) == 1
    x, y, vx, vy, r = output[0]
    assert abs(x - cx) < .02 and abs(y - cy) < .02
    assert abs(vx + .3) < .03 and abs(vy) < .03
    assert abs(r - .25) < .02
    assert tracker.update(4., []) == []


def test_ros_node_starts_and_preserves_scan_stamp():
    import rclpy
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import LaserScan
    from scan_obstacle_tracker import ScanObstacleTracker
    rclpy.init(domain_id=72)
    node = ScanObstacleTracker()
    output = []
    node.publisher = NS(publish=output.append)
    try:
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'laser'
        transform.transform.rotation.w = 1.
        node.tf.set_transform_static(transform, 'unit_test')
        node.map = NS(near_wall=lambda point: False)
        now = node.get_clock().now().nanoseconds / 1e9
        for i in range(3):
            scan = LaserScan()
            t = now - .4 + i * .1
            scan.header.stamp = Time(sec=int(t), nanosec=int((t - int(t)) * 1e9))
            scan.header.frame_id = 'laser'
            scan.angle_min = -.05
            scan.angle_increment = .01
            scan.range_min, scan.range_max = .1, 10.
            scan.ranges = [
                3 * math.cos(-.05 + j * .01) - math.sqrt(.25**2 - 9 * math.sin(-.05 + j * .01)**2)
                for j in range(11)]
            node.scans.append(scan)
            node.tick()
        assert len(output) == 3
        assert output[-1].width == 1
        assert output[-1].header.frame_id == 'map'
        assert output[-1].header.stamp == scan.header.stamp
        assert [f.name for f in output[-1].fields] == ['x', 'y', 'vx', 'vy', 'radius']
        # Added latency starts after processing; publication must keep scan time.
        node.delay, node.noise = .5, .2
        stamp = now - .1
        scan.header.stamp = Time(sec=int(stamp), nanosec=int((stamp - int(stamp)) * 1e9))
        node.scans.append(scan)
        node.tick()
        assert len(output) == 3
        assert len(node.output) == 1
        from sensor_msgs_py.point_cloud2 import read_points
        delayed = node.output[0][1]
        row = list(read_points(delayed))[0]
        assert abs(float(row['vx'])) + abs(float(row['vy'])) > .01
        node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int((now + 1) * 1e9)))
        node.tick()
        assert len(output) == 4
        assert output[-1].header.stamp == scan.header.stamp

    finally:
        node.destroy_node()
        rclpy.shutdown()
