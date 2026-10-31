"""Synthetic laser beams through map filtering, clustering and CV tracking."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from scan_tracker_core import detections, StaticMap, Tracker  # noqa: E402


@pytest.mark.parametrize('resolution,default_count', [(.2, 5), (float(np.float32(.2)), 1)])
def test_map_exclusion_preserves_legacy_boundary_and_accepts_override(resolution, default_count):
    """Keep the exact 0.20 boundary by default; 0.30 also removes diagonal neighbours."""
    info = NS(width=5, height=5, resolution=resolution,
              origin=NS(position=NS(x=0., y=0.), orientation=NS(x=0., y=0., z=0., w=1.)))
    grid = np.zeros((5, 5), dtype=int)
    grid[2, 2] = 100
    message = NS(info=info, data=grid.ravel())
    points = [((x + .5) * resolution, (y + .5) * resolution)
              for y in range(5) for x in range(5)]
    default = StaticMap(message)
    explicit = StaticMap(message, map_exclusion_distance=.20)
    wider = StaticMap(message, map_exclusion_distance=.30)
    assert sum(default.near_wall(p) for p in points) == default_count
    assert [default.near_wall(p) for p in points] == [explicit.near_wall(p) for p in points]
    assert sum(wider.near_wall(p) for p in points) == 9
    assert default.near_wall((-1., -1.)) and wider.near_wall((-1., -1.))


@pytest.mark.parametrize('override,expected', [(None, False), (.30, True)])
def test_ros_map_exclusion_parameter_reaches_map_filter(override, expected):
    """Catch a declared parameter that never reaches map filtering."""
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from scan_obstacle_tracker import ScanObstacleTracker
    args = [] if override is None else ['--ros-args', '-p', f'map_exclusion_distance:={override}']
    rclpy.init(args=args, domain_id=72)
    node = ScanObstacleTracker()
    try:
        message = OccupancyGrid()
        message.header.frame_id = 'map'
        message.info.width = message.info.height = 5
        message.info.resolution = float(np.float32(.2))
        message.info.origin.orientation.w = 1.
        message.data = [0] * 25
        message.data[12] = 100
        node.on_map(message)
        assert bool(node.map.near_wall((.7, .7))) is expected
    finally:
        node.destroy_node()
        rclpy.shutdown()


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
