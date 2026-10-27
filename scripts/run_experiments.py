#!/usr/bin/env python3
"""Connect existing Gazebo/Nav2/scenario_execution to trial-specific evaluation."""
import argparse
from collections import deque
import csv
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from simulation_metrics import (
    clearance_to_circle, interpolate_truth, post_clearance_metrics, support_contact, TrialMetrics)
from simulation_scenarios import BASE_SCENARIOS, position_at, resolve_scenario, speed_at
from std_msgs.msg import String
import yaml


STOP_REQUESTED = False


def request_stop(_signal, _frame):
    # launch may forward SIGINT after the terminal has already delivered it.
    # Repeated signals request one orderly cancellation; cleanup must finish.
    global STOP_REQUESTED
    STOP_REQUESTED = True


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Monitor(Node):
    def __init__(self, environment, obstacle, output):
        super().__init__('experiment_monitor', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.environment, self.obstacle = environment, obstacle
        self.metrics = TrialMetrics(environment['stop_speed'], start_time=0.)
        self.robot = self.other = None
        self.start = None
        self.start_wall = None
        self.end = None
        self.failure_reason = ''
        self.other_history = deque(maxlen=1000)
        self.track_pending = deque(maxlen=100)
        self.track_messages = 0
        self.tracker_wall = 0.
        self.track_detections = 0
        self.schedule_max_error = 0.
        self.pending = deque(maxlen=100)
        self.obstacle_wall = 0.
        self.outcome = None
        self.amcl = False
        self.scan_wall = 0.
        self.truth_wall = 0.
        self.last_sample = -math.inf
        self.last_control = -math.inf
        self.status = 0
        self.contact_count = 0
        self.events = self.create_publisher(String, '/evaluation/events', 10)
        self.stop = self.create_publisher(Twist, '/sim/cmd_vel_raw', 10)
        self.motion = self.create_publisher(Twist, '/evaluation/obstacle_cmd', 10)
        self.cancel = self.create_client(CancelGoal, '/sim/navigate_to_pose/_action/cancel_goal')
        self.subs = [
            self.create_subscription(PointCloud2, '/sim/obstacle_tracks', self.on_tracks, 10),
            self.create_subscription(Odometry, '/evaluation/robot',
                                     self.on_robot, qos_profile_sensor_data),
            self.create_subscription(Odometry, '/evaluation/obstacle',
                                     self.on_other, qos_profile_sensor_data),
            self.create_subscription(LaserScan, '/sim/scan', lambda m: setattr(self,
                                     'scan_wall', time.monotonic()), qos_profile_sensor_data),
            self.create_subscription(
                PoseWithCovarianceStamped, '/sim/amcl_pose', lambda m: setattr(self, 'amcl', True),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            self.create_subscription(
                GoalStatusArray, '/sim/navigate_to_pose/_action/status', self.on_status, 10),
            self.create_subscription(Contacts, '/sim/body_contacts',
                                     self.on_contacts, qos_profile_sensor_data),
        ]
        self.samples = open(output / 'samples.csv', 'w', newline='')
        self.writer = csv.writer(self.samples)
        self.writer.writerow([
            'time_s', 'x_m', 'y_m', 'yaw_rad', 'speed_mps', 'clearance_m',
            'obstacle_x_m', 'obstacle_y_m', 'expected_obstacle_x_m', 'expected_obstacle_y_m',
            'obstacle_position_error_m', 'scheduled_speed_mps'])
        self.track_stream = open(output / 'tracking_errors.csv', 'w', newline='')
        self.track_writer = csv.writer(self.track_stream)
        self.track_writer.writerow([
            'time_s', 'age_s', 'track_count', 'matched', 'false_tracks',
            'x_m', 'y_m', 'vx_mps', 'vy_mps',
            'truth_x_m', 'truth_y_m', 'truth_vx_mps', 'truth_vy_mps',
            'position_error_m', 'velocity_error_mps'])

    def on_status(self, message):
        if not message.status_list:
            return
        self.status = message.status_list[-1].status
        if self.start is None and self.status == 2:
            self.start = self.get_clock().now().nanoseconds / 1e9
            self.start_wall = time.monotonic()
            self.events.publish(String(data='start'))
            self.get_logger().info(
                'Navigation accepted; trial clock and obstacle schedule started')
        if self.start is not None and self.status in (4, 5, 6) and self.outcome is None:
            self.outcome = {4: 'success', 5: 'cancelled', 6: 'navigation_failure'}[self.status]
            self.end = self.get_clock().now().nanoseconds / 1e9 - self.start

    def on_contacts(self, message):
        self.contact_count += 1
        if self.start is None or self.outcome:
            return
        for contact in message.contacts:
            names = contact.collision1.name + ' ' + contact.collision2.name
            if 'floor::' in names and any(
                    k in names for k in ('left_wheel', 'right_wheel', 'caster')):
                continue
            # School has floor and walls in one mesh: allow only support contacts
            # on wheels/caster with a near-vertical normal, never the body box.
            base_z = self.robot.pose.pose.position.z if self.robot else 0.
            if 'school_mesh' in names and support_contact(
                    names, [p.z - base_z for p in contact.positions],
                    [n.z for n in contact.normals]):
                continue
            if contact.positions:
                self.outcome = 'collision'
                self.end = self.get_clock().now().nanoseconds / 1e9 - self.start
                self.events.publish(String(data='collision:' + names))
                break

    @staticmethod
    def stamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec / 1e9

    def on_other(self, message):
        self.other = message
        self.obstacle_wall = time.monotonic()
        t = self.stamp(message)
        if not self.other_history or t > self.other_history[-1][0]:
            a = yaw(message.pose.pose.orientation)
            v = message.twist.twist.linear
            self.other_history.append((
                t, message.pose.pose.position.x, message.pose.pose.position.y,
                math.cos(a) * v.x - math.sin(a) * v.y,
                math.sin(a) * v.x + math.cos(a) * v.y))
        self.drain_samples()
        self.drain_tracks()

    def on_tracks(self, message):
        self.tracker_wall = time.monotonic()
        if self.start is None or self.outcome is not None:
            return
        if message.header.frame_id != 'map':
            self.failure_reason = 'Tracker output frame is not map'
            self.outcome = 'infrastructure_error'
            return
        self.track_pending.append(message)
        self.drain_tracks()

    def drain_tracks(self):
        if self.outcome is not None:
            return
        while self.track_pending:
            cloud = self.track_pending[0]
            stamp = self.stamp(cloud)
            if self.obstacle and (not self.other_history or self.other_history[-1][0] < stamp):
                return
            self.track_pending.popleft()
            if stamp < self.start:
                continue
            rows = list(point_cloud2.read_points(cloud, field_names=(
                'x', 'y', 'vx', 'vy', 'radius'), skip_nans=True))
            self.track_messages += 1
            self.track_detections += len(rows)
            age = self.get_clock().now().nanoseconds / 1e9 - stamp
            truth = None
            for a, b in zip(self.other_history, list(self.other_history)[1:]):
                if a[0] <= stamp <= b[0] and 0 < b[0] - a[0] <= .12:
                    f = (stamp - a[0]) / (b[0] - a[0])
                    truth = [a[i] + f * (b[i] - a[i]) for i in range(1, 5)]
                    break
            if self.obstacle and truth is None:
                continue
            closest = min(rows, key=lambda r: math.hypot(
                float(r[0]) - truth[0], float(r[1]) - truth[1])) if rows and truth else None
            error = (math.hypot(float(closest[0]) - truth[0], float(closest[1]) - truth[1])
                     if closest is not None else None)
            matched = error is not None and error <= 1.0
            values = [float(closest[i]) for i in range(4)] if matched else [None] * 4
            velocity_error = math.hypot(
                values[2] - truth[2], values[3] - truth[3]) if matched else None
            self.track_writer.writerow([
                stamp - self.start, age, len(rows), matched, len(rows) - int(matched),
                *values, *(truth or [None] * 4), error if matched else None, velocity_error])

    def on_robot(self, message):
        self.robot = message
        self.truth_wall = time.monotonic()
        if self.start is None or self.outcome is not None:
            return
        if len(self.pending) == self.pending.maxlen:
            self.outcome = 'infrastructure_error'
            self.failure_reason = 'Obstacle truth lag exceeded pending sample buffer'
            return
        self.pending.append(message)
        self.drain_samples()

    def drain_samples(self):
        if self.start is None or self.outcome is not None:
            return
        while self.pending:
            message = self.pending[0]
            stamp = self.stamp(message)
            if self.obstacle and (not self.other_history or self.other_history[-1][0] < stamp):
                return
            self.pending.popleft()
            elapsed = stamp - self.start
            if elapsed < 0 or elapsed - self.last_sample < .099:
                continue
            ox = oy = gap = None
            p = message.pose.pose
            angle = yaw(p.orientation)
            speed = math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y)
            if self.obstacle:
                try:
                    ox, oy = interpolate_truth(list(self.other_history), stamp)
                except ValueError as exc:
                    self.outcome = 'infrastructure_error'
                    self.failure_reason = str(exc)
                    return
                gap = clearance_to_circle(p.position.x, p.position.y, angle,
                                          ox, oy, self.environment['obstacle_radius'])
            self.last_sample = elapsed
            self.metrics.add(elapsed, p.position.x, p.position.y, speed, gap)
            ex = ey = error = scheduled = None
            if self.obstacle:
                ex, ey = position_at(self.obstacle, elapsed)
                error = math.hypot(ox - ex, oy - ey)
                scheduled = speed_at(self.obstacle, elapsed)
                self.schedule_max_error = max(self.schedule_max_error, error)
            self.writer.writerow([elapsed, p.position.x, p.position.y, angle,
                                 speed, gap, ox, oy, ex, ey, error, scheduled])

    def tick(self):
        if self.start is None:
            return
        elapsed = self.get_clock().now().nanoseconds / 1e9 - self.start
        if self.obstacle and elapsed - self.last_control >= .05:
            self.last_control = elapsed
            command = Twist()
            command.linear.x = speed_at(self.obstacle, elapsed)
            self.motion.publish(command)
        if elapsed > self.environment['timeout'] and self.outcome is None:
            self.outcome = 'timeout'
        if time.monotonic() - self.tracker_wall > 3 and self.outcome is None:
            self.outcome = 'infrastructure_error'
            self.failure_reason = 'Tracker stream stale'
        if self.obstacle and time.monotonic() - self.obstacle_wall > 2 and self.outcome is None:
            self.outcome = 'infrastructure_error'
            self.failure_reason = 'Obstacle truth stream stale'
        if time.monotonic() - min(self.truth_wall, self.scan_wall) > 5 and self.outcome is None:
            self.outcome = 'infrastructure_error'
            self.failure_reason = 'Robot truth or scan stream stale'

    def ready(self):
        return (self.robot is not None and (not self.obstacle or self.other is not None) and
                self.tracker_wall > 0 and self.amcl and self.scan_wall > 0 and
                self.contact_count > 0 and self.cancel.service_is_ready() and
                self.get_clock().now().nanoseconds > 0)

    def finish(self):
        future = self.cancel.call_async(
            CancelGoal.Request()) if self.cancel.service_is_ready() else None
        deadline = time.monotonic() + 5
        stopped = False
        while time.monotonic() < deadline:
            self.stop.publish(Twist())
            self.motion.publish(Twist())
            rclpy.spin_once(self, timeout_sec=.05)
            if future and future.done() and self.robot and time.monotonic() - self.truth_wall < .5:
                v = self.robot.twist.twist.linear
                w = self.robot.twist.twist.angular
                stopped = math.hypot(v.x, v.y) < .02 and abs(w.z) < .03
                if stopped and self.status != 2:
                    break
        self.events.publish(String(data=self.outcome or 'infrastructure_error'))
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=.02)
        self.samples.flush()
        self.samples.close()
        self.track_stream.flush()
        self.track_stream.close()
        return stopped and self.status != 2


def launch_process(command, output, env, handles):
    log = open(output, 'w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                               env=env, start_new_session=True)
    handles.append((process, log))
    return process


def stop_processes(handles):
    # Only process groups created for this trial. Recorder closes on SIGINT.
    for process, _ in reversed(handles):
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
    for process, log in reversed(handles):
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
        log.close()


def run_trial(args, environment, scenario, method, trial, root, config):
    output = root / f'{scenario}_{method}_{trial:03d}'
    output.mkdir()
    seed = args.seed + trial - 1
    obstacle = resolve_scenario(yaml.safe_load(
        (config / 'scenarios.yaml').read_text()), scenario, seed)
    handles = []
    monitor = None
    reason = ''
    decision_wall = None
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=args.domain, GZ_PARTITION=f'deliverybot-{os.getpid()}-{output.name}',
               LIBGL_ALWAYS_SOFTWARE=env.get('LIBGL_ALWAYS_SOFTWARE', '1'))
    try:
        sim = launch_process(
            ['ros2', 'launch', 'oit_deliverybot_sim', 'simulation.launch.py',
             f'environment:={args.environment}', f'scenario:={scenario}',
             f'gui:={args.gui}', 'run:=true',
             *([f"obstacle_y:={obstacle['y']}"] if scenario == 'oncoming' else [])],
            output / 'simulation.log', env, handles)
        nav = launch_process(
            ['ros2', 'launch', 'oit_deliverybot_sim', 'simulation_navigation.launch.py',
             f'environment:={args.environment}', f'method:={method}',
             f'additional_delay:={args.additional_delay}',
             f'velocity_noise_std:={args.velocity_noise_std}'],
            output / 'navigation.log', env, handles)
        monitor = Monitor(environment, obstacle, output)
        deadline = time.monotonic() + args.wall_timeout
        while not monitor.ready():
            if STOP_REQUESTED:
                raise KeyboardInterrupt
            rclpy.spin_once(monitor, timeout_sec=.05)
            if time.monotonic() > deadline or sim.poll() is not None or nav.poll() is not None:
                raise RuntimeError(
                    f'Simulation/Nav2 readiness failed: truth={monitor.robot is not None}, '
                    f'obstacle={monitor.other is not None}, amcl={monitor.amcl}, '
                    f'scan={monitor.scan_wall > 0}, contacts={monitor.contact_count}, '
                    f'action={monitor.cancel.service_is_ready()}; inspect logs')
        topics = ['/clock', '/tf', '/tf_static', '/sim/lidar_points', '/sim/scan', '/sim/odom',
                  '/sim/amcl_pose', '/sim/plan', '/sim/local_plan',
                  '/sim/cmd_vel_raw', '/sim/cmd_vel',
                  '/evaluation/robot', '/evaluation/obstacle',
                  '/evaluation/obstacle_cmd', '/evaluation/events',
                  '/sim/body_contacts', '/sim/navigate_to_pose/_action/status',
                  '/sim/obstacle_tracks',
                  '/sim/local_costmap/costmap_raw', '/sim/local_costmap/published_footprint']
        recorder = launch_process(['ros2', 'bag', 'record', '--storage', 'mcap', '--use-sim-time',
                                   '--include-hidden-topics', '--output', str(output / 'bag'),
                                   '--topics', *topics], output / 'recording.log', env, handles)
        while monitor.events.get_subscription_count() == 0:
            if STOP_REQUESTED:
                raise KeyboardInterrupt
            rclpy.spin_once(monitor, timeout_sec=.05)
            if recorder.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Recorder did not become ready')
        if args.motion_check == 'true':
            monitor.start = monitor.get_clock().now().nanoseconds / 1e9
            monitor.start_wall = time.monotonic()
            monitor.events.publish(String(data='motion_check_start'))
        gx, gy, angle = environment['goal']
        scenario_file = output / 'navigation.osc'
        scenario_file.write_text(
            'import osc.ros\nimport osc.nav2\nscenario deliverybot_navigation:\n'
            '    robot: differential_drive_robot\n    do serial:\n'
            f'        robot.nav_to_pose(pose_3d(position_3d(x: {gx} m, y: {gy} m), '
            f"orientation_3d(yaw: {angle} rad)), namespace_override: '/sim')\n        emit end\n")
        navigator = None if args.motion_check == 'true' else launch_process(
            ['ros2', 'run', 'scenario_execution_ros', 'scenario_execution_ros',
             str(scenario_file), '-o', str(output / 'scenario')],
            output / 'scenario.log', env, handles)
        deadline = time.monotonic() + args.wall_timeout
        while monitor.outcome is None:
            if STOP_REQUESTED:
                raise KeyboardInterrupt
            rclpy.spin_once(monitor, timeout_sec=.02)
            monitor.tick()
            if args.motion_check == 'true':
                monitor.stop.publish(Twist())
                end = (obstacle['delay'] + sum(s['duration']
                       for s in obstacle['segments']) + 1.) if obstacle else 2.
                if monitor.get_clock().now().nanoseconds / 1e9 - monitor.start >= end:
                    monitor.outcome = 'success'
            if time.monotonic() > deadline:
                raise RuntimeError('Wall-time watchdog expired')
            if sim.poll() is not None or nav.poll() is not None or recorder.poll() is not None:
                raise RuntimeError('Simulation, Nav2 or recorder exited unexpectedly')
            if navigator and navigator.poll() is not None and monitor.start is None:
                raise RuntimeError(
                    'Scenario execution exited before goal acceptance; inspect scenario.log')
        decision_wall = time.monotonic()
        if monitor.outcome == 'success' and args.motion_check != 'true':
            # Let the existing scenario executor consume the successful Action result.
            grace = time.monotonic() + 5
            while navigator.poll() is None and time.monotonic() < grace:
                rclpy.spin_once(monitor, timeout_sec=.05)
            p = monitor.robot.pose.pose
            error = math.hypot(p.position.x - gx, p.position.y - gy)
            yaw_error = abs(math.atan2(math.sin(yaw(p.orientation) - angle),
                            math.cos(yaw(p.orientation) - angle)))
            if error > .3 or yaw_error > .4:
                monitor.outcome = 'navigation_failure'
                reason = f'Action success but true goal error {error:.3f}m, {yaw_error:.3f}rad'
    except KeyboardInterrupt:
        if monitor:
            monitor.outcome = 'cancelled'
        reason = 'User interrupted trial'
    except Exception as exc:
        if monitor:
            monitor.outcome = 'infrastructure_error'
        reason = str(exc)
    finally:
        outcome = monitor.outcome if monitor and monitor.outcome else 'infrastructure_error'
        if monitor and monitor.end is None and monitor.start is not None:
            monitor.end = monitor.get_clock().now().nanoseconds / 1e9 - monitor.start
        reason = reason or (monitor.failure_reason if monitor else '')
        result = {
            'scenario': scenario,
            'method': method,
            'trial': trial,
            'seed': seed,
            'seed_set': args.seed_set,
            'actual_delay_s': obstacle['delay'] if obstacle else None,
            'speed_scale': obstacle['speed_scale'] if obstacle else None,
            'actual_segments': json.dumps(obstacle['segments']) if obstacle else '[]',
            'additional_delay_s': args.additional_delay,
            'velocity_noise_std_mps': args.velocity_noise_std,
            'motion_check': args.motion_check,
            'track_messages': monitor.track_messages if monitor else 0,
            'track_detections': monitor.track_detections if monitor else 0,
            'schedule_max_error_m': monitor.schedule_max_error if monitor else None,
            'outcome': outcome,
            'reason': reason,
            'environment': str(args.environment),
            'output': str(output),
            'action_status': monitor.status if monitor else 0,
        }
        result.update(monitor.metrics.result(outcome, terminal_time=monitor.end)
                      if monitor else TrialMetrics().result(outcome))
        run_wall = (decision_wall or time.monotonic()) - \
            monitor.start_wall if monitor and monitor.start_wall else None
        result.update(
            run_wall_s=run_wall,
            real_time_factor=result['duration_s'] / run_wall if run_wall else None,
            shutdown_stopped=False, shutdown_paused=False)
        try:
            if monitor:
                stopped = monitor.finish()
                paused = subprocess.run(
                    ['gz', 'service', '-s', '/world/school/control', '--reqtype',
                     'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '1500', '--req', 'pause: true'],
                    env=env, capture_output=True, text=True, timeout=5)
                result['shutdown_stopped'] = stopped
                result['shutdown_paused'] = paused.returncode == 0 and 'true' in paused.stdout
                if not stopped or not result['shutdown_paused']:
                    result.update(outcome='infrastructure_error', arrival_s=None,
                                  reason=(reason + '; ' if reason else '') +
                                  'Shutdown did not confirm stopped motion and paused physics')
                for _ in range(5):
                    rclpy.spin_once(monitor, timeout_sec=.02)
        except Exception as exc:
            result.update(outcome='infrastructure_error', arrival_s=None,
                          reason=f'Shutdown failed: {exc}')
        finally:
            stop_processes(handles)
            if monitor:
                monitor.destroy_node()
    rows = []
    if obstacle and obstacle.get('post_clearance') and (output / 'samples.csv').exists():
        with (output / 'samples.csv').open() as stream:
            rows = list(csv.DictReader(stream))
    result.update(post_clearance_metrics(rows, environment, obstacle, result['duration_s']))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--environment', type=Path, required=True)
    parser.add_argument('--scenario', default='clear',
                        help='Comma-separated names, or all (original seven scenes)')
    parser.add_argument('--method', default='mppi',
                        help='Comma-separated: dwb,mppi,mppi_pos,mppi_cv,mppi_age')
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--seed_set', choices=['development', 'final'], default='development')
    parser.add_argument('--additional_delay', type=float, default=0.)
    parser.add_argument('--velocity_noise_std', type=float, default=0.)
    parser.add_argument('--motion_check', choices=['true', 'false'], default='false')
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--gui', choices=['true', 'false'], default='false')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain', default='71')
    parser.add_argument('--wall_timeout', type=float, default=600.)
    args = parser.parse_args()
    args.environment = args.environment.expanduser().resolve()
    environment = yaml.safe_load(args.environment.read_text())
    config = Path(get_package_share_directory('oit_deliverybot_sim')) / 'config/simulation'
    available = yaml.safe_load((config / 'scenarios.yaml').read_text())
    scenarios = list(BASE_SCENARIOS) if args.scenario == 'all' else args.scenario.split(',')
    methods = args.method.split(',')
    if (args.repeats < 1 or args.wall_timeout <= 0 or
            any(k not in available for k in scenarios) or
            any(k not in ('dwb', 'mppi', 'mppi_pos', 'mppi_cv', 'mppi_age') for k in methods)):
        parser.error('Invalid repeats, timeout, scenario or method')
    low, high = available['_variation'][args.seed_set + '_seeds']
    if not low <= args.seed <= args.seed + args.repeats - 1 <= high:
        parser.error(f'Seeds for {args.seed_set} must be in [{low}, {high}]')
    if not all(math.isfinite(v) and v >= 0
               for v in (args.additional_delay, args.velocity_noise_std)):
        parser.error('Delay/noise must be finite and nonnegative')
    os.environ['ROS_DOMAIN_ID'] = args.domain
    root = args.output.expanduser().resolve() / datetime.now().strftime('batch_%Y%m%d_%H%M%S_%f')
    root.mkdir(parents=True)
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        with open(root / 'results.csv', 'w', newline='') as stream:
            writer = None
            for scenario in scenarios:
                for trial in range(1, args.repeats + 1):
                    for method in methods:
                        print(f'Start: {scenario} {method} trial {trial}; {root}', flush=True)
                        result = run_trial(args, environment, scenario,
                                           method, trial, root, config)
                        if writer is None:
                            writer = csv.DictWriter(stream, fieldnames=list(result))
                            writer.writeheader()
                        writer.writerow(result)
                        stream.flush()
                        print(f"Result: {result['outcome']} {result['reason']}", flush=True)
                        if result['outcome'] == 'cancelled':
                            from plot_experiments import plot_batch
                            plot_batch(root)
                            return
        from plot_experiments import plot_batch
        plot_batch(root)
        print(f'Results and plots: {root}', flush=True)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
