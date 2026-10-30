"""AMCL and standard Nav2 controllers, with one shared final command publisher."""
import math
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def navigation_parameters(config, method, delay, calibration_file='', noise=0.):
    """Keep baseline parameters intact and require matching calibrated tables."""
    params = yaml.safe_load((config / 'nav2.yaml').read_text())
    if method.startswith('mppi'):
        params['controller_server']['ros__parameters']['FollowPath'] = yaml.safe_load(
            (config / 'mppi.yaml').read_text())
    if method in ('mppi_pos', 'mppi_cv', 'mppi_age', 'mppi_cvu', 'mppi_cvd'):
        controller = params['controller_server']['ros__parameters']['FollowPath']
        controller['critics'].append('TrackedObstacleCritic')
        controller['TrackedObstacleCritic'] = {
            'use_velocity': method != 'mppi_pos', 'cost_weight': 20.0, 'cost_power': 1,
            'safety_margin': 0.25, 'max_track_age': delay + 1.0}
        if method == 'mppi_age':
            controller['TrackedObstacleCritic']['extrapolate_future'] = False
        if method in ('mppi_cvu', 'mppi_cvd'):
            if method == 'mppi_cvd' and not calibration_file:
                calibration_file = config / 'c2d_margins' / f'noise_{noise}.yaml'
            if not calibration_file:
                raise ValueError('mppi_cvu requires calibration_file')
            table = yaml.safe_load(Path(calibration_file).expanduser().read_text())
            if (not isinstance(table, dict) or
                    table.get('time_steps') != controller['time_steps'] or
                    table.get('model_dt') != controller['model_dt']):
                raise ValueError('calibration time_steps/model_dt must match MPPI')
            margins = table.get('prediction_margins')
            if (not isinstance(margins, list) or len(margins) != controller['time_steps'] or
                    any(type(r) not in (float, int) or not math.isfinite(r) or r < 0
                        for r in margins)):
                raise ValueError(
                    'prediction_margins must contain time_steps finite nonnegative values')
            # Only the fixed double array reaches the critic; calibration labels stay offline.
            controller['TrackedObstacleCritic']['prediction_margins'] = [float(r) for r in margins]
            if method == 'mppi_cvd':
                directions = table.get('directional_prediction_margins')
                if (not isinstance(directions, list) or
                        len(directions) != controller['time_steps'] or
                        any(not isinstance(row, list) or len(row) != 36 or
                            any(type(r) not in (float, int) or not math.isfinite(r) or r < 0
                                for r in row) for row in directions)):
                    raise ValueError(
                        'directional_prediction_margins must contain time_steps x 36 '
                        'finite nonnegative values')
                controller['TrackedObstacleCritic']['directional_prediction_margins'] = [
                    float(r) for row in directions for r in row]
    return params


def start(context):
    config = Path(get_package_share_directory('oit_deliverybot_sim')) / 'config/simulation'
    env_path = Path(LaunchConfiguration('environment').perform(context)).expanduser().resolve()
    env = yaml.safe_load(env_path.read_text())
    method = LaunchConfiguration('method').perform(context)
    delay = float(LaunchConfiguration('additional_delay').perform(context))
    noise = float(LaunchConfiguration('velocity_noise_std').perform(context))
    calibration_file = LaunchConfiguration('calibration_file').perform(context)
    params = navigation_parameters(config, method, delay, calibration_file, noise)
    params['amcl']['ros__parameters']['initial_pose'] = {
        'x': env['start'][0], 'y': env['start'][1], 'z': 0., 'yaw': env['start'][3]}
    map_file = Path(env['map']).expanduser()
    if not map_file.is_absolute():
        map_file = env_path.parent / map_file
    if not map_file.is_file():
        raise ValueError(f'2D map missing: {map_file}')
    params['map_server']['ros__parameters']['yaml_filename'] = str(map_file)
    temporary = tempfile.TemporaryDirectory(prefix='deliverybot-nav2-')
    param_file = Path(temporary.name) / 'params.yaml'
    param_file.write_text(yaml.safe_dump({'sim': params}, sort_keys=False))

    def cleanup(_context):
        temporary.cleanup()
        return []
    specs = [('nav2_map_server', 'map_server'), ('nav2_amcl', 'amcl'),
             ('nav2_planner', 'planner_server'), ('nav2_controller', 'controller_server'),
             ('nav2_behaviors', 'behavior_server'), ('nav2_bt_navigator', 'bt_navigator'),
             ('nav2_velocity_smoother', 'velocity_smoother')]
    actions = [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)]))]
    for package, name in specs:
        remaps = [('odom', '/sim/odom')]
        if name in ('controller_server', 'behavior_server'):
            remaps += [('cmd_vel', '/sim/cmd_vel_raw')]
        if name == 'velocity_smoother':
            remaps += [('cmd_vel', '/sim/cmd_vel_raw'), ('cmd_vel_smoothed', '/sim/cmd_vel')]
        actions.append(Node(package=package, executable=name, name=name, namespace='sim',
                            parameters=[str(param_file)], remappings=remaps, output='screen'))
    actions.append(Node(package='oit_deliverybot_sim', executable='scan_obstacle_tracker.py',
                        remappings=[('/map', '/sim/map')],
                        parameters=[{'use_sim_time': True, 'additional_delay': delay,
                                     'velocity_noise_std': noise}], output='screen'))
    actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                        name='lifecycle_manager', namespace='sim', parameters=[{
                            'use_sim_time': True, 'autostart': True, 'bond_timeout': 10.,
                            'node_names': [n for _, n in specs]}], output='screen'))
    return actions


def generate_launch_description():
    config = Path(get_package_share_directory('oit_deliverybot_sim')) / 'config/simulation'
    return LaunchDescription([
        DeclareLaunchArgument('environment', default_value=str(config / 'corridor.yaml')),
        DeclareLaunchArgument('method', default_value='dwb', choices=[
                              'dwb', 'mppi', 'mppi_pos', 'mppi_cv', 'mppi_age', 'mppi_cvu',
                              'mppi_cvd']),
        DeclareLaunchArgument('calibration_file', default_value=''),
        DeclareLaunchArgument('additional_delay', default_value='0.0'),
        DeclareLaunchArgument('velocity_noise_std', default_value='0.0'),
        OpaqueFunction(function=start)])
