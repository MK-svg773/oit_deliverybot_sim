"""AMCL and standard Nav2 controllers, with one shared final command publisher."""
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    config=Path(get_package_share_directory('oit_deliverybot_sim'))/'config/simulation'
    env_path=Path(LaunchConfiguration('environment').perform(context)).expanduser().resolve()
    env=yaml.safe_load(env_path.read_text())
    params=yaml.safe_load((config/'nav2.yaml').read_text())
    method=LaunchConfiguration('method').perform(context)
    delay=float(LaunchConfiguration('additional_delay').perform(context))
    noise=float(LaunchConfiguration('velocity_noise_std').perform(context))
    if method.startswith('mppi'):
        params['controller_server']['ros__parameters']['FollowPath']=yaml.safe_load((config/'mppi.yaml').read_text())
    if method in ('mppi_pos','mppi_cv'):
        controller=params['controller_server']['ros__parameters']['FollowPath']
        controller['critics'].append('TrackedObstacleCritic')
        controller['TrackedObstacleCritic']={
            'use_velocity':method=='mppi_cv','cost_weight':20.0,'cost_power':1,
            'safety_margin':0.25,'max_track_age':delay+1.0}
    params['amcl']['ros__parameters']['initial_pose']={
        'x':env['start'][0], 'y':env['start'][1], 'z':0., 'yaw':env['start'][3]}
    map_file=Path(env['map']).expanduser()
    if not map_file.is_absolute():map_file=env_path.parent/map_file
    if not map_file.is_file():raise ValueError(f'2D map missing: {map_file}')
    params['map_server']['ros__parameters']['yaml_filename']=str(map_file)
    temporary=tempfile.TemporaryDirectory(prefix='deliverybot-nav2-')
    param_file=Path(temporary.name)/'params.yaml'
    param_file.write_text(yaml.safe_dump({'sim':params},sort_keys=False))
    def cleanup(_context):
        temporary.cleanup()
        return []
    specs=[('nav2_map_server','map_server'),('nav2_amcl','amcl'),
           ('nav2_planner','planner_server'),('nav2_controller','controller_server'),
           ('nav2_behaviors','behavior_server'),('nav2_bt_navigator','bt_navigator'),
           ('nav2_velocity_smoother','velocity_smoother')]
    actions=[RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)]))]
    for package,name in specs:
        remaps=[('odom','/sim/odom')]
        if name in ('controller_server','behavior_server'):remaps+=[('cmd_vel','/sim/cmd_vel_raw')]
        if name=='velocity_smoother':remaps+=[('cmd_vel','/sim/cmd_vel_raw'),('cmd_vel_smoothed','/sim/cmd_vel')]
        actions.append(Node(package=package,executable=name,name=name,namespace='sim',
                            parameters=[str(param_file)],remappings=remaps,output='screen'))
    actions.append(Node(package='oit_deliverybot_sim',executable='scan_obstacle_tracker.py',
                        remappings=[('/map','/sim/map')],
                        parameters=[{'use_sim_time':True,'additional_delay':delay,
                                     'velocity_noise_std':noise}],output='screen'))
    actions.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
                        name='lifecycle_manager',namespace='sim',parameters=[{
                            'use_sim_time':True,'autostart':True,'bond_timeout':10.,
                            'node_names':[n for _,n in specs]}],output='screen'))
    return actions


def generate_launch_description():
    config=Path(get_package_share_directory('oit_deliverybot_sim'))/'config/simulation'
    return LaunchDescription([
        DeclareLaunchArgument('environment',default_value=str(config/'corridor.yaml')),
        DeclareLaunchArgument('method',default_value='dwb',choices=['dwb','mppi','mppi_pos','mppi_cv']),
        DeclareLaunchArgument('additional_delay',default_value='0.0'),
        DeclareLaunchArgument('velocity_noise_std',default_value='0.0'),
        OpaqueFunction(function=start)])
