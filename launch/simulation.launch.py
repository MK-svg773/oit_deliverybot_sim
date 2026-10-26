"""Gazebo environment and sensors. No hardware driver is launched."""
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, Shutdown)
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def plugin(parent, kind):
    return ET.SubElement(parent, 'plugin', filename=f'gz-sim-{kind}-system',
                         name='gz::sim::systems::' + ''.join(s.title() for s in kind.split('-')))


def box(world, name, pose, size):
    model = ET.SubElement(world, 'model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = pose
    link = ET.SubElement(model, 'link', name='surface')
    for kind in ('collision', 'visual'):
        geom = ET.SubElement(ET.SubElement(link, kind, name=name), 'geometry')
        ET.SubElement(ET.SubElement(geom, 'box'), 'size').text = size


def truth(model, topic):
    pub = plugin(model, 'odometry-publisher')
    for key, value in {
        'odom_frame': 'world',
        'robot_base_frame': 'truth',
        'dimensions': 3,
        'odom_publish_frequency': 50,
        'odom_topic': topic,
        'tf_topic': topic + '/tf',
    }.items():
        ET.SubElement(pub, key).text = str(value)


def start_simulation(context):
    config = Path(get_package_share_directory('oit_deliverybot_sim')) / 'config/simulation'

    def get(key):
        return LaunchConfiguration(key).perform(context)

    env_path = Path(get('environment')).expanduser().resolve()
    env = yaml.safe_load(env_path.read_text())
    tree = ET.parse(config / 'world.sdf')
    world = tree.getroot().find('world')
    if env['world'] == 'corridor':
        world.remove(world.find('model'))
        box(world, 'floor', '4 0 -0.05 0 0 0', '12 4.2 0.1')
        box(world, 'north_wall', '4 2 0.75 0 0 0', '12 0.15 1.5')
        box(world, 'south_wall', '4 -2 0.75 0 0 0', '12 0.15 1.5')
        box(world, 'west_wall', '-2 0 0.75 0 0 0', '0.2 4.2 1.5')
        box(world, 'east_wall', '10 0 0.75 0 0 0', '0.2 4.2 1.5')
    else:
        mesh_value = get('mesh') or env['mesh']
        mesh = Path(mesh_value).expanduser()
        if not mesh.is_absolute():
            mesh = env_path.parent / mesh
        mesh = mesh.resolve()
        if not mesh.is_file() or mesh.suffix.lower() not in ('.obj', '.stl', '.dae'):
            raise ValueError(f'Provide an existing OBJ/STL/DAE mesh: {mesh}')
        for uri in world.findall('./model/link/*/geometry/mesh/uri'):
            uri.text = mesh.as_uri()
    pose = list(env['start'])
    for i, key in enumerate(('x', 'y', 'z', 'yaw')):
        if get(key):
            pose[i] = float(get(key))
    if not all(math.isfinite(v) for v in pose):
        raise ValueError('Initial pose must be finite')
    robot = ET.parse(config / 'robot.sdf').getroot().find('model')
    ET.SubElement(robot, 'pose').text = f'{pose[0]} {pose[1]} {pose[2]} 0 0 {pose[3]}'
    truth(robot, '/simulation/ground_truth')
    # Contact streams cover all physical robot parts; evaluation excludes support contacts.
    for link in robot.findall('link'):
        if link.get('name') == 'base_link':
            continue
        sensor = ET.SubElement(link, 'sensor', name='contact', type='contact')
        ET.SubElement(sensor, 'topic').text = '/simulation/contacts'
        ET.SubElement(sensor, 'always_on').text = 'true'
        ET.SubElement(sensor, 'update_rate').text = '50'
        ET.SubElement(sensor, 'contact').append(ET.Element('collision'))
        sensor.find('contact/collision').text = link.find('collision').get('name')
        ET.SubElement(sensor.find('contact'), 'topic').text = '/simulation/contacts'
    world.append(robot)
    obstacle = yaml.safe_load((config / 'scenarios.yaml').read_text())[get('scenario')]
    if obstacle and get('obstacle_y'):
        obstacle['y'] = float(get('obstacle_y'))
    if obstacle:
        model = ET.SubElement(world, 'model', name='obstacle')
        ET.SubElement(model, 'pose').text = (
            f"{obstacle['x']} {obstacle['y']} {pose[2] + .6} 0 0 {obstacle['yaw']}")
        link = ET.SubElement(model, 'link', name='body')
        # Keep the cylinder upright and clear of floor friction. VelocityControl
        # controls velocity, not orientation against contact-induced torque.
        ET.SubElement(link, 'gravity').text = 'false'
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = '60'
        inertia = ET.SubElement(inertial, 'inertia')
        for key, value in {
            'ixx': 8,
            'iyy': 8,
            'izz': 2,
        }.items():
            ET.SubElement(inertia, key).text = str(value)
        for kind in ('collision', 'visual'):
            cyl = ET.SubElement(ET.SubElement(ET.SubElement(
                link, kind, name='body'), 'geometry'), 'cylinder')
            ET.SubElement(cyl, 'radius').text = str(env['obstacle_radius'])
            ET.SubElement(cyl, 'length').text = '1.2'
        ctrl = plugin(model, 'velocity-control')
        ET.SubElement(ctrl, 'topic').text = '/simulation/obstacle_cmd'
        truth(model, '/simulation/obstacle_truth')
    temporary = tempfile.TemporaryDirectory(prefix='deliverybot-sim-')
    world_path = Path(temporary.name) / 'world.sdf'
    tree.write(world_path, encoding='utf-8', xml_declaration=True)

    def cleanup(_context):
        temporary.cleanup()
        return []

    cmd = ['gz', 'sim', str(world_path)]
    if get('run') == 'true':
        cmd.append('-r')
    if get('gui') == 'false':
        cmd += ['-s', '--headless-rendering']
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        ExecuteProcess(cmd=cmd, output='screen', on_exit=Shutdown()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='sim_bridge',
             parameters=[{'config_file': str(config / 'bridge.yaml'), 'use_sim_time': True}]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='sim_lidar_tf',
             arguments=['--x', '-0.12', '--y', '0', '--z', '0.6374',
                        '--frame-id', 'sim/base_link', '--child-frame-id', 'sim/hesai_lidar'],
             parameters=[{'use_sim_time': True}]),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
             name='sim_scan', parameters=[{
                 'use_sim_time': True, 'target_frame': 'sim/base_link',
                 'min_height': 0.15, 'max_height': 1.3,
                 'angle_min': -math.pi, 'angle_max': math.pi,
                 'angle_increment': math.pi / 360, 'scan_time': 0.1,
                 'range_min': 0.1, 'range_max': 50.0,
             }], remappings=[('cloud_in', '/sim/lidar_points'), ('scan', '/sim/scan')]),
    ]


def generate_launch_description():
    config = Path(get_package_share_directory('oit_deliverybot_sim')) / 'config/simulation'
    return LaunchDescription([
        DeclareLaunchArgument('environment', default_value=str(config / 'corridor.yaml')),
        DeclareLaunchArgument('mesh', default_value=''),
        DeclareLaunchArgument('obstacle_y', default_value=''),
        *[DeclareLaunchArgument(k, default_value='') for k in ('x', 'y', 'z', 'yaw')],
        DeclareLaunchArgument('scenario', default_value='clear', choices=[
            'clear', 'static', 'crossing', 'following', 'oncoming', 'stop_go', 'speed_change']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('run', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=start_simulation),
    ])
