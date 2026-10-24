"""One command for reproducible Nav2 experiments; each trial owns its processes."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package='oit_deliverybot_sim'
    share=Path(get_package_share_directory(package))
    script=Path(get_package_prefix(package))/'lib'/package/'run_experiments.py'
    defaults={
        'environment':str(share/'config/simulation/corridor.yaml'),
        'scenario':'clear', 'method':'mppi', 'repeats':'1', 'gui':'false',
        'output':str(Path.home()/'ros2_ws/experiments/simulation'), 'domain':'71',
        'wall_timeout':'600', 'seed':'1', 'seed_set':'development',
        'additional_delay':'0.0', 'velocity_noise_std':'0.0', 'motion_check':'false',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()],
        SetEnvironmentVariable('ROS_DOMAIN_ID',LaunchConfiguration('domain')),
        ExecuteProcess(cmd=['python3',str(script),
            *[part for k in defaults for part in ('--'+k,LaunchConfiguration(k))]],
            sigterm_timeout='30',sigkill_timeout='10',output='screen'),
    ])
