import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('parameters'),
            'param',
            'param.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description="Full path of parameter file"),

        Node(
            package='parameters',
            executable='minimal_param_node',
            name='minimal_param_node',
            parameters=[
                # {"my_parameter": "earth"}
                param_dir,
            ],
            emulate_tty=True,
            output='screen'),
    ])