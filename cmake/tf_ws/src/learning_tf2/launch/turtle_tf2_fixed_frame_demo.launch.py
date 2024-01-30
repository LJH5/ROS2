import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    turtle_tf2_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('learning_tf2'), 'launch'),
            '/turtle_tf2_demo.launch.py']),
            # launch_arguments={'target_frame': 'carrot1'}.items()
        )
    return LaunchDescription([
        turtle_tf2_demo,
        Node(
            package='learning_tf2',
            executable='fixed_frame_tf2_broadcaster',
            name='fixed_broadcaster',
        ),
    ])