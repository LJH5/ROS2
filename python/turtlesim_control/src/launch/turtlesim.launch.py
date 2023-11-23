"""Launch realsense2_camera node without rviz2."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import sys
import pathlib

sys.path.append(str(pathlib.Path(__file__).parent.absolute()))

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('turtlesim1', default_value='turtlesim_1', description=''),
        Node(
            namespace= "turtlesim_1",
            package='turtlesim',
            executable='turtlesim_node',
            output='screen',
            parameters=[
                {"background_r": 200},
                {"background_g": 200},
                {"background_b": 200},
            ],
        ),
    ])
