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
        Node(
            namespace= "turtlesim",
            package='turtlesim',
            executable='turtlesim_node',
            output='screen'
        ),
    ])
