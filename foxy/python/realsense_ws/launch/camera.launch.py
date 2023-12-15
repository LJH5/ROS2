"""Launch realsense2_camera node without rviz2."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import sys
import pathlib, logging

sys.path.append(str(pathlib.Path(__file__).parent.absolute()))

local_parameters = [
                    {'name': 'camera_name1', 'default': 'camera_1', 'description': 'camera unique name'},
                    {'name': 'serial_no1', 'default': '_233622073534', 'description': 'serial id number'},  # edit default value
                    {'name': 'align_depth.enable_1', 'default': 'true', 'description': ''},
                   ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):

    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():
    return LaunchDescription(
        declare_configurable_parameters(local_parameters) +
        [
        # launch에서 다른 launch 불러오기
        IncludeLaunchDescription(
            # python 코드를 사용하여 launch 파일을 생성
            PythonLaunchDescriptionSource([FindPackageShare('realsense2_camera'), '/launch/rs_launch.py']),
            launch_arguments=set_configurable_parameters(local_parameters).items(),
        ),
    ])
