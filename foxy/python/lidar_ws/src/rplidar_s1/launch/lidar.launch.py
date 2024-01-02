from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    serial_port1 = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000')
    frame_id1 = LaunchConfiguration('frame_id', default='lidar')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    angle_min = LaunchConfiguration('angle_min', default=1.0)
    angle_max = LaunchConfiguration('angle_max', default=1.1)

    return LaunchDescription([

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port1,
            description='Specifying usb port to connected lidar'
        ),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'
        ),

        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id1,
            description='Specifying frame_id of lidar'
        ),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'
        ),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'
        ),

        DeclareLaunchArgument(
            'angle_min',
            default_value=angle_min,
            description='Specifying whether or not to enable angle_compensate of scan data'
        ),

        DeclareLaunchArgument(
            'angle_max',
            default_value=angle_max,
            description='Specifying whether or not to enable angle_compensate of scan data'
        ),

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_scan_publisher',
            parameters=[{'serial_port': serial_port1,
                         'serial_baudrate': serial_baudrate,
                         'frame_id': frame_id1,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate,
                        }],
            output='screen'
        ),
    ])
