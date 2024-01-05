#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    serial_port = LaunchConfiguration("serial_port", default="/dev/ttyUSB0")
    serial_baudrate = LaunchConfiguration("serial_baudrate", default="256000")
    frame_id = LaunchConfiguration("frame_id", default="laser")
    # 라이다 설치 상하 반전
    inverted = LaunchConfiguration("inverted", default="true")
    # 라이다 성능 향상, SLAM 사용 시 필수
    angle_compensate = LaunchConfiguration("angle_compensate", default="true")

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value=serial_port,
            description="lidar connected to usb"),

        DeclareLaunchArgument(
            "serial_baudrate",
            default_value=serial_baudrate,
            description="usb port baudrate"),

        DeclareLaunchArgument(
            "frame_id",
            default_value=frame_id,
            description="lidar's frame_id"),

        DeclareLaunchArgument(
            "inverted",
            default_value=inverted,
            description="Is the lidar mounted upside down"),

        DeclareLaunchArgument(
            "angle_compensate",
            default_value=angle_compensate,
            description="angle_compensate of scan data"),

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_scan_publisher',
            parameters=[{'serial_port': serial_port,
                         'serial_baudrate': serial_baudrate,
                         'frame_id': frame_id,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate,
                         'topic_name': '/scan1',
                         }],
            output='screen',
        ),

        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("rplidar_s1"),
                    "config", "laser_filter.yaml",
                ])],
            remappings=[
                ('/scan', '/scan1'),
                ('/scan_filtered', '/scan_1'),
                ],
        ),

    ])