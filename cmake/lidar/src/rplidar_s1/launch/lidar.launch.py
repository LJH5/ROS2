#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    lidar_param_dir = LaunchConfiguration(
        'lidar_param_dir',
        default=os.path.join(
            get_package_share_directory('rplidar_s1'),
            'param',
            'lidar.yaml'
        )
    )

    return LaunchDescription([

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_scan_publisher',
            parameters=[lidar_param_dir],
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