#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'car_tutorial'

    pkg_path = os.path.join(get_package_share_directory(package_name))
    rviz_file = os.path.join(pkg_path, 'config', 'car.rviz')
    xacro_file = os.path.join(pkg_path, 'urdf', 'car.xacro')
    robot_description = xacro.process_file(xacro_file)

    return LaunchDescription(
        [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[{
                             'robot_description': robot_description.toxml(),
                             'use_sim_time': False,
                            }],
            ),

            Node(
                package='car_tutorial',
                executable='fake_driver',
                output='screen',
            ),

            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_file],
            )
        ]
    )