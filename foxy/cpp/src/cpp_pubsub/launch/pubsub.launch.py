from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_pubsub',
            executable='talker',
            name='talker',
        ),
        Node(
            package='cpp_pubsub',
            executable='subscriber_member_function.py',
            name='subscriber',
        ),
    ])