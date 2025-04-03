from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # RealSense 카메라
    realsense2_camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
    )

    # YOLOv5 객체 탐지 노드 실행
    yolov5_node = Node(
        package='yolov5_detector',
        executable='yolo_detector',
        name='yolov5_detector',
        output='screen',
    )

    return LaunchDescription([
        realsense2_camera,
        yolov5_node,
    ])
