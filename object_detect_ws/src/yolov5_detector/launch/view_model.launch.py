from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro, tempfile

def to_urdf(xacro_path, parameters=None):
    with tempfile.NamedTemporaryFile(prefix="%s_" % os.path.basename(xacro_path), delete=False) as xacro_file:
        urdf_path = xacro_file.name

    # open and process file
    doc = xacro.process_file(xacro_path, mappings=parameters)
    # open the output file
    with open(urdf_path, 'w') as urdf_file:
        urdf_file.write(doc.toprettyxml(indent='  '))

    return urdf_path

def generate_launch_description():

    # rviz 세팅 파일 경로
    rviz_config_dir = os.path.join(get_package_share_directory('yolov5_detector'), 'config', 'urdf.rviz')
    # xacro 파일 경로
    xacro_path = os.path.join(get_package_share_directory('yolov5_detector'), 'urdf', 'd435_camera.urdf.xacro')
    urdf = to_urdf(xacro_path, {'use_nominal_extrinsics': 'true', 'add_plug': 'true'})

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}]
        )

    model_node = Node(
        name='model_node',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',
        output='screen',
        arguments=[urdf]
        )
    return LaunchDescription([
        rviz_node,
        model_node,
        ])
