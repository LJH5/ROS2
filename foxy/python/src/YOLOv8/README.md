# YOLOv8 ROS2 Package

![Build - foxy](https://img.shields.io/:Build-foxy-yellowgreen.svg)
[![license - Apache-2.0](https://img.shields.io/:license-Apache2.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
 
## Realsense2
### Installation Realsense Camera
``````
$ sudo apt-get install realsense2*
``````
### Running and test Realsense Camera
``````
$ ros2 launch realsense2_camera rs_launch.py
``````
### Using pointcloud
``````
$ ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 pointcloud.enable:=true
``````

---
## Yolov8
[References code and installation link](https://github.com/mgonzs13/yolov8_ros.git)

### Running and test Yolov8
``````
$ ros2 launch realsense2_camera rs_launch.py
$ ros2 launch yolov8_bringup yolov8.launch.py
``````
### Learning customdata
``````
$ cd yolo_ws/
$ ros2 run my_data data_learning
``````
### Using rivz2 with obj coordinate
``````
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30 pointcloud.enable:=true rgb_camera.profile:=640x480x30

ros2 launch yolov8_bringup yolov8.launch.py
``````
When you run the code, you should see the following picture.

For now, I'm only using one camera, but in the future I'll upload the code for how to use this code when multiple cameras are connected.

And rviz2 config file path is hardcoded as an absolute path.

!!!There are no plans to change this not yet, so if you have problems running rviz2, please change the path to the rviz_config_file in yolov8.launch.py to match your environment.!!!

![Alt text](<Screenshot from 2024-01-08 10-12-19.png>)

Run this code if you have more than one rs_camera connected.
``````
ros2 launch yolov8_bringup multi_camera_launch.py

ros2 launch yolov8_bringup yolov8.launch.py
``````
It should work as shown in the picture.

![Alt text](<Screenshot from 2024-01-08 11-39-43.png>)

The serial number of the camera and the rs_camera parameters you want can be modified in the multi_camera_launch.py file in yolov8_bringup.

Of course, if you have more cameras, just copy and add the launch format. (This code is using two cameras.)

However, I don't recommend changing anything except the default value of serial_no in 'name': 'camera_name'.

Serial_no is a unique value for the camera and needs to be changed, but if you change the rest, you will probably need to change the values in other yolov8 files as well.

(If you want to learn more about this code and are studying ROS2, I think it would be a great help to try to make the changes yourself and find the problem! XD)

---
## Build (not yet / Build test required)

``````
$ sudo apt-get install libpoco-dev
$ sudo apt-get install moveit*
$ sudo apt-get install gazebo*
$ sudo apt-get install realsense2*

$ mkdir -p ~/manipulator_ws/src
$ cd ~/manipulator_ws/src
$ git clone https://github.com/cwsfa/Manipulator.git

$ cd ros2_control && git reset --hard 3dc62e28e3bc8cf636275825526c11d13b554bb6 && cd ..
$ cd ros2_controllers && git reset --hard 83c494f460f1c8675f4fdd6fb8707b87e81cb197 && cd ..
$ cd gazebo_ros2_control && git reset --hard 3dfe04d412d5be4540752e9c1165ccf25d7c51fb && cd ..
$ cp doosan-robot2/common2/resource/fake_joint_driver_node.cpp fake_joint/fake_joint_driver/src/fake_joint_driver_node.cpp

$ cd ~/manipulator_ws

$ rm -rf src/doosan-robot2/moveit_config_*/COLCON_IGNORE

$ rosdep update
$ rosdep install --from-paths src --ignore-src --rosdistro foxy -r -y

$ sudo apt-get install ros-foxy-control-msgs ros-foxy-realtime-tools ros-foxy-xacro ros-foxy-joint-state-publisher-gui

$ colcon build --symlink-install
$ . install/setup.bash
``````
## Code test (not yet and will update combine launch code)
``````
$ ros2 run dynamixel_sdk_examples read_write_node
$ ros2 launch dsr_launcher2 single_robot_rviz.launch.py mode:=real host:={IP_ADDRESS} port:=12345
$ ros2 launch dsr_control2 dsr_moveit2.launch.py mode:=real host:={IP_ADDRESS} port:=12345
$ ros2 launch realsense2_camera rs_launch.py
$ ros2 run dsr_example2_py dsr_service_ik_motion_test
$ ros2 run dsr_example2_py dsr_EEctrl_cam.py
$ ros2 run dsr_example2_py camera_test2
``````