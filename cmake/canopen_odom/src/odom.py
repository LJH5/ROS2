#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
import canopen
import math

class Odom(Node):

    def __init__(self):
        super().__init__("odom")
        qos_profile = QoSProfile(depth = 10)
        self.timer = self.create_timer(1, self.odom_callback)

        self.odom_msg = Odometry()
        self.broad_caseter = TransformBroadcaster(self)
        self.base_link_transform = TransformStamped()

        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"

        self.base_link_transform.header.frame_id = "odom"
        self.base_link_transform.child_frame_id = "base_footprint"

        self.x = 0
        self.y = 0
        self.theta = 0

        # 바퀴 사이 거리 (m)
        self.WHEEL_BASE = 0.425

        # 바퀴 지름 (m)
        self.WHEEL_DIAMETER = 0.173

        # 1바퀴 엔코더
        self.ENCORDER_PER_CYCLE = 16384

        # 엔코더 값 초기화 하기
        init_left_encorder, init_last_right_encorder = canopen.check_encorder()
    def odom_callback(self):

        left_encorder, right_encorder = canopen.check_encorder()

        # 이동 거리 계산
        self.left_distance = left_encorder * self.WHEEL_DIAMETER * math.pi / self.ENCORDER_PER_CYCLE
        self.right_distance = right_encorder * self.WHEEL_DIAMETER * math.pi / self.ENCORDER_PER_CYCLE

        # 회전 각도 계산
        d_theta = (self.right_distance - self.left_distance) / self.WHEEL_BASE
        # 회전 각도 누적
        self.theta += d_theta

        # 이동 좌표 계산
        d_x = ((self.left_distance + self.right_distance) / 2) * math.cos(self.theta)
        d_y = ((self.left_distance + self.right_distance) / 2) * math.sin(self.theta)
        # 이동 좌표 누적
        self.x += d_x
        self.y += d_y

        Q = quaternion_from_euler(0, 0, self.theta)

        self.base_link_transform.header.stamp = self.get_clock().now().to_msg()

        self.base_link_transform.transform.translation.x = self.x
        self.base_link_transform.transform.translation.y = self.y

        self.base_link_transform.transform.rotation.x = Q[0]
        self.base_link_transform.transform.rotation.y = Q[1]
        self.base_link_transform.transform.rotation.z = Q[2]
        self.base_link_transform.transform.rotation.w = Q[3]

        self.broad_caseter.sendTransform(self.base_link_transform)

def main(args=None):
    rclpy.init(args=args)
    odom = Odom()
    rclpy.spin(odom)
    odom.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()