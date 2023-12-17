#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image   # 이미지 타입의 메시지 처리
from cv_bridge import CvBridge      # ROS2와 OpenCV 이미지 간의 타입 변환을 제공
import cv2                          # OpenCV 라이브러리

class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        timer_period = 0.1

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cap = cv2.VideoCapture(0)

        # ROS2 이미지를 OpenCV 이미지로 변환
        self.bridge = CvBridge()

    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = self.cap.read()

        if ret == True:
        # Publish the image.
        # The 'cv2_to_imgmsg' method converts an OpenCV
        # image to a ROS 2 image message
            self.publisher_.publish(self.bridge.cv2_to_imgmsg(frame))

        # Display the message on the console
        self.get_logger().info('Publishing video frame')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()