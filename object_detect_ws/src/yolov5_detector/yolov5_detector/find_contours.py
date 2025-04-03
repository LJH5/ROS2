import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch, cv2, os
import numpy as np

class FindContours(Node):
    def __init__(self):
        super().__init__('find_contours')
        self.subscription = self.create_subscription(
            Image,
            'camera/camera/color/image_raw',
            self.listener_callback,
            10)

        self.bridge = CvBridge()

        self.package_dir = os.path.dirname(os.path.realpath(__file__))  # 현재 패키지 경로
        self.parent_dir = os.path.dirname(self.package_dir)  # 상위 폴더 (부모 디렉토리)

    def listener_callback(self, msg):
        # ROS2 msgs -> OpenCV 이미지
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_img_copy = cv_img.copy()

        # BGR -> GRAY
        grayed_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        cv2.imshow('Gray', grayed_img)

        # 노이즈 제거용 블러
        blured_img = cv2.GaussianBlur(grayed_img, (5, 5), 0)
        cv2.imshow("blur", blured_img)

        # 이미지 이진화
        thresh = cv2.adaptiveThreshold(blured_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
        cv2.imshow("thresh", thresh)

        # 외곽선 검출
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 원본 이미지에 외곽선 그리기
        for contour in contours:
            cv2.drawContours(cv_img_copy, [contour], -1, (0, 255, 0), 2)

        cv2.imshow('Find Contours', cv_img_copy)

        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = FindContours()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
