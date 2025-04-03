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
            1)

        self.bridge = CvBridge()

        self.package_dir = os.path.dirname(os.path.realpath(__file__))  # 현재 패키지 경로
        self.parent_dir = os.path.dirname(self.package_dir)  # 상위 폴더 (부모 디렉토리)

    def listener_callback(self, msg):
        # ROS2 msgs -> OpenCV 이미지
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_img_copy = cv_img.copy()
        cv2.imshow('origin', cv_img)

        # BGR -> GRAY
        grayed_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        cv2.imshow('Gray', grayed_img)

        # 노이즈 제거용 블러
        blured_img = cv2.GaussianBlur(grayed_img, (5, 5), 0)
        cv2.imshow("GaussianBlur", blured_img)
        blured_img = cv2.bilateralFilter(grayed_img, 9, 150, 150)
        cv2.imshow("bilateralFilter", blured_img)

        # 이미지 픽셀의 중간값으로 low, high 설정
        # median_value = np.median(blured_img)
        median_value, _ = cv2.threshold(blured_img, 0, 255, cv2.THRESH_OTSU)
        low = int(max(0, 0.1 * median_value))
        high = int(min(255, 0.3 * median_value))
        edges = cv2.Canny(blured_img, low, high)
        cv2.imshow("edges", edges)

        # 엣지 연결을 위한 팽창 연산 적용
        kernel = np.ones((3,3), np.uint8)
        edges_dilated = cv2.dilate(edges, kernel, iterations=1)
        cv2.imshow("Edge Dilated", edges_dilated)

        # 닫힘 연산 적용
        edges_closed = cv2.morphologyEx(edges_dilated, cv2.MORPH_CLOSE, kernel)
        cv2.imshow("Edge Closed", edges_closed)

        # 외곽선 검출
        contours, _ = cv2.findContours(edges_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 원본 이미지에 외곽선 그리기
        for contour in contours:
            cv2.drawContours(cv_img_copy, [contour], -1, (0, 255, 0), cv2.FILLED)

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
