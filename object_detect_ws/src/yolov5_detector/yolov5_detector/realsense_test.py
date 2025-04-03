import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import torch, cv2, os
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class RealsenseTest(Node):
    def __init__(self):
        super().__init__('realsense_test')
        self.subscription = self.create_subscription(
            Image,
            'camera/camera/depth/image_rect_raw',
            self.depth_callback,
            1
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            'camera/camera/depth/color/points',
            self.pointcloud_callback,
            1
        )
        self.bridge = CvBridge()

    def depth_callback(self, msg):
        try:
            # realsense 토픽을 opencv 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")

            # 깊이 이미지를 시각화하기 위해 정규화합니다.
            normalized_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)

            # uint8로 변환하여 cv2.imshow에서 정상적으로 표시되도록 함.
            normalized_image = np.uint8(normalized_image)

            # 깊이 이미지 표시
            cv2.imshow("Depth Image", normalized_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().info(f"오류 발생: {e}")

    def pointcloud_callback(self, msg):
        # pointcloud data 가져오기
        pc2_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        # numpy 배열로 변환
        points = np.array(list(pc2_data))
        if points.size == 0:
            return
        # z값만 추출
        z_values = points[:, 2]
        depth_image = z_values.reshape((640, 480))
        # OpenCV로 이미지 시각화
        depth_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        # uint8로 변환하여 시각화
        depth_image = np.uint8(depth_image)
        cv2.imshow("PointCloud2", depth_image)

def main(args=None):
    rclpy.init(args=args)
    node = RealsenseTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
