import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import torch, cv2, os

class YoloV5Detector(Node):
    def __init__(self):
        super().__init__('yolov5_detector')
        self.subscription = self.create_subscription(
            Image,
            'camera/camera/color/image_raw',
            self.listener_callback,
            10)

        # self.subscription = self.create_subscription(
        #     PointCloud2,
        #     'camera/camera/depth/color/points',
        #     self.pointcloud_callback,
        #     1)

        self.subscription = self.create_subscription(
            Image,
            'camera/camera/depth/image_raw',
            self.depth_image_callback,
            10)

        self.x1 = 0
        self.x2 = 0
        self.y1 = 0
        self.y2 = 0
        self.depth_datas = []

        self.bridge = CvBridge()

        self.package_dir = os.path.dirname(os.path.realpath(__file__))  # 현재 패키지 경로
        self.parent_dir = os.path.dirname(self.package_dir)  # 상위 폴더 (부모 디렉토리)

        # YOLOv5 모델 로딩
        self.weight_path = os.path.join(self.parent_dir, 'weights', 'best.pt')
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', self.weight_path)

    def listener_callback(self, msg):
        # ROS2 msgs -> OpenCV 이미지
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.cv_image_copy = cv_image.copy()

        # YOLOv5 모델로 객체 탐지 수행
        results = self.model(cv_image)
        # results.show()

        # tensor 데이터를 pandas형태로 변환
        filtered_results = results.pandas().xyxy[0]

        # 일치율이 50% 이상인 객체만 필터링
        filtered_results = filtered_results[filtered_results['confidence'] >= 0.0]


        # 탐지된 객체가 있을 때만 처리
        if len(filtered_results) > 0:
            for _, row in filtered_results.iterrows():
                self.x1, self.y1, self.x2, self.y2, conf, cls = row[['xmin', 'ymin', 'xmax', 'ymax', 'confidence', 'name']]
                label = f"{cls} {conf:.2f}"

                # ORA(Object Rectangle Area) 객체 인식 박스 설정
                self.x1, self.y1, self.x2, self.y2 = int(self.x1), int(self.y1), int(self.x2), int(self.y2)
                ora = cv_image[self.y1:self.y2, self.x1:self.x2].copy()

                # 객체 중심 좌표
                center_point_x = int((self.x1 + self.x2) * 0.5)
                center_point_y = int((self.y1 + self.y2) * 0.5)
                cv2.circle(self.cv_image_copy, (center_point_x, center_point_y), 5, (0, 0, 255), -1)

                # 없으면 넘어가기
                if ora.size == 0:
                    continue

                # 외곽선 찾기
                gray = cv2.cvtColor(ora, cv2.COLOR_BGR2GRAY)
                _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
                # cv2.imshow("thresh", thresh)
                thresh = cv2.bitwise_not(thresh)
                # cv2.imshow("thresh_rev", thresh)
                contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # 원본 이미지에 외곽선 그리기
                for contour in contours:
                    contour[:, 0, 0] += self.x1
                    contour[:, 0, 1] += self.y1
                    cv2.drawContours(self.cv_image_copy, [contour], -1, (0, 255, 0), 2)

                # 라벨 표시
                cv2.putText(self.cv_image_copy, label, (self.x1, self.y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.imshow('YOLOv5 Object Contour Detection', self.cv_image_copy)
            cv2.waitKey(1)
        else:
            self.get_logger().info('No objects detected with confidence >= 50%')

    def pointcloud_callback(self, msg):
        self.depth_datas = msg.data
        self.get_logger().info(f"type: {type(self.depth_datas)}, len: {len(self.depth_datas)}")

    def depth_image_callback(self, msg):
        data = msg.data
        pass

def main(args=None):
    rclpy.init(args=args)
    node = YoloV5Detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
