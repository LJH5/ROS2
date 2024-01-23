# ###############일단 되긴함 근데 막 오밀조밀하게 모여있으면 뭔가 하나는 뎁스값이 안나올꺼임
# ####참고로 인텔 카메라는 최소 280mm 떨어져 있어야 값이나옴. 안그럼 아예 안나오거나 값이 ㅈㄴ이상하게 튐
# ####마커 지우는거 추가해야됨!!!!! 23.12.29
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, PointCloud2
# from yolov8_msgs.msg import DetectionArray
# from cv_bridge import CvBridge
# import numpy as np

# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import PointStamped


# class ObjCoordinateNode(Node):
#     def __init__(self):
#         super().__init__('obj_coordinate_node')

#         # RGB 이미지 및 Depth 이미지 구독 -> 일단 뭘 불러와야 처리를 하든 말든 하지
#         self.subscription_image = self.create_subscription(
#             Image,
#             '/camera/color/image_raw',  # 실제 사용 중인 RGB 카메라 이미지 토픽 이름으로 변경
#             self.image_callback,
#             10
#         )
#         self.subscription_depth = self.create_subscription(
#             Image,
#             '/camera/depth/image_rect_raw',  # 실제 사용 중인 Depth 카메라 이미지 토픽 이름으로 변경
#             self.depth_callback,
#             10
#         )

#         # 물체 감지 결과 구독 -> 여기서 좌표 정보, 객체이름, ID 이런거 나옴
#         # ros2 topic echo /yolo/detections 이랑 ros2 topic echo /yolo/tracking 찍어보면
#         # 차이는 ID값 밖에 없음. 이게 없으면 물체가 여러개 있을 때 class_name만 받아오니까 나온 좌표가 어느 물체인지 한눈에 보기 어려움
#         # 그래서 tracking 구독해서 그 안에 있는 ID 갔다가 썻음
#         self.subscription_tracking = self.create_subscription(
#             DetectionArray,
#             '/yolo/tracking',
#             self.detections_callback,
#             10
#         )

#         self.bridge = CvBridge()
#         # 이거 리스트로 받는 이유가 여러 물체 감지할 때 받은 자료 관리 더 쉽게 하려고
#         self.detected_objects = []

#         # Marker Publisher 추가
#         self.marker_publisher = self.create_publisher(Marker, '/object_marker', 10)

#         # Point Publisher 추가
#         self.point_publisher = self.create_publisher(PointStamped, '/object_3d_coordinates', 10)

#     def image_callback(self, msg):
#         # RGB 이미지 처리 opencv로 쓸 수 있게 처리
#         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#     def detections_callback(self, msg):
#         # 물체 감지 결과 섭스크라이버 한거 내가 쓸거에 맞게 처리
#         self.detected_objects = [
#             {
#                 'class_name': detection.class_name,
#                 'id': detection.id,
#                 'bbox': {
#                     'center': {
#                         'position': {'x': detection.bbox.center.position.x, 'y': detection.bbox.center.position.y}
#                     }
#                 }
#             } for detection in msg.detections
#         ]

#     def depth_callback(self, msg):
#         # Depth 이미지 opencv로 쓸 수 있게 처리
#         depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

#         for obj in self.detected_objects:
#             # 2D 좌표 -> 인식된 객체의 박스 중심 좌표
#             x2d, y2d = int(obj['bbox']['center']['position']['x']), int(obj['bbox']['center']['position']['y'])

#             # 2D 좌표를 3D 좌표로 변환 -> 박스 중심 좌표 가져와서 그 픽셀에서의 depth값을 가져오고 하는거
#             depth_value = depth_image[y2d, x2d]  # 중심 좌표에서의 깊이 값
#             point_3d = self.convert_depth_pixel_to_point(x2d, y2d, depth_value)

#             # #물체 감지 결과 로그에 추가. 주석처리 해놓을테니 보고 싶으면 풀어보셈
#             # self.get_logger().info("Object %s %s Center Point in 3D Point Cloud: %s" % (obj['class_name'], obj['id'], str(point_3d)))

#             if point_3d != [0.0, 0.0, 0.0]:
#                 # 3D 좌표를 RViz2에 시각화하기 위해 Marker 메시지 생성
#                 marker = Marker()
#                 marker.header.frame_id = '/camera_link'  # Replace with your camera frame ID
#                 marker.header.stamp = self.get_clock().now().to_msg()
#                 marker.type = Marker.SPHERE
#                 marker.action = Marker.ADD
#                 #자료형이랑 단위 맞춰준거 mm -> m로 변환 *0.001이 그거임
#                 #원래는 TF써서 좌표축 돌려야 하는데 그냥 귀찮아서 일단 대강 맞춰놈
#                 marker.pose.position.x = float(point_3d[2])*0.001
#                 marker.pose.position.y = float(-(point_3d[0]))*0.001
#                 marker.pose.position.z = float(-(point_3d[1]))*0.001
#                 marker.pose.orientation.w = 1.0
#                 marker.scale.x = 0.1  # Adjust the size of the marker
#                 marker.scale.y = 0.1
#                 marker.scale.z = 0.1
#                 marker.color.a = 1.0  # Fully opaque
#                 marker.color.r = 1.0  # Red color
#                 marker.color.g = 0.0
#                 marker.color.b = 0.0
#                 marker.id = int(obj['id'])  # Use object ID as marker ID
#                 # 빨간색 점으로 퍼블리시되는거
#                 self.marker_publisher.publish(marker)

#                 # 3D 좌표를 RViz2에 시각화하기 위해 Marker 메시지 생성 -> 이름띄우는거
#                 text_marker = Marker()
#                 text_marker.header.frame_id = '/camera_link'  # Replace with your camera frame ID
#                 text_marker.header.stamp = self.get_clock().now().to_msg()
#                 text_marker.type = Marker.TEXT_VIEW_FACING
#                 text_marker.action = Marker.ADD
#                 text_marker.pose.position.x = float(point_3d[2]) * 0.001
#                 text_marker.pose.position.y = float(-(point_3d[0])) * 0.001
#                 text_marker.pose.position.z = float(-(point_3d[1])) * 0.001
#                 text_marker.pose.orientation.w = 1.0
#                 text_marker.scale.x = 0.1
#                 text_marker.scale.y = 0.1
#                 text_marker.scale.z = 0.1
#                 text_marker.color.a = 1.0
#                 text_marker.color.r = 1.0
#                 text_marker.color.g = 0.0
#                 text_marker.color.b = 0.0
#                 text_marker.id = (marker.id + 1024)  # Use object ID as marker ID 1024는 그냥 임의. 위에 마커아이디랑 숫자만 안곂치면됨
#                 text_marker.text = f"{obj['class_name']}{obj['id']}"

#                 # Publish the Text Marker for visualization in RViz2
#                 self.marker_publisher.publish(text_marker)

#                 # Publish the 3D coordinates as PointStamped for additional visualization
#                 self.publish_3d_coordinates_as_point_stamped(point_3d)
    
#     def publish_3d_coordinates_as_point_stamped(self, point_3d):
#         point_msg = PointStamped()
#         point_msg.header.stamp = self.get_clock().now().to_msg()
#         point_msg.header.frame_id = '/camera_link'  # Replace with your camera frame ID
#         point_msg.point.x, point_msg.point.y, point_msg.point.z = point_3d

#         # Publish the PointStamped message
#         self.point_publisher.publish(point_msg)

#         # #Log the 3D coordinates for debugging 이것도 그냥 프린트문
#         # self.get_logger().info("3D Coordinates: %s" % str(point_3d))

#     def convert_depth_pixel_to_point(self, x, y, depth_value):
#         # RealSense 카메라 내부 파라미터 불러오는 함수
#         intrinsics = self.get_camera_intrinsics()

#         # 2D 좌표를 Normalized Coordinate로 변환 -> 찾아보면 공식있음
#         normalized_x = (x - intrinsics['cx']) / intrinsics['fx']
#         normalized_y = (y - intrinsics['cy']) / intrinsics['fy']

#         # Normalized Coordinate를 3D 좌표로 변환 -> 이것도 공식있음
#         point_x = normalized_x * depth_value
#         point_y = normalized_y * depth_value
#         point_z = float(depth_value)

#         return [point_x, point_y, point_z]

#     def get_camera_intrinsics(self):
#         # 실제로는 캘리브레이션을 통해 얻어진 값 사용 지금값은 널리 알려진 값. realsense device 정보 공식문서가서 찾아보거나 직접 캘리브레이션 해야하는데 귀찮
#         return {'fx': 600, 'fy': 600, 'cx': 320, 'cy': 240}

# def main(args=None):
#     rclpy.init(args=args)
#     node = ObjCoordinateNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

############################## 일단 지워지긴함. 근데 왤케 버벅이지
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from yolov8_msgs.msg import DetectionArray
from cv_bridge import CvBridge

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped

class ObjCoordinateNode(Node):
    def __init__(self):
        super().__init__('obj_coordinate_node')

        # RGB 이미지 및 Depth 이미지 구독 -> 일단 뭘 불러와야 처리를 하든 말든 하지
        self.subscription_image = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # 실제 사용 중인 RGB 카메라 이미지 토픽 이름으로 변경
            self.image_callback,
            10
        )
        self.subscription_depth = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',  # 실제 사용 중인 Depth 카메라 이미지 토픽 이름으로 변경
            self.depth_callback,
            10
        )

        # 물체 감지 결과 구독 -> 여기서 좌표 정보, 객체이름, ID 이런거 나옴
        # ros2 topic echo /yolo/detections 이랑 ros2 topic echo /yolo/tracking 찍어보면
        # 차이는 ID값 밖에 없음. 이게 없으면 물체가 여러개 있을 때 class_name만 받아오니까 나온 좌표가 어느 물체인지 한눈에 보기 어려움
        # 그래서 tracking 구독해서 그 안에 있는 ID 갔다가 썻음
        self.subscription_tracking = self.create_subscription(
            DetectionArray,
            '/yolo/tracking',
            self.detections_callback,
            10
        )

        self.bridge = CvBridge()

        # 이거 리스트로 받는 이유가 여러 물체 감지할 때 받은 자료 관리 더 쉽게 하려고
        self.detected_objects = []

        self.current_detected_ids = set()  # 추가: 현재 프레임에서 감지된 물체들의 ID를 저장하는 집합, 집합으로 받으면 중복되는 걸 쉽게 관리할 수 있음. 궁금하면 개인적으로 알아볼것

        self.previous_detected_ids = set()  # 추가: 이전에 인식한 물체 정보를 저장하는 집합

        # Marker Publisher 추가
        self.marker_publisher = self.create_publisher(Marker, '/object_marker', 10)

        # Point Publisher 추가
        self.point_publisher = self.create_publisher(PointStamped, '/object_3d_coordinates', 10) 

    def image_callback(self, msg):
        # RGB 이미지 처리 opencv로 쓸 수 있게 처리
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def detections_callback(self, msg):
        # 물체 감지 결과 섭스크라이버 한거 내가 쓸거에 맞게 처리
        self.detected_objects = [
            {
                'class_name': detection.class_name,
                'id': detection.id,
                'bbox': {
                    'center': {
                        'position': {'x': detection.bbox.center.position.x, 'y': detection.bbox.center.position.y}
                    }
                }
            } for detection in msg.detections
        ]
        
        # 현재 프레임에서 감지된 물체들의 ID를 업데이트
        self.current_detected_ids = {obj['id'] for obj in self.detected_objects}
        
        # 현재 프레임에서 감지된 물체들의 ID 중 이전에 인식한 물체들의 ID에 없는 것만 추가
        new_ids = self.current_detected_ids - self.previous_detected_ids
        self.previous_detected_ids.update(new_ids)

        self.none_detecte_objects_ids = self.previous_detected_ids - self.current_detected_ids

        # print(self.current_detected_ids)
        # print(self.previous_detected_ids)

        #화면에 나오는 물체 없으면 전체 삭제
        if self.current_detected_ids == set():
            for none_detected_marker_ids in self.none_detecte_objects_ids:
                # self.get_logger().info(f"Object is no longer detected.")
                delete_marker = Marker()
                delete_marker.header.frame_id = '/camera_link'  
                delete_marker.header.stamp = self.get_clock().now().to_msg()
                delete_marker.id = int(none_detected_marker_ids)
                delete_marker.action = Marker.DELETEALL

                delete_text_marker = Marker()
                delete_text_marker.header.frame_id = '/camera_link'
                delete_text_marker.header.stamp = self.get_clock().now().to_msg()
                delete_text_marker.id = delete_marker.id + 1024
                delete_text_marker.action = Marker.DELETEALL

                self.marker_publisher.publish(delete_marker)
                self.marker_publisher.publish(delete_text_marker)
                break

        elif self.none_detecte_objects_ids != set():
            for none_detected_marker_ids in self.none_detecte_objects_ids:
                # self.get_logger().info(f"Object with ID {none_detected_marker_ids} is no longer detected.")
                delete_marker = Marker()
                delete_marker.header.frame_id = '/camera_link'  
                delete_marker.header.stamp = self.get_clock().now().to_msg()
                delete_marker.id = int(none_detected_marker_ids)
                delete_marker.action = Marker.DELETEALL

                delete_text_marker = Marker()
                delete_text_marker.header.frame_id = '/camera_link' 
                delete_text_marker.header.stamp = self.get_clock().now().to_msg()
                delete_text_marker.id = delete_marker.id + 1024
                delete_text_marker.action = Marker.DELETEALL

                self.marker_publisher.publish(delete_marker)
                self.marker_publisher.publish(delete_text_marker)

            self.previous_detected_ids -= self.none_detecte_objects_ids    

    def depth_callback(self, msg):
        # Depth 이미지 opencv로 쓸 수 있게 처리
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        for obj in self.detected_objects:
            # 2D 좌표 -> 인식된 객체의 박스 중심 좌표
            x2d, y2d = int(obj['bbox']['center']['position']['x']), int(obj['bbox']['center']['position']['y'])

            # 2D 좌표를 3D 좌표로 변환 -> 박스 중심 좌표 가져와서 그 픽셀에서의 depth값을 가져오고 하는거
            depth_value = depth_image[y2d, x2d]  # 중심 좌표에서의 깊이 값
            point_3d = self.convert_depth_pixel_to_point(x2d, y2d, depth_value)

            # #물체 감지 결과 로그에 추가. 주석처리 해놓을테니 보고 싶으면 풀어보셈
            # self.get_logger().info("Object %s %s Center Point in 3D Point Cloud: %s" % (obj['class_name'], obj['id'], str(point_3d)))

            if point_3d != [0.0, 0.0, 0.0]:
                # 3D 좌표를 RViz2에 시각화하기 위해 Marker 메시지 생성
                marker = Marker()
                marker.header.frame_id = '/camera_link'  # Replace with your camera frame ID
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                #자료형이랑 단위 맞춰준거 mm -> m로 변환 *0.001이 그거임
                #원래는 TF써서 좌표축 돌려야 하는데 그냥 귀찮아서 일단 대강 맞춰놈
                marker.pose.position.x = float(point_3d[2])*0.001
                marker.pose.position.y = float(-(point_3d[0]))*0.001
                marker.pose.position.z = float(-(point_3d[1]))*0.001
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.1  # Adjust the size of the marker
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.a = 1.0  # Fully opaque
                marker.color.r = 1.0  # Red color
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.id = int(obj['id'])  # Use object ID as marker ID
                # 빨간색 점으로 퍼블리시되는거
                self.marker_publisher.publish(marker)

                # 3D 좌표를 RViz2에 시각화하기 위해 Marker 메시지 생성 -> 이름띄우는거
                text_marker = Marker()
                text_marker.header.frame_id = '/camera_link'  # Replace with your camera frame ID
                text_marker.header.stamp = self.get_clock().now().to_msg()
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = marker.pose.position.x
                text_marker.pose.position.y = marker.pose.position.y
                text_marker.pose.position.z = marker.pose.position.z
                text_marker.pose.orientation.w = 1.0
                text_marker.scale.x = 0.1
                text_marker.scale.y = 0.1
                text_marker.scale.z = 0.1
                text_marker.color.a = 1.0
                text_marker.color.r = 1.0
                text_marker.color.g = 0.0
                text_marker.color.b = 0.0
                text_marker.id = (marker.id + 1024)  # Use object ID as marker ID 1024는 그냥 임의. 위에 마커아이디랑 숫자만 안겹치면됨
                text_marker.text = f"{obj['class_name']}{obj['id']}"

                # Publish the Text Marker for visualization in RViz2
                self.marker_publisher.publish(text_marker)

                # Publish the 3D coordinates as PointStamped for additional visualization
                self.publish_3d_coordinates_as_point_stamped(point_3d)
    
    def publish_3d_coordinates_as_point_stamped(self, point_3d):
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = '/camera_link'  # Replace with your camera frame ID
        point_msg.point.x, point_msg.point.y, point_msg.point.z = point_3d

        # Publish the PointStamped message
        self.point_publisher.publish(point_msg)

        # #Log the 3D coordinates for debugging 이것도 그냥 프린트문
        # self.get_logger().info("3D Coordinates: %s" % str(point_3d))

    def convert_depth_pixel_to_point(self, x, y, depth_value):
        # RealSense 카메라 내부 파라미터 불러오는 함수
        intrinsics = self.get_camera_intrinsics()

        # 2D 좌표를 Normalized Coordinate로 변환 -> 찾아보면 공식있음
        normalized_x = (x - intrinsics['cx']) / intrinsics['fx']
        normalized_y = (y - intrinsics['cy']) / intrinsics['fy']

        # Normalized Coordinate를 3D 좌표로 변환 -> 이것도 공식있음
        point_x = normalized_x * depth_value
        point_y = normalized_y * depth_value
        point_z = float(depth_value)

        return [point_x, point_y, point_z]

    def get_camera_intrinsics(self):
        # 실제로는 캘리브레이션을 통해 얻어진 값 사용 지금값은 널리 알려진 값. realsense device 정보 공식문서가서 찾아보거나 직접 캘리브레이션 해야하는데 귀찮
        return {'fx': 600, 'fy': 600, 'cx': 320, 'cy': 240}

def main(args=None):
    rclpy.init(args=args)
    node = ObjCoordinateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
#####################3 되는거 같기도 하고
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, PointCloud2
# from yolov8_msgs.msg import DetectionArray
# from cv_bridge import CvBridge
# import numpy as np

# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import PointStamped
# import time

# class ObjCoordinateNode(Node):
#     def __init__(self):
#         super().__init__('obj_coordinate_node')

#         # RGB 이미지 및 Depth 이미지 구독 -> 일단 뭘 불러와야 처리를 하든 말든 하지
#         self.subscription_image = self.create_subscription(
#             Image,
#             '/camera/color/image_raw',  # 실제 사용 중인 RGB 카메라 이미지 토픽 이름으로 변경
#             self.image_callback,
#             10
#         )
#         self.subscription_depth = self.create_subscription(
#             Image,
#             '/camera/depth/image_rect_raw',  # 실제 사용 중인 Depth 카메라 이미지 토픽 이름으로 변경
#             self.depth_callback,
#             10
#         )

#         # 물체 감지 결과 구독 -> 여기서 좌표 정보, 객체이름, ID 이런거 나옴
#         # ros2 topic echo /yolo/detections 이랑 ros2 topic echo /yolo/tracking 찍어보면
#         # 차이는 ID값 밖에 없음. 이게 없으면 물체가 여러개 있을 때 class_name만 받아오니까 나온 좌표가 어느 물체인지 한눈에 보기 어려움
#         # 그래서 tracking 구독해서 그 안에 있는 ID 갔다가 썻음
#         self.subscription_tracking = self.create_subscription(
#             DetectionArray,
#             '/yolo/tracking',
#             self.detections_callback,
#             10
#         )

#         self.bridge = CvBridge()
#         # 이거 리스트로 받는 이유가 여러 물체 감지할 때 받은 자료 관리 더 쉽게 하려고
#         self.detected_objects = []

#         self.current_detected_ids = set()  # 추가: 현재 프레임에서 감지된 물체들의 ID를 저장하는 변수

#         self.previous_detected_ids = set()  # 추가: 이전에 인식한 물체 정보를 저장하는 리스트

#         # Marker Publisher 추가
#         self.marker_publisher = self.create_publisher(Marker, '/object_marker', 10)

#         # Point Publisher 추가
#         self.point_publisher = self.create_publisher(PointStamped, '/object_3d_coordinates', 10) 

#     def image_callback(self, msg):
#         # RGB 이미지 처리 opencv로 쓸 수 있게 처리
#         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#     def detections_callback(self, msg):
#         # 물체 감지 결과 섭스크라이버 한거 내가 쓸거에 맞게 처리
#         self.detected_objects = [
#             {
#                 'class_name': detection.class_name,
#                 'id': detection.id,
#                 'bbox': {
#                     'center': {
#                         'position': {'x': detection.bbox.center.position.x, 'y': detection.bbox.center.position.y}
#                     }
#                 }
#             } for detection in msg.detections
#         ]
        
#         # 현재 프레임에서 감지된 물체들의 ID를 업데이트
#         self.current_detected_ids = {obj['id'] for obj in self.detected_objects}
        
#         # 현재 프레임에서 감지된 물체들의 ID 중 이전에 인식한 물체들의 ID에 없는 것만 추가
#         new_ids = self.current_detected_ids - self.previous_detected_ids
#         self.previous_detected_ids.update(new_ids)

#         # print(self.current_detected_ids)
#         # print(self.previous_detected_ids)

#     def depth_callback(self, msg):
#         # Depth 이미지 opencv로 쓸 수 있게 처리
#         depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

#         for obj in self.detected_objects:
#             # 2D 좌표 -> 인식된 객체의 박스 중심 좌표
#             x2d, y2d = int(obj['bbox']['center']['position']['x']), int(obj['bbox']['center']['position']['y'])

#             # 2D 좌표를 3D 좌표로 변환 -> 박스 중심 좌표 가져와서 그 픽셀에서의 depth값을 가져오고 하는거
#             depth_value = depth_image[y2d, x2d]  # 중심 좌표에서의 깊이 값
#             point_3d = self.convert_depth_pixel_to_point(x2d, y2d, depth_value)

#             # #물체 감지 결과 로그에 추가. 주석처리 해놓을테니 보고 싶으면 풀어보셈
#             # self.get_logger().info("Object %s %s Center Point in 3D Point Cloud: %s" % (obj['class_name'], obj['id'], str(point_3d)))

#             if point_3d != [0.0, 0.0, 0.0] and self.current_detected_ids is not None:
#                 # 3D 좌표를 RViz2에 시각화하기 위해 Marker 메시지 생성
#                 marker = Marker()
#                 marker.header.frame_id = '/camera_link'  # Replace with your camera frame ID
#                 marker.header.stamp = self.get_clock().now().to_msg()
#                 marker.type = Marker.SPHERE
#                 marker.action = Marker.ADD
#                 #자료형이랑 단위 맞춰준거 mm -> m로 변환 *0.001이 그거임
#                 #원래는 TF써서 좌표축 돌려야 하는데 그냥 귀찮아서 일단 대강 맞춰놈
#                 marker.pose.position.x = float(point_3d[2])*0.001
#                 marker.pose.position.y = float(-(point_3d[0]))*0.001
#                 marker.pose.position.z = float(-(point_3d[1]))*0.001
#                 marker.pose.orientation.w = 1.0
#                 marker.scale.x = 0.1  # Adjust the size of the marker
#                 marker.scale.y = 0.1
#                 marker.scale.z = 0.1
#                 marker.color.a = 1.0  # Fully opaque
#                 marker.color.r = 1.0  # Red color
#                 marker.color.g = 0.0
#                 marker.color.b = 0.0
#                 marker.id = int(obj['id'])  # Use object ID as marker ID
#                 # 빨간색 점으로 퍼블리시되는거
#                 self.marker_publisher.publish(marker)

#                 # 3D 좌표를 RViz2에 시각화하기 위해 Marker 메시지 생성 -> 이름띄우는거
#                 text_marker = Marker()
#                 text_marker.header.frame_id = '/camera_link'  # Replace with your camera frame ID
#                 text_marker.header.stamp = self.get_clock().now().to_msg()
#                 text_marker.type = Marker.TEXT_VIEW_FACING
#                 text_marker.action = Marker.ADD
#                 text_marker.pose.position.x = float(point_3d[2]) * 0.001
#                 text_marker.pose.position.y = float(-(point_3d[0])) * 0.001
#                 text_marker.pose.position.z = float(-(point_3d[1])) * 0.001
#                 text_marker.pose.orientation.w = 1.0
#                 text_marker.scale.x = 0.1
#                 text_marker.scale.y = 0.1
#                 text_marker.scale.z = 0.1
#                 text_marker.color.a = 1.0
#                 text_marker.color.r = 1.0
#                 text_marker.color.g = 0.0
#                 text_marker.color.b = 0.0
#                 text_marker.id = (marker.id + 1024)  # Use object ID as marker ID 1024는 그냥 임의. 위에 마커아이디랑 숫자만 안곂치면됨
#                 text_marker.text = f"{obj['class_name']}{obj['id']}"

#                 # Publish the Text Marker for visualization in RViz2
#                 self.marker_publisher.publish(text_marker)

#                 # Publish the 3D coordinates as PointStamped for additional visualization
#                 self.publish_3d_coordinates_as_point_stamped(point_3d)

#                 self.none_detecte_objects_ids = self.previous_detected_ids - self.current_detected_ids
#                 # print(self.current_detected_ids)

#                 for none_detected_marker_ids in self.none_detecte_objects_ids:
#                     # self.get_logger().info(f"Object with ID {none_detected_marker_ids} is no longer detected.")
#                     delete_marker = Marker()
#                     delete_marker.header.frame_id = '/camera_link'
#                     delete_marker.header.stamp = self.get_clock().now().to_msg()
#                     delete_marker.id = int(none_detected_marker_ids)
#                     delete_marker.action = Marker.DELETE

#                     delete_text_marker = Marker()
#                     delete_text_marker.header.frame_id = '/camera_link'
#                     delete_text_marker.header.stamp = self.get_clock().now().to_msg()
#                     delete_text_marker.id = delete_marker.id + 1024
#                     delete_text_marker.action = Marker.DELETE

#                     self.marker_publisher.publish(delete_marker)
#                     self.marker_publisher.publish(delete_text_marker)
    
#     def publish_3d_coordinates_as_point_stamped(self, point_3d):
#         point_msg = PointStamped()
#         point_msg.header.stamp = self.get_clock().now().to_msg()
#         point_msg.header.frame_id = '/camera_link'  # Replace with your camera frame ID
#         point_msg.point.x, point_msg.point.y, point_msg.point.z = point_3d

#         # Publish the PointStamped message
#         self.point_publisher.publish(point_msg)

#         # #Log the 3D coordinates for debugging 이것도 그냥 프린트문
#         # self.get_logger().info("3D Coordinates: %s" % str(point_3d))

#     def convert_depth_pixel_to_point(self, x, y, depth_value):
#         # RealSense 카메라 내부 파라미터 불러오는 함수
#         intrinsics = self.get_camera_intrinsics()

#         # 2D 좌표를 Normalized Coordinate로 변환 -> 찾아보면 공식있음
#         normalized_x = (x - intrinsics['cx']) / intrinsics['fx']
#         normalized_y = (y - intrinsics['cy']) / intrinsics['fy']

#         # Normalized Coordinate를 3D 좌표로 변환 -> 이것도 공식있음
#         point_x = normalized_x * depth_value
#         point_y = normalized_y * depth_value
#         point_z = float(depth_value)

#         return [point_x, point_y, point_z]

#     def get_camera_intrinsics(self):
#         # 실제로는 캘리브레이션을 통해 얻어진 값 사용 지금값은 널리 알려진 값. realsense device 정보 공식문서가서 찾아보거나 직접 캘리브레이션 해야하는데 귀찮
#         return {'fx': 600, 'fy': 600, 'cx': 320, 'cy': 240}

# def main(args=None):
#     rclpy.init(args=args)
#     node = ObjCoordinateNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

##################
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from yolov8_msgs.msg import DetectionArray
# from cv_bridge import CvBridge

# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import PointStamped

# class ObjCoordinateNode(Node):
#     def __init__(self):
#         super().__init__('obj_coordinate_node')

#         # RGB 이미지 및 Depth 이미지 구독 -> 일단 뭘 불러와야 처리를 하든 말든 하지
#         self.subscription_image = self.create_subscription(
#             Image,
#             '/camera/color/image_raw',  # 실제 사용 중인 RGB 카메라 이미지 토픽 이름으로 변경
#             self.image_callback,
#             10
#         )
#         self.subscription_depth = self.create_subscription(
#             Image,
#             '/camera/depth/image_rect_raw',  # 실제 사용 중인 Depth 카메라 이미지 토픽 이름으로 변경
#             self.depth_callback,
#             10
#         )

#         # 물체 감지 결과 구독 -> 여기서 좌표 정보, 객체이름, ID 이런거 나옴
#         # ros2 topic echo /yolo/detections 이랑 ros2 topic echo /yolo/tracking 찍어보면
#         # 차이는 ID값 밖에 없음. 이게 없으면 물체가 여러개 있을 때 class_name만 받아오니까 나온 좌표가 어느 물체인지 한눈에 보기 어려움
#         # 그래서 tracking 구독해서 그 안에 있는 ID 갔다가 썻음
#         self.subscription_tracking = self.create_subscription(
#             DetectionArray,
#             '/yolo/tracking',
#             self.detections_callback,
#             10
#         )

#         self.bridge = CvBridge()

#         # 이거 리스트로 받는 이유가 여러 물체 감지할 때 받은 자료 관리 더 쉽게 하려고
#         self.detected_objects = []

#         self.current_detected_ids = set()  # 추가: 현재 프레임에서 감지된 물체들의 ID를 저장하는 집합, 집합으로 받으면 중복되는 걸 쉽게 관리할 수 있음. 궁금하면 개인적으로 알아볼것

#         self.previous_detected_ids = set()  # 추가: 이전에 인식한 물체 정보를 저장하는 집합

#         # Marker Publisher 추가
#         self.marker_publisher = self.create_publisher(Marker, '/object_marker', 10)

#         # Point Publisher 추가
#         self.point_publisher = self.create_publisher(PointStamped, '/object_3d_coordinates', 10)

#     def image_callback(self, msg):
#         # RGB 이미지 처리 opencv로 쓸 수 있게 처리
#         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#     def depth_callback(self, msg):
#         # Depth 이미지 opencv로 쓸 수 있게 처리
#         self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         self.marker_create(self.detected_objects)

#     def detections_callback(self, msg):
#         # 물체 감지 결과 섭스크라이버 한거 내가 쓸거에 맞게 처리
#         self.detected_objects = [
#             {
#                 'class_name': detection.class_name,
#                 'id': detection.id,
#                 'bbox': {
#                     'center': {
#                         'position': {'x': detection.bbox.center.position.x, 'y': detection.bbox.center.position.y}
#                     }
#                 }
#             } for detection in msg.detections
#         ]
        
#         # 현재 프레임에서 감지된 물체들의 ID를 업데이트
#         self.current_detected_ids = {obj['id'] for obj in self.detected_objects}
        
#         # 현재 프레임에서 감지된 물체들의 ID 중 이전에 인식한 물체들의 ID에 없는 것만 추가
#         new_ids = self.current_detected_ids - self.previous_detected_ids
#         self.previous_detected_ids.update(new_ids)

#         self.none_detecte_objects_ids = self.previous_detected_ids - self.current_detected_ids

#         # print(self.current_detected_ids)
#         # print(self.previous_detected_ids)

#         # self.marker_create(self.detected_objects)

#     def depth_callback(self, msg):
#         # Depth 이미지 opencv로 쓸 수 있게 처리
#         self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         self.marker_create(self.detected_objects)
    
#     def publish_3d_coordinates_as_point_stamped(self, point_3d):
#         point_msg = PointStamped()
#         point_msg.header.stamp = self.get_clock().now().to_msg()
#         point_msg.header.frame_id = '/camera_link'  # Replace with your camera frame ID
#         point_msg.point.x, point_msg.point.y, point_msg.point.z = point_3d

#         # Publish the PointStamped message
#         self.point_publisher.publish(point_msg)

#         # #Log the 3D coordinates for debugging 이것도 그냥 프린트문
#         # self.get_logger().info("3D Coordinates: %s" % str(point_3d))

#     def convert_depth_pixel_to_point(self, x, y, depth_value):
#         # RealSense 카메라 내부 파라미터 불러오는 함수
#         intrinsics = self.get_camera_intrinsics()

#         # 2D 좌표를 Normalized Coordinate로 변환 -> 찾아보면 공식있음
#         normalized_x = (x - intrinsics['cx']) / intrinsics['fx']
#         normalized_y = (y - intrinsics['cy']) / intrinsics['fy']

#         # Normalized Coordinate를 3D 좌표로 변환 -> 이것도 공식있음
#         point_x = normalized_x * depth_value
#         point_y = normalized_y * depth_value
#         point_z = float(depth_value)

#         return [point_x, point_y, point_z]

#     def get_camera_intrinsics(self):
#         # 실제로는 캘리브레이션을 통해 얻어진 값 사용 지금값은 널리 알려진 값. realsense device 정보 공식문서가서 찾아보거나 직접 캘리브레이션 해야하는데 귀찮
#         return {'fx': 600, 'fy': 600, 'cx': 320, 'cy': 240}
    
#     def marker_create(self, detected_objects):
#         self.none_detecte_objects_ids = self.previous_detected_ids - self.current_detected_ids
#         for obj in detected_objects:
#             x2d, y2d = int(obj['bbox']['center']['position']['x']), int(obj['bbox']['center']['position']['y'])
#             depth_value = self.depth_image[y2d, x2d]
#             point_3d = self.convert_depth_pixel_to_point(x2d, y2d, depth_value)

#             if point_3d != [0.0, 0.0, 0.0]:
#                 marker = Marker()
#                 marker.header.frame_id = '/camera_link'
#                 marker.header.stamp = self.get_clock().now().to_msg()
#                 marker.type = Marker.SPHERE
#                 marker.action = Marker.ADD
#                 marker.pose.position.x = float(point_3d[2]) * 0.001
#                 marker.pose.position.y = float(-(point_3d[0])) * 0.001
#                 marker.pose.position.z = float(-(point_3d[1])) * 0.001
#                 marker.pose.orientation.w = 1.0
#                 marker.scale.x = 0.1
#                 marker.scale.y = 0.1
#                 marker.scale.z = 0.1
#                 marker.color.a = 1.0
#                 marker.color.r = 1.0
#                 marker.color.g = 0.0
#                 marker.color.b = 0.0
#                 marker.id = int(obj['id'])
#                 self.marker_publisher.publish(marker)

#                 text_marker = Marker()
#                 text_marker.header.frame_id = '/camera_link'
#                 text_marker.header.stamp = self.get_clock().now().to_msg()
#                 text_marker.type = Marker.TEXT_VIEW_FACING
#                 text_marker.action = Marker.ADD
#                 text_marker.pose.position.x = marker.pose.position.x
#                 text_marker.pose.position.y = marker.pose.position.y
#                 text_marker.pose.position.z = marker.pose.position.z
#                 text_marker.pose.orientation.w = 1.0
#                 text_marker.scale.x = 0.1
#                 text_marker.scale.y = 0.1
#                 text_marker.scale.z = 0.1
#                 text_marker.color.a = 1.0
#                 text_marker.color.r = 1.0
#                 text_marker.color.g = 0.0
#                 text_marker.color.b = 0.0
#                 text_marker.id = (marker.id + 1024)
#                 text_marker.text = f"{obj['class_name']}{obj['id']}"
#                 self.marker_publisher.publish(text_marker)

#                 self.publish_3d_coordinates_as_point_stamped(point_3d)

#                 if self.current_detected_ids == set():
#                     for none_detected_marker_ids in self.none_detecte_objects_ids:
#                         # self.get_logger().info(f"Object is no longer detected.")
#                         delete_marker = Marker()
#                         delete_marker.header.frame_id = '/camera_link'  
#                         delete_marker.header.stamp = self.get_clock().now().to_msg()
#                         delete_marker.id = int(none_detected_marker_ids)
#                         delete_marker.action = Marker.DELETEALL

#                         delete_text_marker = Marker()
#                         delete_text_marker.header.frame_id = '/camera_link'
#                         delete_text_marker.header.stamp = self.get_clock().now().to_msg()
#                         delete_text_marker.id = delete_marker.id + 1024
#                         delete_text_marker.action = Marker.DELETEALL

#                         self.marker_publisher.publish(delete_marker)
#                         self.marker_publisher.publish(delete_text_marker)
#                         break

#                 elif self.none_detecte_objects_ids != set():
#                     for none_detected_marker_ids in self.none_detecte_objects_ids:
#                         # self.get_logger().info(f"Object with ID {none_detected_marker_ids} is no longer detected.")
#                         delete_marker = Marker()
#                         delete_marker.header.frame_id = '/camera_link'  
#                         delete_marker.header.stamp = self.get_clock().now().to_msg()
#                         delete_marker.id = int(none_detected_marker_ids)
#                         delete_marker.action = Marker.DELETE

#                         delete_text_marker = Marker()
#                         delete_text_marker.header.frame_id = '/camera_link'
#                         delete_text_marker.header.stamp = self.get_clock().now().to_msg()
#                         delete_text_marker.id = delete_marker.id + 1024
#                         delete_text_marker.action = Marker.DELETE

#                         self.marker_publisher.publish(delete_marker)
#                         self.marker_publisher.publish(delete_text_marker)

#                     self.previous_detected_ids = self.current_detected_ids  #???? 업데이트 되는거 맞아?    

# def main(args=None):
#     rclpy.init(args=args)
#     node = ObjCoordinateNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
