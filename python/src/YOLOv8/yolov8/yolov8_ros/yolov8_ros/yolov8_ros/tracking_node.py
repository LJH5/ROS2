# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


# import numpy as np

# import rclpy
# from rclpy.qos import qos_profile_sensor_data
# from rclpy.node import Node

# import message_filters
# from cv_bridge import CvBridge

# from ultralytics.trackers import BOTSORT, BYTETracker
# from ultralytics.trackers.basetrack import BaseTrack
# from ultralytics.utils import IterableSimpleNamespace, yaml_load
# from ultralytics.utils.checks import check_requirements, check_yaml
# from ultralytics.engine.results import Boxes

# from sensor_msgs.msg import Image
# from yolov8_msgs.msg import Detection
# from yolov8_msgs.msg import DetectionArray


# class TrackingNode(Node):

#     def __init__(self) -> None:
#         super().__init__("tracking_node")

#         # params
#         self.declare_parameter("tracker", "bytetrack.yaml")
#         tracker = self.get_parameter(
#             "tracker").get_parameter_value().string_value

#         self.cv_bridge = CvBridge()
#         self.tracker = self.create_tracker(tracker)

#         # pubs
#         self._pub = self.create_publisher(DetectionArray, "tracking", 10)

#         # subs
#         image_sub = message_filters.Subscriber(
#             self, Image, "/camera/color/image_raw", qos_profile=qos_profile_sensor_data)
#         detections_sub = message_filters.Subscriber(
#             self, DetectionArray, "detections", qos_profile=10)

#         self._synchronizer = message_filters.ApproximateTimeSynchronizer(
#             (image_sub, detections_sub), 10, 0.5)
#         self._synchronizer.registerCallback(self.detections_cb)

#     def create_tracker(self, tracker_yaml: str) -> BaseTrack:

#         TRACKER_MAP = {"bytetrack": BYTETracker, "botsort": BOTSORT}
#         check_requirements("lap")  # for linear_assignment

#         tracker = check_yaml(tracker_yaml)
#         cfg = IterableSimpleNamespace(**yaml_load(tracker))

#         assert cfg.tracker_type in ["bytetrack", "botsort"], \
#             f"Only support 'bytetrack' and 'botsort' for now, but got '{cfg.tracker_type}'"
#         tracker = TRACKER_MAP[cfg.tracker_type](args=cfg, frame_rate=1)
#         return tracker

#     def detections_cb(self, img_msg: Image, detections_msg: DetectionArray) -> None:

#         tracked_detections_msg = DetectionArray()
#         tracked_detections_msg.header = img_msg.header

#         # convert image
#         cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg)

#         # parse detections
#         detection_list = []
#         detection: Detection
#         for detection in detections_msg.detections:

#             detection_list.append(
#                 [
#                     detection.bbox.center.position.x - detection.bbox.size.x / 2,
#                     detection.bbox.center.position.y - detection.bbox.size.y / 2,
#                     detection.bbox.center.position.x + detection.bbox.size.x / 2,
#                     detection.bbox.center.position.y + detection.bbox.size.y / 2,
#                     detection.score,
#                     detection.class_id
#                 ]
#             )

#         # tracking
#         if len(detection_list) > 0:

#             det = Boxes(
#                 np.array(detection_list),
#                 (img_msg.height, img_msg.width)
#             )

#             tracks = self.tracker.update(det, cv_image)

#             if len(tracks) > 0:

#                 for t in tracks:

#                     tracked_box = Boxes(
#                         t[:-1], (img_msg.height, img_msg.width))

#                     tracked_detection: Detection = detections_msg.detections[int(
#                         t[-1])]

#                     # get boxes values
#                     box = tracked_box.xywh[0]
#                     tracked_detection.bbox.center.position.x = float(box[0])
#                     tracked_detection.bbox.center.position.y = float(box[1])
#                     tracked_detection.bbox.size.x = float(box[2])
#                     tracked_detection.bbox.size.y = float(box[3])

#                     # get track id
#                     track_id = ""
#                     if tracked_box.is_track:
#                         track_id = str(int(tracked_box.id))
#                     tracked_detection.id = track_id

#                     # append msg
#                     tracked_detections_msg.detections.append(tracked_detection)

#         # publish detections
#         self._pub.publish(tracked_detections_msg)


# def main():
#     rclpy.init()
#     node = TrackingNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()



##########################
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

import message_filters
from cv_bridge import CvBridge

from ultralytics.trackers import BOTSORT, BYTETracker
from ultralytics.trackers.basetrack import BaseTrack
from ultralytics.utils import IterableSimpleNamespace, yaml_load
from ultralytics.utils.checks import check_requirements, check_yaml
from ultralytics.engine.results import Boxes

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from yolov8_msgs.msg import Detection
from yolov8_msgs.msg import DetectionArray


from tf2_msgs.msg import TFMessage
from tf2_ros import TransformStamped
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs
import tf2_ros



class TrackingNode(Node):

    def __init__(self) -> None:
        super().__init__("tracking_node")

        # params
        self.declare_parameter("tracker", "bytetrack.yaml")
        tracker = self.get_parameter(
            "tracker").get_parameter_value().string_value

        self.declare_parameter("image_reliability",
                               QoSReliabilityPolicy.BEST_EFFORT)
        image_qos_profile = QoSProfile(
            reliability=self.get_parameter(
                "image_reliability").get_parameter_value().integer_value,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.cv_bridge = CvBridge()
        self.tracker = self.create_tracker(tracker)

        self.tf_broadcaster = TransformBroadcaster(self)

        # pubs
        self._pub = self.create_publisher(DetectionArray, "tracking", 10)

        # subs
        image_sub = message_filters.Subscriber(
            self, Image, "/camera/color/image_raw", qos_profile= image_qos_profile)
        detections_sub = message_filters.Subscriber(
            self, DetectionArray, "detections", qos_profile=10)

        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            (image_sub, detections_sub), 50, 0.5)
        self._synchronizer.registerCallback(self.detections_cb)

        # Dictionary to keep track of active TF frames
        self.active_tf_frames = {}

    def create_tracker(self, tracker_yaml: str) -> BaseTrack:

        TRACKER_MAP = {"bytetrack": BYTETracker, "botsort": BOTSORT}
        check_requirements("lap")  # for linear_assignment

        tracker = check_yaml(tracker_yaml)
        cfg = IterableSimpleNamespace(**yaml_load(tracker))

        assert cfg.tracker_type in ["bytetrack", "botsort"], \
            f"Only support 'bytetrack' and 'botsort' for now, but got '{cfg.tracker_type}'"
        tracker = TRACKER_MAP[cfg.tracker_type](args=cfg, frame_rate=1)
        return tracker

    def publish_tf(self, track_id: str, tracked_box: Boxes, header: Header) -> None:
        tf_msg = TFMessage()
        tf_stamped = TransformStamped()
        tf_stamped.header = header
        tf_stamped.child_frame_id = f'tracked_object_{track_id}'

        # Set TF frame translation
        tf_stamped.transform.translation.x = float(tracked_box.xywh[0][0])
        tf_stamped.transform.translation.y = float(tracked_box.xywh[0][1])
        tf_stamped.transform.translation.z = 0.0  # 높이 정보가 없으므로 0으로 설정

        # Set TF frame rotation
        tf_stamped.transform.rotation.w = 1.0  # 단위 사원수 (Quaternion)의 기본 값

        tf_msg.transforms.append(tf_stamped)
        self.tf_broadcaster.sendTransform(tf_msg.transforms)

        # Keep track of active TF frames
        self.active_tf_frames[track_id] = f'tracked_object_{track_id}'

    def remove_inactive_tf_frames(self, active_tracks, header):
        # Remove TF frames for inactive tracks
        for track_id, tf_frame_id in list(self.active_tf_frames.items()):
            if track_id not in active_tracks:
                transform = TransformStamped()
                transform.header.stamp = header.stamp
                transform.header.frame_id = "map"
                transform.child_frame_id = tf_frame_id
                transform.transform.translation.x = 0.0
                transform.transform.translation.y = 0.0
                transform.transform.translation.z = 0.0
                transform.transform.rotation.w = 1.0  # 단위 사원수 (Quaternion)의 기본 값
                self.tf_broadcaster.sendTransform([transform])

    def detections_cb(self, img_msg: Image, detections_msg: DetectionArray) -> None:

        tracked_detections_msg = DetectionArray()
        tracked_detections_msg.header = img_msg.header

        # convert image
        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg)

        # parse detections
        detection_list = []
        detection: Detection
        for detection in detections_msg.detections:

            detection_list.append(
                [
                    detection.bbox.center.position.x - detection.bbox.size.x / 2,
                    detection.bbox.center.position.y - detection.bbox.size.y / 2,
                    detection.bbox.center.position.x + detection.bbox.size.x / 2,
                    detection.bbox.center.position.y + detection.bbox.size.y / 2,
                    detection.score,
                    detection.class_id
                ]
            )

        # tracking
        if len(detection_list) > 0:

            det = Boxes(
                np.array(detection_list),
                (img_msg.height, img_msg.width)
            )

            tracks = self.tracker.update(det, cv_image)


            # Get active track IDs
            active_tracks = [int(t[-1]) for t in tracks]

            # Remove TF frames for inactive tracks
            self.remove_inactive_tf_frames(active_tracks, img_msg.header)

            if len(tracks) > 0:

                for t in tracks:

                    tracked_box = Boxes(
                        t[:-1], (img_msg.height, img_msg.width))

                    tracked_detection: Detection = detections_msg.detections[int(
                        t[-1])]

                    # get boxes values
                    box = tracked_box.xywh[0]
                    tracked_detection.bbox.center.position.x = float(box[0])
                    tracked_detection.bbox.center.position.y = float(box[1])
                    tracked_detection.bbox.size.x = float(box[2])
                    tracked_detection.bbox.size.y = float(box[3])

                    # get track id
                    track_id = ""
                    if tracked_box.is_track:
                        track_id = str(int(tracked_box.id))
                    tracked_detection.id = track_id

                    # append msg
                    tracked_detections_msg.detections.append(tracked_detection)

                    # Publish TF
                    self.publish_tf(tracked_detection.id, tracked_box, detections_msg.header)

        # publish detections
        self._pub.publish(tracked_detections_msg)


def main():
    rclpy.init()
    node = TrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()