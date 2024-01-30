import cv2
import torch
import random

from typing import List, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from cv_bridge import CvBridge

from ultralytics import YOLO
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
from ultralytics.engine.results import Masks
from ultralytics.engine.results import Keypoints

from sensor_msgs.msg import Image
from yolov8_msgs.msg import Point2D
from yolov8_msgs.msg import BoundingBox2D
from yolov8_msgs.msg import Mask
from yolov8_msgs.msg import KeyPoint2D
from yolov8_msgs.msg import KeyPoint2DArray
from yolov8_msgs.msg import Detection
from yolov8_msgs.msg import DetectionArray
from std_srvs.srv import SetBool


class Yolov8Node(Node):

    def __init__(self) -> None:
        super().__init__("yolov8_node")

        # params
        self.declare_parameter("model", "yolov8m.pt")
        model = self.get_parameter(
            "model").get_parameter_value().string_value

        self.declare_parameter("device", "cuda:0")
        self.device = self.get_parameter(
            "device").get_parameter_value().string_value

        self.declare_parameter("threshold", 0.5)
        self.threshold = self.get_parameter(
            "threshold").get_parameter_value().double_value

        self.declare_parameter("enable", True)
        self.enable = self.get_parameter(
            "enable").get_parameter_value().bool_value

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
        self.yolo = YOLO(model)
        self.yolo.fuse()

        # pubs
        self._pub = self.create_publisher(DetectionArray, "detections", 10)
        self._dbg_pub = self.create_publisher(Image, "dbg_image", 10)
        # subs
        self._sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.image_cb,
            image_qos_profile
        )

        # services
        self._srv = self.create_service(SetBool, "enable", self.enable_cb)

    def enable_cb(
        self,
        req: SetBool.Request,
        res: SetBool.Response
    ) -> SetBool.Response:
        self.enable = req.data
        res.success = True
        return res

    def parse_hypothesis(self, results: Results) -> List[Dict]:

        hypothesis_list = []

        box_data: Boxes
        for box_data in results.boxes:
            hypothesis = {
                "class_id": int(box_data.cls),
                "class_name": self.yolo.names[int(box_data.cls)],
                "score": float(box_data.conf)
            }
            hypothesis_list.append(hypothesis)

        return hypothesis_list

    def parse_boxes(self, results: Results) -> List[BoundingBox2D]:

        boxes_list = []

        box_data: Boxes
        for box_data in results.boxes:

            msg = BoundingBox2D()

            # get boxes values
            box = box_data.xywh[0]
            msg.center.position.x = float(box[0])
            msg.center.position.y = float(box[1])
            msg.size.x = float(box[2])
            msg.size.y = float(box[3])

            # append msg
            boxes_list.append(msg)

        return boxes_list

    def parse_masks(self, results: Results) -> List[Mask]:

        masks_list = []

        def create_point2d(x: float, y: float) -> Point2D:
            p = Point2D()
            p.x = x
            p.y = y
            return p

        mask: Masks
        for mask in results.masks:

            msg = Mask()

            msg.data = [create_point2d(float(ele[0]), float(ele[1]))
                        for ele in mask.xy[0].tolist()]
            msg.height = results.orig_img.shape[0]
            msg.width = results.orig_img.shape[1]

            masks_list.append(msg)

        return masks_list

    def parse_keypoints(self, results: Results) -> List[KeyPoint2DArray]:

        keypoints_list = []

        points: Keypoints
        for points in results.keypoints:

            msg_array = KeyPoint2DArray()

            if points.conf is None:
                continue

            for kp_id, (p, conf) in enumerate(zip(points.xy[0], points.conf[0])):

                if conf >= self.threshold:
                    msg = KeyPoint2D()

                    msg.id = kp_id + 1
                    msg.point.x = float(p[0])
                    msg.point.y = float(p[1])
                    msg.score = float(conf)

                    msg_array.data.append(msg)

            keypoints_list.append(msg_array)

        return keypoints_list

    def image_cb(self, msg: Image) -> None:

        if self.enable:

            # convert image + predict
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
            results = self.yolo.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                conf=self.threshold,
                device=self.device
            )
            results: Results = results[0].cpu()

            if results.boxes:
                hypothesis = self.parse_hypothesis(results)
                boxes = self.parse_boxes(results)

            if results.masks:
                masks = self.parse_masks(results)

            if results.keypoints:
                keypoints = self.parse_keypoints(results)

            # create detection msgs
            detections_msg = DetectionArray()

            for i in range(len(results)):

                aux_msg = Detection()

                if results.boxes:
                    aux_msg.class_id = hypothesis[i]["class_id"]
                    aux_msg.class_name = hypothesis[i]["class_name"]
                    aux_msg.score = hypothesis[i]["score"]

                    aux_msg.bbox = boxes[i]

                if results.masks:
                    aux_msg.mask = masks[i]

                if results.keypoints:
                    aux_msg.keypoints = keypoints[i]

                detections_msg.detections.append(aux_msg)

            # publish detections
            detections_msg.header = msg.header
            self._pub.publish(detections_msg)

def main():
    rclpy.init()
    node = Yolov8Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    

#################################


# import cv2
# import torch
# import random

# import rclpy
# from rclpy.qos import qos_profile_sensor_data
# from rclpy.node import Node
# from cv_bridge import CvBridge

# from ultralytics import YOLO
# from ultralytics.tracker import BOTSORT, BYTETracker
# from ultralytics.tracker.trackers.basetrack import BaseTrack
# from ultralytics.yolo.utils import IterableSimpleNamespace, yaml_load
# from ultralytics.yolo.utils.checks import check_requirements, check_yaml

# from sensor_msgs.msg import Image
# from vision_msgs.msg import Detection2D
# from vision_msgs.msg import ObjectHypothesisWithPose
# from vision_msgs.msg import Detection2DArray
# from std_srvs.srv import SetBool


# class Yolov8Node(Node):

#     def __init__(self) -> None:
#         super().__init__("yolov8_node")

#         # params
#         self.declare_parameter("model", "yolov8m.pt")
#         model = self.get_parameter(
#             "model").get_parameter_value().string_value

#         self.declare_parameter("tracker", "bytetrack.yaml")
#         tracker = self.get_parameter(
#             "tracker").get_parameter_value().string_value

#         self.declare_parameter("device", "cpu") # "cuda:0"
#         device = self.get_parameter(
#             "device").get_parameter_value().string_value

#         self.declare_parameter("threshold", 0.6)
#         self.threshold = self.get_parameter(
#             "threshold").get_parameter_value().double_value

#         self.declare_parameter("enable", True)
#         self.enable = self.get_parameter(
#             "enable").get_parameter_value().bool_value

#         self._class_to_color = {}
#         self.cv_bridge = CvBridge()
#         self.tracker = self.create_tracker(tracker)
#         self.yolo = YOLO(model)
#         self.yolo.fuse()
#         self.yolo.to(device)

#         # topcis
#         self._pub = self.create_publisher(Detection2DArray, "detections", 10)
#         self._dbg_pub = self.create_publisher(Image, "dbg_image", 10)
#         self._sub = self.create_subscription(
#             Image, "/image_raw", self.image_cb,  # Image, "/camera/color/image_raw", self.image_cb,
#             qos_profile_sensor_data
#         )

#         # services
#         self._srv = self.create_service(SetBool, "enable", self.enable_cb)

#     def create_tracker(self, tracker_yaml) -> BaseTrack:

#         TRACKER_MAP = {"bytetrack": BYTETracker, "botsort": BOTSORT}
#         check_requirements("lap")  # for linear_assignment

#         tracker = check_yaml(tracker_yaml)
#         cfg = IterableSimpleNamespace(**yaml_load(tracker))

#         assert cfg.tracker_type in ["bytetrack", "botsort"], \
#             f"Only support 'bytetrack' and 'botsort' for now, but got '{cfg.tracker_type}'"
#         tracker = TRACKER_MAP[cfg.tracker_type](args=cfg, frame_rate=1)
#         return tracker

#     def enable_cb(self,
#                   req: SetBool.Request,
#                   res: SetBool.Response
#                   ) -> SetBool.Response:
#         self.enable = req.data
#         res.success = True
#         return res

#     def image_cb(self, msg: Image) -> None:

#         if self.enable:

#             # convert image + predict
#             cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
#             results = self.yolo.predict(
#                 source=cv_image,
#                 verbose=False,
#                 stream=False,
#                 conf=0.1,
#                 mode="track"
#             )

#             # track
#             det = results[0].boxes.cpu().numpy()

#             if len(det) > 0:
#                 im0s = self.yolo.predictor.batch[2]
#                 im0s = im0s if isinstance(im0s, list) else [im0s]

#                 tracks = self.tracker.update(det, im0s[0])
#                 if len(tracks) > 0:
#                     results[0].update(boxes=torch.as_tensor(tracks[:, :-1]))

#             # create detections msg
#             detections_msg = Detection2DArray()
#             detections_msg.header = msg.header

#             results = results[0].cpu()

#             for b in results.boxes:

#                 label = self.yolo.names[int(b.cls)]
#                 score = float(b.conf)

#                 if score < self.threshold:
#                     continue

#                 detection = Detection2D()

#                 box = b.xywh[0]

#                 # get boxes values
#                 detection.bbox.center.x = float(box[0]) # detection.bbox.center.position.x
#                 detection.bbox.center.y = float(box[1])
#                 detection.bbox.size_x = float(box[2])
#                 detection.bbox.size_y = float(box[3])

#                 # get track id
#                 track_id = -1
#                 if not b.id is None:
#                     track_id = int(b.id)
#                 #detection.id = str(track_id)

#                 # get hypothesis
#                 hypothesis = ObjectHypothesisWithPose()
#                 # hypothesis.hypothesis.class_id = label
#                 hypothesis.id = label
#                 # hypothesis.hypothesis.score = score
#                 hypothesis.score = score
#                 detection.results.append(hypothesis)

#                 # draw boxes for debug
#                 if label not in self._class_to_color:
#                     r = random.randint(0, 255)
#                     g = random.randint(0, 255)
#                     b = random.randint(0, 255)
#                     self._class_to_color[label] = (r, g, b)
#                 color = self._class_to_color[label]

#                 min_pt = (round(detection.bbox.center.x - detection.bbox.size_x / 2.0), # round(detection.bbox.center.position.x - detection.bbox.size_x / 2.0
#                           round(detection.bbox.center.y - detection.bbox.size_y / 2.0))
#                 max_pt = (round(detection.bbox.center.x + detection.bbox.size_x / 2.0),
#                           round(detection.bbox.center.y + detection.bbox.size_y / 2.0))
#                 cv2.rectangle(cv_image, min_pt, max_pt, color, 2)

#                 label = "{} ({}) ({:.3f})".format(label, str(track_id), score)
#                 pos = (min_pt[0] + 5, min_pt[1] + 25)
#                 font = cv2.FONT_HERSHEY_SIMPLEX
#                 cv2.putText(cv_image, label, pos, font,
#                             1, color, 1, cv2.LINE_AA)

#                 # append msg
#                 detections_msg.detections.append(detection)

#             # publish detections and dbg image
#             self._pub.publish(detections_msg)
#             self._dbg_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image,
#                                                                encoding=msg.encoding))
#         cv2.imshow('result', cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
#         cv2.waitKey(10)

# def main():
#     rclpy.init()
#     node = Yolov8Node()
#     rclpy.spin(node)
#     rclpy.shutdown()