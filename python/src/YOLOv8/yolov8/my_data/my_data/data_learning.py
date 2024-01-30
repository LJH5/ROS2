import rclpy
from rclpy.node import Node
from ultralytics import YOLO

class DataLearningNode(Node):
    def __init__(self):
        super().__init__('data_learning_node')
        self.learning()

    def learning(self):
        model = YOLO('yolov8s.pt')
        model.train(data='/home/roboarm/yolo_ws/src/my_data/data_set/data.yaml', epochs=5)

def main(args=None):
    print("data_learning 노드에서 안녕하세요!")
    rclpy.init(args=args)
    data_learning_node = DataLearningNode()
    rclpy.spin(data_learning_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()