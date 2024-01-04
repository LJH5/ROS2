import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(depth=10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.scan_pub = self.create_publisher(LaserScan, 'scan_1', qos_profile)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.sacn_callback, qos_profile)

        self.scan_msg = LaserScan()

    def timer_callback(self):
        self.scan_msg.angle_max = 1.2
        self.scan_msg.angle_min = 1.0
        self.scan_pub.publish(self.scan_msg)

    def scan_callback(self, msg):
        self.scan_msg = msg

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()