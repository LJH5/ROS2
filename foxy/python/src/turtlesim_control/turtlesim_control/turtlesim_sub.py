import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtlesimSub(Node):

    def __init__(self):
        super().__init__('turtlesim_sub')
        qos_profile = QoSProfile(depth=10)
        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.listener_callback,
            qos_profile)

        self.x = 0
        self.y = 0
        self.theta = 0
        self.linear = 0
        self.angular = 0


    def listener_callback(self, msg):
        self.x = round(msg.x, 4)
        self.y = round(msg.y, 4)
        self.theta = msg.theta
        self.linear = msg.linear_velocity
        self.angular = msg.angular_velocity

        self.get_logger().info(f'x: {self.x}, y: {self.y}, theta:{self.theta}, linear: {self.linear}, angular:{self.angular}')


def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimSub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
