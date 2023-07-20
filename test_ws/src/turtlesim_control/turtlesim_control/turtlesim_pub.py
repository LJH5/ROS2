import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist

class TurtlesimPub(Node):

    def __init__(self):
        super().__init__('turtlesim_pub')
        qos_profile = QoSProfile(depth=10)
        self.goal_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', qos_profile)
        self.timer = self.create_timer(1, self.pub_goal_msg)

    def pub_goal_msg(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 1.0
        self.goal_pub.publish(msg)
        self.get_logger().info('작동중')

def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
