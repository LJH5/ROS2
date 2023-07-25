import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped
from action_msgs.msg import GoalStatus

class Test3(Node):

    def __init__(self):
        super().__init__('test3')

        qos_profile = QoSProfile(depth=10)

        self.test_act = ActionClient

def main(args=None):
    rclpy.init(args=args)
    node = Test3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()