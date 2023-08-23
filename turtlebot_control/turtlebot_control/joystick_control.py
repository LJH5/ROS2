import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav2_msgs.action import NavigateToPose
from turtlebot_control.turtlebot_joystick import TurtlebotJoystick

class JoystickControl(Node):

    def __init__(self):
        super().__init__('joystick_control')
        qos_profile = QoSProfile(depth=10)
        self.timer = self.create_timer(1, self.timer_callback)

        self.position= [3.4501534899741504, 0.21125736574441306, 0.0]
        self.orientation = [0.0, 0.0, -0.9805457129167302, 0.19629086805203474]
        self.node = TurtlebotJoystick()

    def timer_callback(self):
        print('a')
        self.node.goal(self.position, self.orientation)
        print(self.node.LB_btn)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()