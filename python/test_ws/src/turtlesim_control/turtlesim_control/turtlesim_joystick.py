import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TurtlesimJoystick(Node):

    def __init__(self):
        super().__init__('turtlesim_joystick')
        qos_profile = QoSProfile(depth=10)
        self.joy_sub = self.create_subscription(Joy,'/joy',self.joy_sub_callback, qos_profile)
        self.twist_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', qos_profile)
        self.timer = self.create_timer(1, self.pub_twist_msg)

        # 축 모음(인덱스 순서)
        self.LSTICK_LR = 0
        self.LSTICK_UD = 0
        self.LT = 1
        self.RSTICK_LR = 0
        self.RSTICK_UD = 0
        self.RT = 1
        self.DPAD_LR = 0
        self.DPAD_UD = 0

        # 버튼 모음(인덱스 순서)
        self.A_btn = 0
        self.B_btn = 0
        self.X_btn = 0
        self.Y_btn = 0
        self.LB_btn = 0
        self.RB_btn = 0
        self.BACK_btn = 0
        self.START_btn = 0
        self.HOME_btn = 0
        self.LSTICK_btn = 0
        self.RSTICK_btn = 0

    def joy_sub_callback(self, msg):
        # self.get_logger().info('조이스틱 연결됨')
        self.LSTICK_LR = msg.axes[0]
        self.LSTICK_UD = msg.axes[1]
        self.LT = msg.axes[2]
        self.RSTICK_LR = msg.axes[3]
        self.RSTICK_UD = msg.axes[4]
        self.RT = msg.axes[5]
        self.DPAD_LR = msg.axes[6]
        self.DPAD_UD = msg.axes[7]

        self.A_btn = msg.buttons[0]
        self.B_btn = msg.buttons[1]
        self.X_btn = msg.buttons[2]
        self.Y_btn = msg.buttons[3]
        self.LB_btn = msg.buttons[4]
        self.RB_btn = msg.buttons[5]
        self.BACK_btn = msg.buttons[6]
        self.START_btn = msg.buttons[7]
        self.HOME_btn = msg.buttons[8]
        self.LSTICK_btn = msg.buttons[9]
        self.RSTICK_btn = msg.buttons[10]

    def pub_twist_msg(self):
        msg = Twist()
        if self.LB_btn:
            print('조종 중')
            if self.LSTICK_UD >= 0.9:
                msg.linear.x = 1.0          # 전진
            elif self.LSTICK_UD <= -0.9:
                msg.linear.x = -1.0         # 후진
            elif self.LSTICK_LR >= 0.9:
                msg.linear.y = 1.0          # 왼쪽 게걸음
            elif self.LSTICK_LR <= -0.9:
                msg.linear.y = -1.0         # 오른쪽 게걸음

            if self.RSTICK_LR >= 0.9:
                msg.angular.z = 1.0         # 머리 왼쪽
            elif self.RSTICK_LR <= -0.9:
                msg.angular.z = -1.0        # 머리 오른쪽
            self.twist_pub.publish(msg)
        else:
            print('LB 버튼을 누르고 조작하세요')

def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimJoystick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
