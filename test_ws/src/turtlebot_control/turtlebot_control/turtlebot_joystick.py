import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped
from action_msgs.msg import GoalStatus

class TurtlebotJoystick(Node):

    def __init__(self):
        super().__init__('turtlebot_joystick')

        qos_profile = QoSProfile(depth=10)

        # Parameter Server
        self.declare_parameters(
            namespace='',
            parameters=[
                ('is_sim', True),
                ('is_debug', True),
                ('is_goal', False),
                ('max_speed', 1.0),
                ('min_speed', 0.05),
                ('default_speed', 0.3),
            ]
        )

        self.is_sim = self.get_parameter('is_sim')._value
        self.is_debug = self.get_parameter('is_debug')._value
        self.is_goal = self.get_parameter('is_goal')._value
        self.max_speed = self.get_parameter('max_speed')._value
        self.default_speed = self.get_parameter('default_speed')._value
        self.points = {
            'X' : {
                'position':[3.4501534899741504, 0.21125736574441306, 0.0],
                'orientation':[0.0, 0.0, -0.9805457129167302, 0.19629086805203474],
            },
            'Y' : {
                'position':[2.338565855433268, 2.42559073188208, 0.0],
                'orientation':[0.0, 0.0, 0.9795557358262224, 0.2011729614285879],
            },
            'A' : {
                'position':[0.2699406628449572, 1.8149537668832585, 0.0],
                'orientation':[0.0, 0.0, -0.8218713636688626, 0.5696731181836512],
            },
            'B' : {
                'position':[0.026617723363159953, 0.1764267246015428, 0.0],
                'orientation':[0.0, 0.0, -0.15265524102222397, 0.988279503677197],
            },
        }

        # ROS Message
        self.twist_msg = Twist()
        self.goal_msg = NavigateToPose.Goal()
        self.to_goal_msg = PoseStamped()

        # Publisher
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose',qos_profile)
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)

        # Subscriber
        self.joy_sub = self.create_subscription(Joy,'/joy',self.joy_sub_callback, qos_profile)
        self.pose_sub = self.create_subscription(PoseStamped, '/amcl_pose', self.pose_sub_callback, qos_profile)

        # Action
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # callback
        self.timer = self.create_timer(1, self.timer_callback)

        # 속도 구간 조절
        self.max_speed = 0.5
        self.min_speed = 0.05

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

        # 현재 속도
        self.now_speed = self.default_speed

        #
        self.action_result_future = None

    def timer_callback(self):
        self.speed_change()

        if self.A_btn:
            self.goal(self.points['A']['position'], self.points['A']['orientation'])
        elif self.B_btn:
            self.goal(self.points['B']['position'], self.points['B']['orientation'])
        elif self.X_btn:
            self.goal(self.points['X']['position'], self.points['X']['orientation'])
        elif self.Y_btn:
            self.goal(self.points['Y']['position'], self.points['Y']['orientation'])

        if self.RB_btn:
            self.cancel_move_to_goal()

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

    def twist_control(self):
        # print('조종 중')
        if self.LB_btn:
            print('조작 모드 ON')
            if self.LSTICK_UD >= 0.9:
                self.twist_msg.linear.x = self.now_speed    # 전진
            elif self.LSTICK_UD <= -0.9:
                self.twist_msg.linear.x = -self.now_speed   # 후진
            else:
                self.twist_msg.linear.x = 0.0               # 멈춤

            if self.RSTICK_LR >= 0.9:
                self.twist_msg.angular.z = 0.5              # 머리 왼쪽 회전
            elif self.RSTICK_LR <= -0.9:
                self.twist_msg.angular.z = -0.5             # 머리 오른쪽 회전
            else:
                self.twist_msg.angular.z = 0.0              # 멈춤
        else:
            self.stop()
        self.twist_pub.publish(self.twist_msg)

    def goal(self, goal_pose: list, orientation: list):
        # nav
        print('gg')
        self.to_goal_msg.pose.position.x = goal_pose[0]
        self.to_goal_msg.pose.position.y = goal_pose[1]
        self.to_goal_msg.pose.position.z = goal_pose[2]
        self.to_goal_msg.pose.orientation.x = orientation[0]
        self.to_goal_msg.pose.orientation.y = orientation[1]
        self.to_goal_msg.pose.orientation.z = orientation[2]
        self.to_goal_msg.pose.orientation.w = orientation[3]
        self.goal_msg.pose = self.to_goal_msg

        self.send_goal_future = self.action_client.send_goal_async(self.goal_msg, feedback_callback=self.get_nav_action_goal)

    def pose_sub_callback(self, msg):
        print(msg.header)
        print(msg.pose.pose.position)       # 터틀봇의 (x, y, z)
        print(msg.pose.pose.orientation)    # 터틀봇의 회전
        print(msg.pose.covariance)
        print('')

    def stop(self):
        self.twist_msg.linear.x = 0.0
        self.twist_msg.linear.y = 0.0
        self.twist_msg.angular.z = 0.0

    def speed_change(self):
        if self.BACK_btn:
            self.now_speed = max(self.min_speed, self.now_speed - 0.05)
        elif self.START_btn:
            self.now_speed = min(self.max_speed, self.now_speed + 0.05)
        # print(f'현재 속도: {self.now_speed:.2f} (max:{self.max_speed}, min: {self.min_speed})')

    def goal_callback(self, msg):
        print(msg)

    def get_nav_action_goal(self, future):
        future = self.send_goal_future
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warning('Action goal rejected')
            return
        self.get_logger().info('Action goal accepted')
        self.action_result_future = self.goal_handle.get_result_async()
        self.action_result_future.add_done_callback(self.get_nav_action_result)

    def get_nav_action_result(self, future):
        action_status = future.result().status
        action_result = future.result().result
        if action_status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('action succeeded')
            self.get_logger().info(f'action result: {action_result}')
        else:
            self.get_logger().warning(f'action failed: {action_status}')

    def cancel_move_to_goal(self):
        if self.action_result_future:
            print('이동 취소')
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancle_response_callback)

    def cancle_response_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('취소 성공')
        else:
            self.get_logger().info('취소 실패')


def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotJoystick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()