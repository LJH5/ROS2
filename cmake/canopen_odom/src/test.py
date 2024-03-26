#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from canopen_odom.srv import Velocity


class CanOpenTest(Node):

    def __init__(self):
        super().__init__('can_open_test')
        self.cli = self.create_client(Velocity, "velocity")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available")
        self.req = Velocity.Request()


        self.left_wheel_vel = 150
        self.right_wheel_vel = 150
        self.move_time = 10

        while rclpy.ok():
            try:
                self.send_request()
                response = self.future.result()
                self.get_logger().info(f"res: {response}")

            except Exception as e:
                self.get_logger().info(e)

    def send_request(self):
        self.req.left_wheel_vel = self.left_wheel_vel
        self.req.right_wheel_vel = self.right_wheel_vel
        self.req.time = self.move_time

        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    can_open_test = CanOpenTest()
    rclpy.spin(can_open_test)
    can_open_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()