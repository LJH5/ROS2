#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial
import ros2
from std_srvs.srv import SetBool

from example_interfaces.srv import AddTwoInts
 
class AddTwoIntsClientNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self.call_add_two_ints_server(6, 7)
        self.timer = self.create_timer(1, self.timecallback)
    
    def timecallback(self):
        print("hell")
        
    def call_add_two_ints_server(self, a, b):
        client = ros2.ServiceProxy('set_boot', SetBool)
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Add Two Ints...")

        response = client(data=True)

    def callback_call_add_two_ints(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(str(a) + " + " + 
                                   str(b) + " = " + str(response.sum))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
 
def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()