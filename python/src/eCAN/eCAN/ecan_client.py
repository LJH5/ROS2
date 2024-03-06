import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry

import socket

class EcanClient(Node):

    def __init__(self):
        super().__init__('ecan_client')

        # Client TCP connect
        HOST = "192.168.100.120"
        PORT = 4001
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((HOST, PORT))

        self.client_socket.send(b"\x77\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00")

        try:
            self.get_logger().info("수신중")
            data = self.client_socket.recv(1024)
            if not data:
                self.get_logger().warn("data 없음")
            else:
                self.get_logger().info(f"data:{data}")
        except Exception as e:
            self.get_logger().warn(f"오류: {e}")
        finally:
            self.client_socket.close()
def main(args=None):
    rclpy.init(args=args)
    ecan_client = EcanClient()
    rclpy.spin(ecan_client)
    ecan_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()