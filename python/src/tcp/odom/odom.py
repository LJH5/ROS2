import rclpy
from rclpy.node import Node
import socket
import threading


class ConnectClient(Node):

    def __init__(self):
        super().__init__('connect_client')
        self.my_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        HOST = '192.168.100.120'
        PORT = 4001
        timer_period = 0.5

        # 소켓이 사용 중 일때 에러 해결
        self.my_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.my_socket.connect((HOST, PORT))
        # 소켓이 연결된 원격 주소 반환
        print(self.my_socket.getpeername())

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        print(self.my_socket.recv(1024))

def main(args=None):
    rclpy.init(args=args)
    connect_client = ConnectClient()
    rclpy.spin(connect_client)
    connect_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()