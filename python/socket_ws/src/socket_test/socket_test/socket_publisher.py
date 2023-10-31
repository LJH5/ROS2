import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket


class SocketPublisher(Node):

    def __init__(self):
        super().__init__('socket_publisher')
        self.socket_pub_ = self.create_publisher(String, 'socket_topic', 10)
        self.my_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = '192.168.0.91'
        port = 12345
        self.my_socket.bind((host, port))
        self.my_socket.listen(1)
        self.conn, addr = self.my_socket.accept()
        print(f'addr: {addr}')
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        data = self.conn.recv(1024).decode('utf-8')
        msg = String()
        msg.data = data
        self.socket_pub_.publish(msg)
        self.get_logger().info(f'Published: {data}')


def main(args=None):
    rclpy.init(args=args)
    socket_publisher = SocketPublisher()
    rclpy.spin(socket_publisher)
    socket_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()