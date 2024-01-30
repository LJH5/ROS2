import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class SocketSubscriber(Node):

    def __init__(self):
        super().__init__('socket_subscriber')
        self.socket_sub = self.create_subscription(
            String,
            'socket_topic',
            self.listener_callback,
            10)
        self.socket_sub  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    socket_subscriber = SocketSubscriber()
    rclpy.spin(socket_subscriber)
    socket_subscriber.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()