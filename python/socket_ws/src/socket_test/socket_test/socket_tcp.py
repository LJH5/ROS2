#!/usr/bin/env python3
# -*- coding: utf-8 -*- #

from ast import arg
import rclpy
import socket
from socket_test.srv import SendData
# from rospy.core import rospywarn
from std_msgs.msg import String
from rclpy.node import Node


class TCPServer(Node):
    def __init__(self, host, port):
        print(host, port)
        super().__init__('tcp_test')
        # load socket config
        self.host = host
        self.port = port

        # ros service
        self.srv = self.create_service(SendData, '/tcp_server/send_data', self.send_data)

        # Address family: IPv4 / Type: TCP
        self.get_logger().info("Create TCP server socket...")
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # port usage check
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        self.get_logger().info("The server socket has been initialized - " + self.host + ':' + str(self.port))

        self.client_ip, self.client_socket = None, None
        self.wait_for_client()

    def wait_for_client(self):
        """
        @brief      client socket과의 연결 대기
        """
        while rclpy.ok():
            # try:
            if True:
                self.get_logger().info("Wait for client...")
                self.client_socket, addr = self.server_socket.accept()
                self.get_logger().info('adrr: ' + addr)
                self.client_ip = addr[0]
                self.get_logger().info('Connected by ' + self.client_ip)
                break
            # except:
            #     self.get_logger().warn("Re-try connection...")


    def wait_for_data(self):
        """
        @brief      데이터 수신 대기 모드
        """
        while rclpy.ok():
            try:
                data = self.client_socket.recv(1024)
                self.get_logger().info('Received from ' + self.client_ip + ' - ' + data.decode())
                # self.data_pub.publish(data)
                self.client_socket.send('1')

            except:
                self.client_socket.close() # 기존 소켓과의 연결 해제
                self.get_logger().warn('Disconnected by ' + self.client_ip)
                self.wait_for_client() # 클라이언트 소켓 연결 대기
                self.get_logger().warn('Re-connected by ' + self.client_ip)


    def send_data(self, request, response):
        """
        @brief      client socket으로 데이터 송신

        @param      request         service request / SendData.srv -> String data
        @retrun     response        service response / SendData.srv -> String data
        """
        while rclpy.ok():
            try:
                self.client_socket.send(f'1,{request.data}'.encode())
                self.get_logger().info("Message sent to the client: " + str(request.data))

                return response

            except:
                # 소켓 연결에 문제가 생겼을 시 재 연결
                self.client_socket.close() # 기존 소켓과의 연결 해제
                self.get_logger().warn('Disconnected by ' + self.client_ip)
                self.wait_for_client() # 클라이언트 소켓 연결 대기
                self.get_logger().warn('Re-connected by ' + self.client_ip)


def main():
    rclpy.init(args=None)
    rclpy.create_node('tcp_test')
    tcp_server = TCPServer('localhost', 12345)
    try:
        rclpy.spin(tcp_server)
    except:
        rclpy.shutdown()
        tcp_server.destroy_node()


if __name__ == '__main__':
    main()