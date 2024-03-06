import rclpy
from rclpy.node import Node

import socket
from _thread import *

class ServerSocket(Node):

    def __init__(self):
        super().__init__('server_socket')

        # server 설정
        HOST = socket.gethostbyname(socket.gethostname())
        PORT = 9999

        # TCP 통신으로 설정
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # UDP 통신으로 설정
        # self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # 소켓 옵션 설정
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # 소켓 정의
        self.server_socket.bind((HOST, PORT))

        # 대기열 생성(100명까지)
        self.server_socket.listen(100)

        # client들의 정보를 담을 변수
        self.client_sockets = []

        # 연결 정보 출력
        self.get_logger().info(f"server start: {HOST}, {PORT}")

        self.wait_client()

    def wait_client(self):
        """
        @brief  client 연결 대기
        """
        while rclpy.ok():
            try:
                # 연결 시도
                self.get_logger().info("wating client...")
                client_socket, addr = self.server_socket.accept()
                self.client_sockets.append(client_socket)
                self.get_logger().info(f"client socket: {client_socket}")
                self.get_logger().info(f"add info: {addr}")
                start_new_thread(self.wait_datas, (client_socket, addr))
                self.get_logger().info(f"참가자 수: {len(self.client_sockets)}")
            except Exception as e:
                self.get_logger().info(f"연결 오류: {e}")

        self.server_socket.close()

    def wait_datas(self, client_socket, addr):
        """
        @brief  data 수신 대기
        """
        while rclpy.ok():
            try:
                # client로부터 테이터 수신
                self.get_logger().info("wait data...")
                self.data = client_socket.recv(1024)
                if not self.data:
                    self.get_logger().info("None data")
                else:
                    self.get_logger().info(f"수신: {self.data.decode()}")

                # 다른 client들에게 데이터 보내기
                for client in self.client_sockets:
                    if client != client_socket:
                        client.send(self.data)
            except ConnectionResetError as e:
                self.get_logger().info(f"수신 오류: {e}")

        # client가 접속을 종료 시 참가자 list에서 삭제
        if client_socket in self.client_sockets:
            self.client_sockets.remove(client_socket)
            print('참가자 수: ', len(self.client_sockets))

def main(args=None):
    rclpy.init(args=args)
    server_socket = ServerSocket()
    rclpy.spin(server_socket)
    server_socket.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()