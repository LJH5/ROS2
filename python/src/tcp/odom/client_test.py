import socket
import threading

HOST = '192.168.100.120'
PORT = 4001

client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))

# 서버로부터 메세지를 받는 메소드
# 스레드로 구동 시켜, 메세지를 보내는 코드와 별개로 작동하도록 처리
def recv_data() :
    while True :
        data = client_socket.recv(1024)

        print("recive : ",repr(data.decode()))

recv_data_thread = threading.Thread(target=recv_data)
recv_data_thread.start()
print ('>> Connect Server')

while True:
    message = input('')
    if message == 'quit':
        close_data = message
        break

    client_socket.send(message.encode())


client_socket.close()