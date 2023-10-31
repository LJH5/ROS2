import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
HOST = '192.168.0.91'
PORT = 12345
sock.connect((HOST, PORT))

while True:
    message = input("Enter a message: ")
    sock.send(message.encode('utf-8'))
