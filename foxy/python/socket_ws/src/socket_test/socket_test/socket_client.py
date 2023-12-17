import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
HOST = 'localhost'
PORT = 12345
sock.connect((HOST, PORT))

while True:
    message = input("Enter a message: ")
    sock.send(message.encode('utf-8'))
