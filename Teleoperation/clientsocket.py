import socket

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(("0.0.0.0", 9999))


# ('127.0.0.1', 9999))#('10.251.72.253', 9999))#10.251.77.176
server_socket.listen(1)
print("Server listening on 10.251.72.253:9999")

conn, addr = server_socket.accept()
print("Client connected:", addr)

while True:
    data = conn.recv(1024)
    if not data:
        break
    print("Received:", data)
    conn.sendall(b"Hello from Python\n")

conn.close()
server_socket.close()
import socket
