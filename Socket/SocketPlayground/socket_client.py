import socket
import pickle

HOST = '10.68.0.1'  # Replace with your server's IP
PORT = 65432

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))

    while True:
        data = s.recv(1024)
        if not data:
            break

        x, y, z = pickle.loads(data)
        print(f"Received from server: x={x}, y={y}, z={z}")

        transformed = [x, y, z, x, y, z, x]
        s.sendall(pickle.dumps(transformed, protocol=2))  # ✅ Fix here
        print(f"Sent to server: {transformed}")
