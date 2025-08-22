import socket
from PIL import Image
import numpy as np
import struct
import time

def recvall(sock, n):
    """Helper to receive exactly n bytes from socket."""
    data = b''
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            raise ConnectionError("Connection lost during transfer.")
        
        data += packet
        print(len(data))
    return data

LISTEN_IP = '0.0.0.0'
PORT = 5004
WIDTH, HEIGHT = 640, 480  # Must match sender's image dimensions

server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_sock.bind((LISTEN_IP, PORT))
server_sock.listen(1)

print("Waiting for TCP connection...")
conn, addr = server_sock.accept()
print(f"Connected to {addr}")

start_time = time.time()

# Step 1: Receive 4-byte length header
header = recvall(conn, 4)
image_size = struct.unpack('>I', header)[0]
print(f"Receiving image of size: {image_size} bytes")

# Step 2: Receive full image data
image_data = recvall(conn, image_size)

# Step 3: Convert to image
arr = np.frombuffer(image_data, dtype=np.uint8).reshape((HEIGHT, WIDTH, 3))
img = Image.fromarray(arr)
img.save('received_image.png')

print(f"Image saved as received_image.png in {time.time() - start_time:.2f} seconds")

conn.close()
server_sock.close()
