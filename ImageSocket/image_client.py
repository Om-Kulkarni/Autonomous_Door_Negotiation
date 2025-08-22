import socket
from PIL import Image
import numpy as np
import time


CHUNK_SIZE = 7200
LISTEN_IP = '0.0.0.0'
PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LISTEN_IP, PORT))

image_chunks = {}
expected_chunks = None

#print("Waiting for image chunks...")
start_time = time.time()
while True:
    data, addr = sock.recvfrom(CHUNK_SIZE + 4)

    if len(data) < 4:
        continue

    total_chunks = (data[0] << 8) + data[1]
    chunk_index = (data[2] << 8) + data[3]
    payload = data[4:]

    if expected_chunks is None:
        expected_chunks = total_chunks

    image_chunks[chunk_index] = payload

    #print(f"Received chunk {chunk_index + 1}/{total_chunks}")

    if len(image_chunks) == expected_chunks:
        #print("Image fully received!")

        # Reassemble image bytes
        image_data = b''.join([image_chunks[i] for i in sorted(image_chunks.keys())])

        # Convert raw bytes to numpy array and save as PNG
        width, height = 640, 480  # Adjust if needed

        arr = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width, 3))
        img = Image.fromarray(arr)

        img.save('received_image.png')
        print(time.time() - start_time)
        print("Image saved as received_image.png")

        break  # Exit after one image

sock.close()
