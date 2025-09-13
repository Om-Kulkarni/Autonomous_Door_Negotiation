import socket
import struct

import rospy
from sensor_msgs.msg import Image

# TCP parameters
dest_ip = "10.68.0.130"
dest_port = 5005

# Create a TCP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((dest_ip, dest_port))  # Connect to the TCP client


def callback(msg):
    # Convert ROS Image message to raw bytes
    raw_data = bytearray(msg.data)
    total_size = len(raw_data)

    # Step 1: Send 4-byte header (total size)
    sock.sendall(struct.pack(">I", total_size))

    # Step 2: Send the image data
    sock.sendall(raw_data)


def listener():
    rospy.init_node("image_subscriber")
    rospy.Subscriber("/xtion/rgb/image_raw", Image, callback)
    rospy.spin()
    sock.close()  # Close the socket when node shuts down


if __name__ == "__main__":
    listener()
