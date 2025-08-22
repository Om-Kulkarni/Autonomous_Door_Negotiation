import socket
from sensor_msgs.msg import Image
import rospy

CHUNK_SIZE = 7200
dest_ip = '10.68.0.130'
dest_port = 5005

sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

def callback(msg):
        #print(len(msg.data))
        #print('Thats an image')
        #print(type(msg.data[0]))       
        #pixel_value = [ord(b) for b in msg.data]
        #print(pixel_value[0])
        #print(pixel_value[1])  
        raw_data = bytearray(msg.data)
        total_size = len(raw_data)

        num_chunks = (total_size + CHUNK_SIZE -1) //CHUNK_SIZE

        for i in range(num_chunks):
                start = i * CHUNK_SIZE
                end = start + CHUNK_SIZE
                chunk_data = raw_data[start:end]

                # Prefix each chunk with a 4-byte header: [total_chunks (2 bytes)][chunk_index (2 bytes)]
                header = chr((num_chunks >> 8) & 0xFF) + chr(num_chunks & 0xFF) + chr((i >> 8) & 0xFF) + chr(i & 0xFF)
                packet = header + chunk_data

        sock.sendto(packet, (dest_ip, dest_port))

def listener():
        rospy.init_node('image_subscriber')
        rospy.Subscriber('/xtion/rgb/image_raw',Image,callback)
        rospy.spin()

if __name__ == '__main__':
        listener()
