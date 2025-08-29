import rospy
from sensor_msgs.msg import Joy
import socket
import pickle

HOST = '0.0.0.0'
PORT = 65432

class JoySocketServer:
    def __init__(self):
        # Start ROS node
        rospy.init_node('joy_socket_server')

        # Latest axes values
        self.axes = [0.0, 0.0, 0.0]

        # Set up subscriber to /joy topic
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Set up socket server
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((HOST, PORT))
        self.s.listen(1)
        print("Server listening on {}:{}".format(HOST, PORT))

        self.conn, self.addr = self.s.accept()
        print("Connected by", self.addr)

    def joy_callback(self, data):
        # Update latest axes (taking first 3)
        if len(data.axes) >= 3:
            self.axes = data.axes[:3]

    def run(self):
        try:
            rate = rospy.Rate(10)  # 10 Hz
            while not rospy.is_shutdown():
                x, y, z = self.axes

                # Send x,y,z to client
                data_to_send = (x, y, z)
                self.conn.sendall(pickle.dumps(data_to_send))

                # Receive transformed data from client
                received_data = self.conn.recv(1024)
                if not received_data:
                    print("Client disconnected.")
                    break

                transformed = pickle.loads(received_data)
                print("Received from client:", transformed)

                rate.sleep()
        except rospy.ROSInterruptException:
            pass
        except KeyboardInterrupt:
            print("Shutting down server.")
        finally:
            self.conn.close()
            self.s.close()

if __name__ == '__main__':
    server = JoySocketServer()
    server.run()
