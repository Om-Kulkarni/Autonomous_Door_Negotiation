import rospy
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import socket
import pickle

HOST = '0.0.0.0'
PORT = 65432

class JoySocketServer:
    def __init__(self):
        # Start ROS node
        rospy.init_node('joy_socket_server')

        # Latest axes values from joystick
        self.axes = [0.0, 0.0, 0.0,0.0,0.0,0.0]

        # Joint names of the robot arm
        self.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
            'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        # Set up ROS publisher for the robot arm
        self.arm_pub = rospy.Publisher(
            '/arm_controller/command',
            JointTrajectory,
            queue_size=1
        )

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
        # Read the first 3 axes and scale them
        
        self.axes = [a * 0.1 for a in data.axes]

    def publish_to_arm(self, joint_positions):
        # Ensure we have 7 values
        if len(joint_positions) != 7:
            rospy.logwarn("Received joint position list of invalid length: %d", len(joint_positions))
            return

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rospy.Duration(1.0)

        traj.points.append(point)

        rospy.loginfo("Publishing joint trajectory: %s", joint_positions)
        self.arm_pub.publish(traj)

    def run(self):
        try:
            rate = rospy.Rate(10)  # 10 Hz
            while not rospy.is_shutdown():
                # Send current joystick-derived x, y, z to client
                x, y, z,roll,pitch,yaw = self.axes
                data_to_send = (x, y, z,roll,pitch,yaw)
                self.conn.sendall(pickle.dumps(data_to_send))

                # Receive transformed data from client
                received_data = self.conn.recv(1024)
                if not received_data:
                    print("Client disconnected.")
                    break

                try:
                    transformed = pickle.loads(received_data)
                except Exception as e:
                    rospy.logwarn("Failed to deserialize data: %s", e)
                    continue

                print("Received from client:", transformed)

                # Send transformed joint values to robot arm
                self.publish_to_arm(transformed)

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
