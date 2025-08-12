#!/usr/bin/env python
import rospy
import json
import socket
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Initialize the ROS node
rospy.init_node('tcp_joint_state_listener', anonymous=True)

# Publisher for joint trajectory
joint_trajectory_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

class TCPServer:
    def __init__(self, ip, port):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ip = ip
        self.port = port
        self.server_socket.bind((self.ip, self.port))
        self.server_socket.listen(5)
        print("Server listening on {}:{}".format(self.ip, self.port))

    def accept_client(self):
        client_socket, client_address = self.server_socket.accept()
        print("Connection from {}".format(client_address))
        return client_socket

    def handle_client(self, client_socket):
        while not rospy.is_shutdown():
            try:
                data = client_socket.recv(1024)
                # If recv returns an empty string, the client has closed the connection
                if not data:
                    print("Client disconnected.")
                    break

                print("Received data: ", data.decode())  # Debug log

                try:
                    joint_state_data = json.loads(data.decode())
                    rospy.loginfo("Received Joint State: %s", joint_state_data)

                    # Create Joint state trajectory message
                    joint_trajectory_msg = JointTrajectory()
                    joint_trajectory_msg.header.stamp = rospy.Time.now()
                    joint_trajectory_msg.header.frame_id = ''

                    # Set the joint names for the trajectory
                    joint_trajectory_msg.joint_names = [
                        'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 
                        'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
                    ]

                    # Define the point (Joint positions and time_from_start)
                    point = JointTrajectoryPoint()
                    point.positions = [joint_state_data['arm_1_joint'], 
                                       joint_state_data['arm_2_joint'],
                                       joint_state_data['arm_3_joint'],
                                       joint_state_data['arm_4_joint'],
                                       joint_state_data['arm_5_joint'],
                                       joint_state_data['arm_6_joint'],
                                       joint_state_data['arm_7_joint']]
                    point.time_from_start = rospy.Duration(1)  # 1 second from start

                    # Add point to the trajectory
                    joint_trajectory_msg.points.append(point)

                    joint_trajectory_pub.publish(joint_trajectory_msg)
                    rospy.loginfo("Published Joint Trajectory")

                    # Send ACK to the client after successful processing
                    print("Sending ACK to client...")  # Debug log
                    client_socket.send(b"ACK")

                except Exception as e:
                    rospy.logerr("Error processing received data: %s", str(e))
                    client_socket.send(b"NACK")  
                    print("Error while processing. Sent NACK.")   

            except socket.error as e:
                rospy.logerr("Socket error during communication: %s", str(e))
                break # Exit loop on socket error 

        # Close the client socket once the loop is broken
        client_socket.close()
        print("Client socket closed.")

    def close(self):
        self.server_socket.close()
        print("Server closed.")

if __name__ == '__main__':
    server = TCPServer('0.0.0.0', 51004)
    try:
        while not rospy.is_shutdown():
            client_socket = server.accept_client()
            server.handle_client(client_socket)
    except Exception as e:
        rospy.logerr("Error occurred in server: %s", str(e))
    finally:
        server.close()
