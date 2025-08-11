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

    def receive_data(self, client_socket):
        try:
            data = client_socket.recv(1024)
            if data:
                print("Data received: ", data.decode())  # Debug log
                try:
                    joint_state_data = json.loads(data.decode())
                    rospy.loginfo("Received Joint State: %s", joint_state_data)

                    # Create the JointTrajectory message
                    joint_trajectory_msg = JointTrajectory()
                    joint_trajectory_msg.header.stamp = rospy.Time.now()
                    joint_trajectory_msg.header.frame_id = ''

                    # Set the joint names for the trajectory
                    joint_trajectory_msg.joint_names = [
                        'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 
                        'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
                    ]

                    # Define the point (joint positions and time_from_start)
                    point = JointTrajectoryPoint()
                    point.positions = [joint_state_data['arm_1_joint'], joint_state_data['arm_2_joint'], 
                                       joint_state_data['arm_3_joint'], joint_state_data['arm_4_joint'], 
                                       joint_state_data['arm_5_joint'], joint_state_data['arm_6_joint'], 
                                       joint_state_data['arm_7_joint']]
                    point.velocities = []
                    point.accelerations = []
                    point.effort = []
                    point.time_from_start = rospy.Duration(1)  # 1 second from start

                    # Add the point to the trajectory
                    joint_trajectory_msg.points.append(point)

                    # Publish the JointTrajectory message
                    rospy.loginfo("Publishing Joint Trajectory: %s", joint_trajectory_msg)  # Debug log
                    joint_trajectory_pub.publish(joint_trajectory_msg)

                    # Send ACK to the client after successful processing
                    print("Sending ACK to client...")  # Debug log
                    client_socket.send(b"ACK")

                except Exception as e:
                    rospy.logerr("Error processing received data: %s", str(e))
                    # Send NACK in case of error
                    client_socket.send(b"NACK")  
                    print("Error while processing. Sent NACK.")
            else:
                print("No data received. Closing connection.")
                client_socket.close()
        except socket.error as e:
            rospy.logerr("Error receiving data: %s", str(e))

    def close(self):
        self.server_socket.close()
        print("Server closed.")

if __name__ == '__main__':
    server = TCPServer('0.0.0.0', 51004)
    while not rospy.is_shutdown():
        client_socket = server.accept_client()
        server.receive_data(client_socket)  # Handle data reception
    server.close()
