#!/usr/bin/env python
"""
@file socket_server.py
@brief Socket server to receive torso and arm joint positions from simulation and command the Tiago robot via ROS topics.
"""

import pickle
import socket
import struct

import rospy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Socket configuration
HOST = "0.0.0.0"  # Listen on all interfaces
PORT = 65432

# Default start positions for torso and arm joints
DEFAULT_TORSO_POS = 0.30  # meters
DEFAULT_ARM_POSITIONS = [1.61, -0.93, -3.14, 1.83, -1.58, -0.62, -1.58]  # radians

# Gripper configuration
GRIPPER_OPEN_POS = [0.045, 0.045]  # Slightly less than max for safety
GRIPPER_CLOSED_POS = [0.0, 0.0]


def send_torso_command(z_height, torso_pub):
    traj = JointTrajectory()
    traj.joint_names = ["torso_lift_joint"]

    point = JointTrajectoryPoint()
    point.positions = [z_height]
    point.time_from_start = rospy.Duration(1.0)

    traj.points = [point]

    torso_pub.publish(traj)
    rospy.loginfo(f"Published torso height: {z_height:.3f}")


def send_arm_command(arm_positions, arm_pub):
    traj = JointTrajectory()
    traj.joint_names = [
        "arm_1_joint",
        "arm_2_joint",
        "arm_3_joint",
        "arm_4_joint",
        "arm_5_joint",
        "arm_6_joint",
        "arm_7_joint",
    ]

    point = JointTrajectoryPoint()
    point.positions = arm_positions
    point.time_from_start = rospy.Duration(1.0)

    traj.points = [point]

    arm_pub.publish(traj)
    rospy.loginfo(f"Published arm joint positions: {arm_positions}")


def send_base_command(lin_vel_x, ang_vel_z, base_pub):
    """Creates and publishes a Twist message to control the mobile base."""
    twist_msg = Twist()
    twist_msg.linear.x = lin_vel_x
    twist_msg.angular.z = ang_vel_z
    base_pub.publish(twist_msg)


def send_gripper_command(positions, gripper_pub):
    """Creates and publishes a JointTrajectory message to control the gripper."""
    traj = JointTrajectory()
    traj.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration(1.0)
    traj.points = [point]
    gripper_pub.publish(traj)
    rospy.loginfo(f"Published gripper positions: {positions}")


def recv_msg(conn):
    """Helper function to receive a message with a 4-byte length prefix."""
    # Read the header to get the message length
    raw_msglen = conn.recv(4)
    if not raw_msglen:
        return None
    msglen = struct.unpack(">I", raw_msglen)[0]

    # Read the full message payload
    data = bytearray()
    while len(data) < msglen:
        packet = conn.recv(msglen - len(data))
        if not packet:
            return None
        data.extend(packet)
    return data


def main():
    rospy.init_node("socket_server", anonymous=True)

    torso_pub = rospy.Publisher("/torso_controller/command", JointTrajectory, queue_size=10)
    arm_pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=10)
    base_pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)
    gripper_pub = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=10)

    # Publish default start positions once at launch
    rospy.sleep(1.0)  # Wait for publishers to register
    send_torso_command(DEFAULT_TORSO_POS, torso_pub)
    send_arm_command(DEFAULT_ARM_POSITIONS, arm_pub)
    send_gripper_command(GRIPPER_OPEN_POS, gripper_pub)
    rospy.loginfo("Published default start positions on startup.")

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow socket reuse

    try:
        server_socket.bind((HOST, PORT))
        server_socket.listen(1)
        rospy.loginfo(f"Listening for joint commands on port {PORT}")

        while not rospy.is_shutdown():
            try:
                rospy.loginfo("Waiting for connection...")
                conn, addr = server_socket.accept()
                rospy.loginfo(f"Connected by {addr}")

                # Handle connection - Python 2 compatible way
                try:
                    while not rospy.is_shutdown():
                        try:
                            data = recv_msg(conn)
                            if not data:
                                rospy.logwarn("No data received, client likely disconnected")
                                break

                            try:
                                joint_command = pickle.loads(data)
                                if (
                                    not isinstance(joint_command, (list, tuple))
                                    or len(joint_command) != 11
                                ):
                                    rospy.logwarn(
                                        "Received invalid joint command format: %s",
                                        joint_command,
                                    )
                                    continue

                                torso_pos = joint_command[0]
                                arm_positions = joint_command[1:8]  # Takes joints 1 through 7
                                lin_vel_x = joint_command[8]
                                ang_vel_z = joint_command[9]
                                gripper_command = joint_command[10]  # 0: no-op, 1: open, 2: close

                                send_torso_command(torso_pos, torso_pub)
                                send_arm_command(arm_positions, arm_pub)
                                send_base_command(lin_vel_x, ang_vel_z, base_pub)

                                if gripper_command == 1:
                                    send_gripper_command(GRIPPER_OPEN_POS, gripper_pub)
                                elif gripper_command == 2:
                                    send_gripper_command(GRIPPER_CLOSED_POS, gripper_pub)

                            except (pickle.UnpicklingError, ValueError) as e:
                                rospy.logwarn(f"Failed to parse joint command: {e}")
                                continue

                        except socket.timeout:
                            # Timeout is fine, just continue
                            continue
                        except OSError as e:
                            rospy.logerr(f"Socket error while receiving data: {e}")
                            break

                except Exception as e:
                    rospy.logerr(f"Error handling connection: {e}")
                finally:
                    try:
                        # Stops robot when connection closes
                        rospy.loginfo("Client disconnected. Stopping base movement.")
                        send_base_command(0.0, 0.0, base_pub)

                        conn.close()
                        rospy.loginfo("Connection closed")
                    except Exception:
                        pass

            except OSError as e:
                rospy.logerr(f"Socket error while accepting connections: {e}")
                rospy.sleep(1.0)  # Wait before retrying
                continue
            except Exception as e:
                rospy.logerr(f"Unexpected error: {e}")
                break

    except KeyboardInterrupt:
        rospy.loginfo("Server interrupted by user")
    except Exception as e:
        rospy.logerr(f"Server error: {e}")
    finally:
        try:
            server_socket.close()
            rospy.loginfo("Server socket closed")
        except Exception:
            pass


if __name__ == "__main__":
    main()
