#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@file tiago.py
@brief Hardware driver class for the Tiago robot using ROS.
@details This class abstracts the ROS communication for controlling the arm, base, and gripper,
         and for gathering sensor observations. It is designed to be used by a network host script.
"""

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry  
import threading

class Tiago:
    """A class to control the Tiago robot via ROS topics."""

    def __init__(self):
        """Initializes ROS publishers and subscribers."""
        rospy.loginfo("Initializing Tiago Hardware Driver...")

        # Publishers
        self.torso_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=10)
        self.arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
        self.base_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
        self.gripper_pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)

        # Subscribers
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self._joint_state_callback)
        # NEW: Subscriber for base velocity from odometry data
        self.odom_sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, self._odom_callback)

        # Observation data storage and lock
        self._latest_observation = {}
        self._lock = threading.Lock()

        # Gripper configuration
        self.GRIPPER_OPEN_POS = [0.045, 0.045]
        self.GRIPPER_CLOSED_POS = [0.0, 0.0]

        rospy.sleep(1.0)  # Wait for publishers and subscribers to register
        rospy.loginfo("Tiago Hardware Driver Initialized.")

    def _joint_state_callback(self, msg):
        """Callback to update the latest joint states for arm, torso, and gripper."""
        # Process joint state information
        with self._lock:
            for i, name in enumerate(msg.name):
                # UPDATED: Now includes gripper joints in the observation
                if 'arm' in name or 'torso' in name or 'gripper' in name:
                    # Storing with a '.pos' suffix for consistency
                    self._latest_observation[f"{name}.pos"] = msg.position[i]

    def _odom_callback(self, msg):
        """Callback to update the latest base velocities."""
        with self._lock:
            # Storing with a '.vel' suffix for consistency
            self._latest_observation['base_linear_x.vel'] = msg.twist.twist.linear.x
            self._latest_observation['base_angular_z.vel'] = msg.twist.twist.angular.z

    def get_observation(self):
        """
        Retrieves the latest observation data from the robot.
        
        Returns:
            dict: A dictionary containing the latest sensor data including joint positions,
                  gripper values, and base velocities.
        """
        with self._lock:
            # In a real scenario, you would also add camera data here
            return self._latest_observation.copy()
        
    def send_action(self, action):
        """
        Receives an action dictionary and commands the robot accordingly.

        Args:
            action (dict): A dictionary containing commands for different parts of the robot.
                           Example:
                           {
                               'torso_lift_joint.pos': 0.3,
                               'arm_joint_positions': [1.6, -0.9, ...],
                               'base_linear_velocity': 0.1,
                               'base_angular_velocity': -0.2,
                               'gripper_command': 1  # 1 for open, 2 for close
                           }
        """

        if 'arm_joint_positions' in action and len(action['arm_joint_positions']) == 5:
            # Pad the 5-DOF command to 7-DOF for Tiago's controller.
            # THIS MUST BE ADJUSTED BASED ON YOUR ACTUAL 5-DOF SETUP
            full_arm_positions = list(action['arm_joint_positions'])
            full_arm_positions.insert(2, 0.0) # Example: arm_3_joint fixed at 0.0
            full_arm_positions.insert(5, 0.0) # Example: arm_6_joint fixed at 0.0
            self._send_arm_command(full_arm_positions)

        if 'torso_lift_joint.pos' in action:
            self._send_torso_command(action['torso_lift_joint.pos'])

        # --- BASE ---
        lin_vel = action.get('base_linear_velocity', 0.0)
        ang_vel = action.get('base_angular_velocity', 0.0)
        self._send_base_command(lin_vel, ang_vel)

        # --- GRIPPER ---
        gripper_cmd = action.get('gripper_command')
        if gripper_cmd == 1:
            self._send_gripper_command(self.GRIPPER_OPEN_POS)
        elif gripper_cmd == 2:
            self._send_gripper_command(self.GRIPPER_CLOSED_POS)


    def _send_torso_command(self, z_height):
        """Sends a command to the torso."""
        traj = JointTrajectory()
        traj.joint_names = ['torso_lift_joint']
        point = JointTrajectoryPoint()
        point.positions = [z_height]
        point.time_from_start = rospy.Duration(0.5)
        traj.points = [point]
        self.torso_pub.publish(traj)

    def _send_arm_command(self, arm_positions):
        """Sends a command to the arm."""
        traj = JointTrajectory()
        traj.joint_names = [f'arm_{i}_joint' for i in range(1, 8)]
        point = JointTrajectoryPoint()
        point.positions = arm_positions
        point.time_from_start = rospy.Duration(0.5)
        traj.points = [point]
        self.arm_pub.publish(traj)

    def _send_base_command(self, lin_vel_x, ang_vel_z):
        """Sends a command to the mobile base."""
        twist_msg = Twist()
        twist_msg.linear.x = lin_vel_x
        twist_msg.angular.z = ang_vel_z
        self.base_pub.publish(twist_msg)

    def _send_gripper_command(self, positions):
        """Sends a command to the gripper."""
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(0.5)
        traj.points = [point]
        self.gripper_pub.publish(traj)

    def stop_robot(self):
        """Stops all robot movement safely."""
        rospy.loginfo("Stopping all robot movement.")
        self._send_base_command(0.0, 0.0)