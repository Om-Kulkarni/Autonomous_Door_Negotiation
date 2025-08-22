#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState

class ArmJoyTrajectoryPublisher:
    def __init__(self):
        rospy.init_node('arm_joy_trajectory_publisher')

        # Arm joint names
        self.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
            'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        # Current joint positions (will be updated via /arm_controller/state)
        self.current_positions = [0.0] * len(self.joint_names)

        # How much to increment per axis push
        self.increment_step = 0.02

        # Publisher to trajectory topic
        self.arm_pub = rospy.Publisher(
            '/arm_controller/command',
            JointTrajectory,
            queue_size=1
        )

        # Subscribe to joystick input
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Subscribe to arm controller state to update actual joint positions
        rospy.Subscriber('/arm_controller/state', JointTrajectoryControllerState, self.state_callback)

        rospy.loginfo("ArmJoyTrajectoryPublisher is running and listening to /joy and /arm_controller/state")

    def state_callback(self, msg):
        # Update current joint positions from controller state
        self.current_positions = list(msg.actual.positions)

    def joy_callback(self, msg):
        if len(msg.axes) < 6:
            rospy.logwarn("Received /joy message with insufficient axes.")
            return

        # Compute delta values from joystick axes
        deltas = [msg.axes[i] * self.increment_step for i in range(6)]
        deltas.append(0.0)  # 7th joint unused here, or assign a button later

        # Skip if there's no significant movement
        if all(abs(d) < 1e-6 for d in deltas):
            return

        # Calculate new joint positions from actual positions
        new_positions = [
            curr + delta for curr, delta in zip(self.current_positions, deltas)
        ]

        # Build trajectory message
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = new_positions
        point.time_from_start.secs = 1  # Move within 1 second

        traj.points.append(point)

        rospy.loginfo("Publishing new joint trajectory: %s", new_positions)
        self.arm_pub.publish(traj)

def main():
    try:
        ArmJoyTrajectoryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
