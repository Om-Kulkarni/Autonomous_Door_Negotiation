#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmJoyTrajectoryPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("arm_joy_trajectory_publisher")

        # Arm joint names
        self.joint_names = [
            "arm_1_joint",
            "arm_2_joint",
            "arm_3_joint",
            "arm_4_joint",
            "arm_5_joint",
            "arm_6_joint",
            "arm_7_joint",
        ]

        # Initialize joint positions to zero
        self.current_positions = [0.0] * len(self.joint_names)

        # Max increment per axis input
        self.increment_step = 0.02  # radians per input step

        # Publisher to arm trajectory controller
        self.arm_pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)

        # Subscribe to joystick input
        rospy.Subscriber("/joy", Joy, self.joy_callback)

        rospy.loginfo("ArmJoyTrajectoryPublisher ready and listening to /joy.")

    def joy_callback(self, msg):
        if len(msg.axes) < 6:
            rospy.logwarn("Received /joy message with insufficient axes.")
            return

        deltas = []
        for i in range(6):
            axis_val = msg.axes[i]
            # Apply deadzone and scaling
            if abs(axis_val) < 0.1:
                deltas.append(0.0)
            else:
                deltas.append(axis_val * self.increment_step)
        deltas.append(0.0)  # For the 7th joint, if applicable
        print("Joystick deltas:", deltas)
        # Skip publishing if there's no meaningful input
        if all(abs(d) < 1e-6 for d in deltas):
            return

        # Compute new joint positions
        self.current_positions = [
            curr + delta for curr, delta in zip(self.current_positions, deltas)
        ]

        # Create trajectory message
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.current_positions
        point.time_from_start.secs = 1  # 1 second in the future

        traj.points.append(point)

        # Publish the trajectory
        rospy.loginfo("Publishing joint trajectory: %s", self.current_positions)
        self.arm_pub.publish(traj)


def main():
    try:
        ArmJoyTrajectoryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
