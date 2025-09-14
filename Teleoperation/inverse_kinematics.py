import sys

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    MoveGroupAction,
    MoveGroupGoal,
    OrientationConstraint,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node("plan_arm_torso_ik_low_level")

    # Parse args (x y z roll pitch yaw)
    if len(sys.argv) < 7:
        rospy.loginfo("Usage: rosrun your_package your_script.py x y z roll pitch yaw")
        return

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])
    roll = float(sys.argv[4])
    pitch = float(sys.argv[5])
    yaw = float(sys.argv[6])

    # Create quaternion from RPY
    q = quaternion_from_euler(roll, pitch, yaw)

    # Define target pose
    pose = PoseStamped()
    pose.header.frame_id = "base_footprint"  # Update if needed
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]

    # Set up MoveGroup action client
    client = actionlib.SimpleActionClient("move_group", MoveGroupAction)
    rospy.loginfo("Waiting for move_group action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_group action server")

    # Build motion plan request
    req = MotionPlanRequest()
    req.group_name = "arm_torso"
    req.num_planning_attempts = 5
    req.allowed_planning_time = 5.0

    # ========== Build Goal Constraints from Pose ==========
    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = pose.header.frame_id
    position_constraint.link_name = "arm_tool_link"
    position_constraint.target_point_offset.x = 0.0
    position_constraint.target_point_offset.y = 0.0
    position_constraint.target_point_offset.z = 0.0

    # Small box around goal pose
    bounding_volume = BoundingVolume()
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [0.01, 0.01, 0.01]  # 1 cm cube
    bounding_volume.primitives.append(box)
    bounding_volume.primitive_poses.append(pose.pose)
    position_constraint.constraint_region = bounding_volume
    position_constraint.weight = 1.0

    # Orientation constraint
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = pose.header.frame_id
    orientation_constraint.link_name = "arm_tool_link"
    orientation_constraint.orientation = pose.pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = 0.1
    orientation_constraint.absolute_y_axis_tolerance = 0.1
    orientation_constraint.absolute_z_axis_tolerance = 0.1
    orientation_constraint.weight = 1.0

    # Full constraints message
    goal_constraints = Constraints()
    goal_constraints.position_constraints.append(position_constraint)
    goal_constraints.orientation_constraints.append(orientation_constraint)

    # Attach to request
    req.goal_constraints.append(goal_constraints)

    # Build MoveGroupGoal
    goal = MoveGroupGoal()
    goal.request = req
    goal.planning_options.plan_only = False  # Set True if you only want planning
    goal.planning_options.replan = True
    goal.planning_options.replan_attempts = 2

    rospy.loginfo("Sending goal to move_group...")
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()

    if result:
        rospy.loginfo("Motion plan succeeded.")
    else:
        rospy.logerr("Motion plan failed.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
