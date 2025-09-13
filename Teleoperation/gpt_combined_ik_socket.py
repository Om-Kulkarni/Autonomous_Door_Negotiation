import socket

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

THRESHOLD = 0.01


def send_ik_goal(x, y, z, client):
    roll = pitch = yaw = 0.0
    q = quaternion_from_euler(roll, pitch, yaw)

    pose = PoseStamped()
    pose.header.frame_id = "base_footprint"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]

    req = MotionPlanRequest()
    req.group_name = "arm_torso"
    req.num_planning_attempts = 5
    req.allowed_planning_time = 5.0

    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = pose.header.frame_id
    position_constraint.link_name = "arm_tool_link"
    position_constraint.target_point_offset.x = 0.0
    position_constraint.target_point_offset.y = 0.0
    position_constraint.target_point_offset.z = 0.0

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [0.01, 0.01, 0.01]
    bounding_volume = BoundingVolume()
    bounding_volume.primitives.append(box)
    bounding_volume.primitive_poses.append(pose.pose)

    position_constraint.constraint_region = bounding_volume
    position_constraint.weight = 1.0

    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = pose.header.frame_id
    orientation_constraint.link_name = "arm_tool_link"
    orientation_constraint.orientation = pose.pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = 0.1
    orientation_constraint.absolute_y_axis_tolerance = 0.1
    orientation_constraint.absolute_z_axis_tolerance = 0.1
    orientation_constraint.weight = 1.0

    goal_constraints = Constraints()
    goal_constraints.position_constraints.append(position_constraint)
    goal_constraints.orientation_constraints.append(orientation_constraint)

    req.goal_constraints.append(goal_constraints)

    goal = MoveGroupGoal()
    goal.request = req
    goal.planning_options.plan_only = False
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


def main():
    rospy.init_node("socket_ik_controller_py2")

    # Set up socket server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(("0.0.0.0", 9999))
    server_socket.listen(1)

    rospy.loginfo("Socket server listening on 0.0.0.0:9999")

    # Connect to MoveIt move_group action server
    client = actionlib.SimpleActionClient("move_group", MoveGroupAction)
    rospy.loginfo("Waiting for move_group action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_group action server.")

    conn, addr = server_socket.accept()
    rospy.loginfo("Client connected from %s", str(addr))
    prevx, prevy, prevz = 0.0, 0.0, 0.0
    while not rospy.is_shutdown():
        print("start")
        try:
            data = conn.recv(1024)
            if not data:
                rospy.loginfo("Client disconnected.")
                break

            data = data.strip()
            rospy.loginfo("Received: %s", data)

            parts = data.split(",")
            if len(parts) != 3:
                rospy.logwarn("Invalid input. Expected 'x,y,z'")
                continue

            try:
                x = float(parts[0])
                y = float(parts[1])
                z = float(parts[2])
            except ValueError:
                rospy.logwarn("Non-float values received.")
                continue
            x, y, z = round(x, 2), round(y, 2), round(z, 2)
            if (
                abs(x - prevx) < THRESHOLD
                and abs(y - prevy) < THRESHOLD
                and abs(z - prevz) < THRESHOLD
            ):
                rospy.loginfo("No significant change in coordinates, skipping.")
                continue

            rospy.loginfo(f"Processed coordinates: {x:.2f}, {y:.2f}, {z:.2f}")
            send_ik_goal(x, y, z, client)
            prevx, prevy, prevz = x, y, z
            conn.sendall("Move command received\n")

        except Exception as e:
            rospy.logerr("Error: %s", str(e))
            break
        print("end")
    conn.close()
    server_socket.close()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
