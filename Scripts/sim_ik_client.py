#!/usr/bin/env python
"""
@file sim_ik_client.py
@brief Simulates Tiago in PyBullet and sends arm + torso joint positions via socket to the real robot.
@details
  - Uses PyBullet to simulate Tiago with GUI sliders for 6-DoF end-effector control.
  - Performs inverse kinematics to compute 7-DOF arm + torso joint values.
  - Sends the computed joint values over TCP socket using `pickle`.
"""

import pybullet as p
import pybullet_data
import time
import os
import numpy as np
import socket
import pickle
import pygame

# Define robot joint indices based on discovery
TORSO_INDEX = 21
ARM_INDICES = [31, 32, 33, 34, 35, 36, 37]  # arm_1 to arm_7 joints

# Set initial joint positions to match default positions
INITIAL_POSITIONS = {
    21: 0.30,    # torso_lift_joint
    31: 1.61,    # arm_1_joint
    32: -0.93,   # arm_2_joint
    33: -3.14,   # arm_3_joint
    34: 1.83,    # arm_4_joint
    35: -1.58,   # arm_5_joint
    36: -0.62,   # arm_6_joint
    37: -1.58    # arm_7_joint
}

# Controller Constants for DS3 on Linux
AXIS_LEFT_STICK_X = 0
AXIS_LEFT_STICK_Y = 1
AXIS_RIGHT_STICK_X = 3
AXIS_RIGHT_STICK_Y = 4
AXIS_L2 = 2
AXIS_R2 = 5

BUTTON_SELECT = 8
BUTTON_TRIANGLE = 2
BUTTON_CROSS = 0

# Socket config (match real robot IP and port)
SERVER_IP = "10.68.0.1"  # Replace with robot IP
SERVER_PORT = 65432
USE_REAL_ROBOT = False

def send_joint_command(torso, arm_joints,  lin_vel_x, ang_vel_z):
    """
    @brief Sends torso + arm joint commands to the robot over TCP socket.
    @param torso: float, torso lift height
    @param arm_joints: list of 7 floats
    @param lin_vel_x: float, base linear velocity in x
    @param ang_vel_z: float, base angular velocity around z
    """
    if not USE_REAL_ROBOT:
        return  # Skip socket communication

    message = [torso] + arm_joints + [lin_vel_x, ang_vel_z]

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((SERVER_IP, SERVER_PORT))
            s.sendall(pickle.dumps(message, protocol=2))  # Use protocol=2 for Python2 compatibility
    except Exception as e:
        print(f"⚠️ Socket send failed: {e}")

def main():
    # Initialize Pygame and the joystick
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"✅ Joystick found: {joystick.get_name()}")
    else:
        print("⚠️ No joystick found. Base velocities will be zero.")
        joystick = None


    # 1. Connect to PyBullet
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    # 2. Load Tiago robot
    urdf_path = "tiago_rl/assets/tiago_pal_gripper.urdf"
    if not os.path.exists(urdf_path):
        print(f"URDF file not found: {urdf_path}")
        return

    robotId = p.loadURDF(urdf_path, [0, 0, 0.1], p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)
    print("✅ Tiago loaded")

    # 3. Increase mass for stability
    for link_id in range(-1, p.getNumJoints(robotId)):
        m = p.getDynamicsInfo(robotId, link_id)[0]
        p.changeDynamics(robotId, link_id, mass=m * 5.0)

    for joint_id, pos in INITIAL_POSITIONS.items():
        p.resetJointState(robotId, joint_id, pos)

    print("✅ Set initial joint positions")

    # 5. Get joint information and create mappings
    numJoints = p.getNumJoints(robotId)
    jointIds = []
    jointNames = []
    ee_link_index = None

    for i in range(numJoints):
        info = p.getJointInfo(robotId, i)
        name = info[1].decode('utf-8')
        jointType = info[2]

        if name == "arm_tool_joint":
            ee_link_index = i
            print(f"Found end-effector: '{name}' at index {i}")

        if jointType in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
            jointIds.append(i)
            jointNames.append(name)

    if ee_link_index is None:
        print("⚠️ End-effector joint not found.")
        return

    # Create mapping from joint indices to IK solution indices
    torso_ik_index = None
    arm_ik_indices = []
    
    for ik_idx, joint_id in enumerate(jointIds):
        if joint_id == TORSO_INDEX:
            torso_ik_index = ik_idx
        elif joint_id in ARM_INDICES:
            arm_ik_indices.append((ik_idx, joint_id))
    
    # Sort arm indices to maintain order
    arm_ik_indices.sort(key=lambda x: x[1])  # Sort by joint_id
    
    print(f"✅ Torso IK index: {torso_ik_index}")
    print(f"✅ Arm IK indices: {arm_ik_indices}")

    # Calculate the actual end-effector pose from initial joint positions
    ee_state = p.getLinkState(robotId, ee_link_index)
    initial_ee_pos = list(ee_state[0])
    initial_ee_orn = ee_state[1]
    initial_euler = list(p.getEulerFromQuaternion(initial_ee_orn))

    print(f"Initial EE position: {initial_ee_pos}")
    print(f"Initial EE orientation (euler): {initial_euler}")

    # Create sliders with the actual initial end-effector pose
    # sliders = {
    #     'x': p.addUserDebugParameter("X", -1.0, 1.0, initial_ee_pos[0]),
    #     'y': p.addUserDebugParameter("Y", -1.0, 1.0, initial_ee_pos[1]),
    #     'z': p.addUserDebugParameter("Z", 0.2, 1.2, initial_ee_pos[2]),
    #     'roll': p.addUserDebugParameter("Roll", -3.14, 3.14, initial_euler[0]),
    #     'pitch': p.addUserDebugParameter("Pitch", -3.14, 3.14, initial_euler[1]),
    #     'yaw': p.addUserDebugParameter("Yaw", -3.14, 3.14, initial_euler[2]),
    #     # 'lin_vel_x': p.addUserDebugParameter("Base Lin Vel X", -0.2, 0.2, 0.0),
    #     # 'ang_vel_z': p.addUserDebugParameter("Base Ang Vel Z", -0.3, 0.3, 0.0),
    # }
    # print("✅ Sliders ready")

    # Initialize control state variables
    control_mode = 'BASE'  # Start in Base Control mode
    target_pos = initial_ee_pos
    target_euler = initial_euler
    print(f"✅ Starting in '{control_mode}' mode. Press SELECT to toggle.")

    marker_id = None
    dead_zone = 0.15

    try:
        while True:
            # Initialize velocities for this loop iteration
            lin_vel_x, ang_vel_z = 0.0, 0.0
            d_pos = [0, 0, 0]  # Change in position [dx, dy, dz]
            d_euler = [0, 0, 0] # Change in orientation [droll, dpitch, dyaw]

            # --- Process Pygame Events for Mode Switching ---
            if joystick:
                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN:
                        if event.button == BUTTON_SELECT:
                            control_mode = 'ARM' if control_mode == 'BASE' else 'BASE'
                            print(f"\n-- MODE SWITCH: {control_mode} CONTROL --")

            # --- Apply Control Logic Based on Current Mode ---
            if joystick:
                if control_mode == 'BASE':
                    # --- BASE CONTROL MODE ---
                    # Left Stick for base velocity
                    lin_vel_raw = -joystick.get_axis(AXIS_LEFT_STICK_Y)
                    if abs(lin_vel_raw) > dead_zone:
                        lin_vel_x = lin_vel_raw * 0.2

                    ang_vel_raw = -joystick.get_axis(AXIS_LEFT_STICK_X)
                    if abs(ang_vel_raw) > dead_zone:
                        ang_vel_z = ang_vel_raw * 0.3

                elif control_mode == 'ARM':
                    # --- ARM CONTROL MODE ---
                    # Base is stationary
                    lin_vel_x, ang_vel_z = 0.0, 0.0

                    # Left Stick for EE Pos (X, Y)
                    dx_raw = -joystick.get_axis(AXIS_LEFT_STICK_Y) # Fwd/Back
                    dy_raw = -joystick.get_axis(AXIS_LEFT_STICK_X) # Left/Right
                    if abs(dx_raw) > dead_zone: d_pos[0] = dx_raw
                    if abs(dy_raw) > dead_zone: d_pos[1] = dy_raw
                    
                    # Triangle/Cross for EE Pos (Z)
                    if joystick.get_button(BUTTON_TRIANGLE): d_pos[2] = 1.0  # Up
                    if joystick.get_button(BUTTON_CROSS): d_pos[2] = -1.0 # Down

                    # Right Stick for EE Orient (Pitch, Yaw)
                    dpitch_raw = joystick.get_axis(AXIS_RIGHT_STICK_Y) # Nod
                    dyaw_raw = joystick.get_axis(AXIS_RIGHT_STICK_X)   # Turn
                    if abs(dpitch_raw) > dead_zone: d_euler[1] = dpitch_raw
                    if abs(dyaw_raw) > dead_zone: d_euler[2] = dyaw_raw
                    
                    # L2/R2 for EE Orient (Roll) using analog axes
                    # Raw axis values are -1.0 (rest) to 1.0 (fully pressed).
                    # We convert them to a 0.0 to 1.0 pressure scale.
                    r2_pressure = (joystick.get_axis(AXIS_R2) + 1) / 2
                    l2_pressure = (joystick.get_axis(AXIS_L2) + 1) / 2
                    d_euler[0] = r2_pressure - l2_pressure # R2 adds, L2 subtracts

            # --- Update Target Pose Incrementally ---
            arm_speed = 0.005  # Position change speed
            rot_speed = 0.01   # Rotation change speed
            target_pos[0] += d_pos[0] * arm_speed
            target_pos[1] += d_pos[1] * arm_speed
            target_pos[2] += d_pos[2] * arm_speed
            target_euler[0] += d_euler[0] * rot_speed
            target_euler[1] += d_euler[1] * rot_speed
            target_euler[2] += d_euler[2] * rot_speed

            # Print status
            status_str = f"Mode: {control_mode} | LinVel: {lin_vel_x:.2f} | AngVel: {ang_vel_z:.2f}"
            print(status_str, end='\r')

            # --- IK and Simulation (largely unchanged) ---
            orn = p.getQuaternionFromEuler(target_euler)

            ik_solution = p.calculateInverseKinematics(
                robotId, ee_link_index, target_pos, targetOrientation=orn,
                maxNumIterations=100, residualThreshold=1e-4
            )

            torso = ik_solution[torso_ik_index] if torso_ik_index is not None and torso_ik_index < len(ik_solution) else 0.0

            arm_joint_positions = []
            for ik_idx, joint_id in arm_ik_indices:
                if ik_idx < len(ik_solution):
                    arm_joint_positions.append(ik_solution[ik_idx])
                else:
                    arm_joint_positions.append(0.0)

            if len(arm_joint_positions) != 7: continue

            if torso_ik_index is not None:
                p.setJointMotorControl2(robotId, TORSO_INDEX, p.POSITION_CONTROL, torso, force=1000)
            
            for i, (ik_idx, joint_id) in enumerate(arm_ik_indices):
                p.setJointMotorControl2(robotId, joint_id, p.POSITION_CONTROL, arm_joint_positions[i], force=1000)

            send_joint_command(torso, arm_joint_positions, lin_vel_x, ang_vel_z)

            if marker_id is not None: p.removeUserDebugItem(marker_id)
            z_axis = p.getMatrixFromQuaternion(orn)[6:]
            end_point = [target_pos[i] + z_axis[i] * 0.1 for i in range(3)]
            marker_id = p.addUserDebugLine(target_pos, end_point, [0, 0, 1], 3, 0.05)

            p.stepSimulation()
            time.sleep(1 / 240.)


    except KeyboardInterrupt:
        print("👋 Simulation terminated.")

    finally:
        p.disconnect()
        pygame.quit()

if __name__ == "__main__":
    main()