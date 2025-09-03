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

# Define robot joint indices based on discovery
TORSO_INDEX = 21
ARM_INDICES = [31, 32, 33, 34, 35, 36, 37]  # arm_1 to arm_7 joints

# Define default start positions for torso and arm (from image)
DEFAULT_POSITIONS = {
    21: 0.1372,  # torso_lift_joint
    31: -0.3571,  # arm_1_joint
    32: -0.6874,  # arm_2_joint
    33: 0.5230,   # arm_3_joint
    34: 0.6508,   # arm_4_joint
    35: -0.8439,  # arm_5_joint
    36: 0.5752,   # arm_6_joint
    37: 0.0       # arm_7_joint
}

# Socket config (match real robot IP and port)
SERVER_IP = "10.68.0.1"  # Replace with robot IP
SERVER_PORT = 65432

def send_joint_command(torso, arm_joints):
    """
    @brief Sends torso + arm joint commands to the robot over TCP socket.
    @param torso: float, torso lift height
    @param arm_joints: list of 7 floats
    """
    message = [torso] + arm_joints
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((SERVER_IP, SERVER_PORT))
            s.sendall(pickle.dumps(message, protocol=2))  # Use protocol=2 for Python2 compatibility
    except Exception as e:
        print(f"⚠️ Socket send failed: {e}")

def main():
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

    # 4. Set initial joint positions (torso + arm)
    for joint_id, pos in DEFAULT_POSITIONS.items():
        p.resetJointState(robotId, joint_id, pos)

    # 5. Get joint information and create mappings
    numJoints = p.getNumJoints(robotId)
    jointIds = []
    jointNames = []
    ee_link_index = None

    for i in range(numJoints):
        info = p.getJointInfo(robotId, i)
        name = info[1].decode('utf-8')
        jointType = info[2]

        if name == "gripper_left_finger_joint":
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

    # 6. Create GUI sliders
    sliders = {
        'x': p.addUserDebugParameter("X", -1.0, 1.0, 0.5),
        'y': p.addUserDebugParameter("Y", -1.0, 1.0, -0.6),
        'z': p.addUserDebugParameter("Z", 0.2, 1.2, 1.0),
        'roll': p.addUserDebugParameter("Roll", -3.14, 3.14, 0.0),
        'pitch': p.addUserDebugParameter("Pitch", -3.14, 3.14, 0.0),
        'yaw': p.addUserDebugParameter("Yaw", -3.14, 3.14, 0.0),
    }

    print("✅ Sliders ready")

    marker_id = None

    try:
        while True:
            # 7. Read sliders
            tx = p.readUserDebugParameter(sliders['x'])
            ty = p.readUserDebugParameter(sliders['y'])
            tz = p.readUserDebugParameter(sliders['z'])
            r = p.readUserDebugParameter(sliders['roll'])
            p_ = p.readUserDebugParameter(sliders['pitch'])
            y_ = p.readUserDebugParameter(sliders['yaw'])

            pos = [tx, ty, tz]
            orn = p.getQuaternionFromEuler([r, p_, y_])

            # 8. Compute IK
            ik_solution = p.calculateInverseKinematics(
                robotId,
                ee_link_index,
                pos,
                targetOrientation=orn,
                maxNumIterations=500,
                residualThreshold=1e-5
            )

            # 9. Extract desired joint positions using correct mapping
            torso = ik_solution[torso_ik_index] if torso_ik_index is not None and torso_ik_index < len(ik_solution) else 0.0
            
            arm_joint_positions = []
            for ik_idx, joint_id in arm_ik_indices:
                if ik_idx < len(ik_solution):
                    arm_joint_positions.append(ik_solution[ik_idx])
                else:
                    arm_joint_positions.append(0.0)  # fallback

            print(f"IK solution length: {len(ik_solution)}")
            print(f"Torso position: {torso}")
            print(f"Arm joint positions: {arm_joint_positions}")

            if len(arm_joint_positions) != 7:
                print("⚠️ IK output does not contain all arm joints.")
                continue

            # 10. Apply joints in simulation
            if torso_ik_index is not None:
                p.setJointMotorControl2(robotId, TORSO_INDEX, p.POSITION_CONTROL, torso, force=1000)
            
            for i, (ik_idx, joint_id) in enumerate(arm_ik_indices):
                p.setJointMotorControl2(robotId, joint_id, p.POSITION_CONTROL, arm_joint_positions[i], force=1000)

            # 11. Send to real robot
            send_joint_command(torso, arm_joint_positions)

            # 12. Visual debug marker
            if marker_id is not None:
                p.removeUserDebugItem(marker_id)

            z_axis = p.getMatrixFromQuaternion(orn)[6:]
            end_point = [pos[i] + z_axis[i] * 0.1 for i in range(3)]
            marker_id = p.addUserDebugLine(pos, end_point, [0, 0, 1], 3, 0.05)

            # 13. Step simulation
            p.stepSimulation()
            time.sleep(1 / 240.)

    except KeyboardInterrupt:
        print("👋 Simulation terminated.")

    finally:
        p.disconnect()

if __name__ == "__main__":
    main()