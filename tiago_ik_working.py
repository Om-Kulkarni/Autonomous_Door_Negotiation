import pybullet as p
import pybullet_data
import time
import os
import numpy as np
import socket
import pickle


def main():
    # 1. Connect to PyBullet
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    # 2. Load TiAGO robot
    urdf_path = "tiago_rl/assets/tiago_pal_gripper.urdf"
    if not os.path.exists(urdf_path):
        print(f"URDF file not found: {urdf_path}")
        print("Current working directory:", os.getcwd())
        return

    startPos = [0, 0, 0.1]
    startOrn = p.getQuaternionFromEuler([0, 0, 0])
    robotId = p.loadURDF(urdf_path, startPos, startOrn, useFixedBase=True)  # Base is static
    print(f"Loaded TiAGO robot with ID: {robotId}")

    # Increase mass for all links
    mass_factor = 5.0  # Adjust as needed
    num_links = p.getNumJoints(robotId) + 1
    for link_id in range(-1, num_links - 1):
        dynamics_info = p.getDynamicsInfo(robotId, link_id)
        original_mass = dynamics_info[0]
        new_mass = original_mass * mass_factor
        p.changeDynamics(robotId, link_id, mass=new_mass)
        print(f"Link {link_id}: Mass increased from {original_mass:.2f} to {new_mass:.2f}")
    print(f"✅ Robot mass increased by factor of {mass_factor}")

    # 3. Get joint and end-effector info
    numJoints = p.getNumJoints(robotId)
    print(f"Robot has {numJoints} joints")

    jointIds = []
    jointNames = []
    ee_link_index = None

    for i in range(numJoints):
        info = p.getJointInfo(robotId, i)
        name = info[1].decode('utf-8')
        jointType = info[2]

        # Update this to match your URDF (e.g., "gripper_left_finger_joint" or "gripper_right_finger_joint")
        if name == "gripper_left_finger_joint":  # Changed from "gripper_joint"
            ee_link_index = i
            print(f"Found end-effector: '{name}' at index {i}")

        if jointType in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
            jointIds.append(i)
            jointNames.append(name)
            print(f"Controllable joint {i}: {name}")

    if ee_link_index is None:
        print("⚠️ End-effector link not found.")
        print("Available joints:")
        for i in range(numJoints):
            info = p.getJointInfo(robotId, i)
            name = info[1].decode('utf-8')
            print(f"  {i}: {name}")
        print("Update the script with the correct end-effector joint name.")
        return

    # 4. Create GUI sliders for target pose
    gui_params = {
        'target_x': p.addUserDebugParameter("Target X", -1.0, 1.0, 0.5),
        'target_y': p.addUserDebugParameter("Target Y", -1.0, 1.0, -0.6),
        'target_z': p.addUserDebugParameter("Target Z", 0.2, 1.2, 1.0),  # Adjusted range for better reachability
        'roll': p.addUserDebugParameter("Roll", -3.14, 3.14, 0.0),
        'pitch': p.addUserDebugParameter("Pitch", -3.14, 3.14, 0.0),
        'yaw': p.addUserDebugParameter("Yaw", -3.14, 3.14, 0.0)
    }

    print("✅ Sliders created for 6-DoF end-effector control.")

    marker_id = None

    try:
        while True:
         
            # 5. Read target position and orientation
            tx = p.readUserDebugParameter(gui_params['target_x'])
            ty = p.readUserDebugParameter(gui_params['target_y'])
            tz = p.readUserDebugParameter(gui_params['target_z'])
            roll = p.readUserDebugParameter(gui_params['roll'])
            pitch = p.readUserDebugParameter(gui_params['pitch'])
            yaw = p.readUserDebugParameter(gui_params['yaw'])

            target_pos = [tx, ty, tz]
            print(target_pos)
            target_orn = p.getQuaternionFromEuler([roll, pitch, yaw])

            # 6. Inverse Kinematics (increased iterations and lowered threshold for better convergence)
            ik_solution = p.calculateInverseKinematics(
                robotId,
                ee_link_index,
                target_pos,
                targetOrientation=target_orn,
                maxNumIterations=500,  # Increased from 200
                residualThreshold=1e-5  # Lowered from 1e-4
            )
            #print(f'IK Solution:{ik_solution}')
            #print(f'Joint Names:{jointNames}')
            #print(f'Joints : {jointIds}')
            # 7. Apply IK joint values to robot (increased force)
            for i, jointId in enumerate(jointIds):
                if i < len(ik_solution):
                    p.setJointMotorControl2(
                        robotId,
                        jointId,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=ik_solution[i],
                        force=1000  # Increased from 500
                    )
            print(f'Joint Nammes : {jointNames[-9:-2]}')
            print(ik_solution[-9:-2])
            # 8. Debug: Print current end-effector position vs target
            ee_state = p.getLinkState(robotId, ee_link_index)
            current_pos = ee_state[0]
            current_orn = ee_state[1]
            print(f"Target: {target_pos}, Current: {[round(p, 3) for p in current_pos]}, Error: {round(np.linalg.norm(np.array(target_pos) - np.array(current_pos)), 3)}")

            # 9. Visual marker for target
            if marker_id is not None:
                p.removeUserDebugItem(marker_id)

            dir_vec = [0, 0, 0.1]
            orn_mat = p.getMatrixFromQuaternion(target_orn)
            z_axis = [orn_mat[6], orn_mat[7], orn_mat[8]]
            end_pt = [target_pos[i] + z_axis[i] * 0.1 for i in range(3)]

            marker_id = p.addUserDebugLine(
                target_pos, end_pt, [0, 0, 1], lineWidth=4, lifeTime=0.05
            )

            # 10. Step simulation
            p.stepSimulation()
            time.sleep(1. / 240.)

    except KeyboardInterrupt:
        print("Simulation interrupted by user.")
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()