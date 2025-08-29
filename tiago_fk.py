import pybullet as p
import pybullet_data
import time
import os

def main():
    # Connect to PyBullet
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Set up the simulation
    p.setGravity(0, 0, -9.81)
    planeId = p.loadURDF("plane.urdf")
    
    # Load the TiAGO robot
    urdf_path = "tiago_rl/assets/tiago_pal_gripper.urdf"
    
    if not os.path.exists(urdf_path):
        print(f"URDF file not found at: {urdf_path}")
        print("Current working directory:", os.getcwd())
        return

    startPos = [0, 0, 0.1]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    
    try:
        robotId = p.loadURDF(urdf_path, startPos, startOrientation, useFixedBase=False)
        print(f"Successfully loaded TiAGO robot with ID: {robotId}")
    except Exception as e:
        print(f"Error loading URDF: {e}")
        return

    # Get joint information
    numJoints = p.getNumJoints(robotId)
    print(f"Robot has {numJoints} joints")

    jointIds = []
    jointNames = []
    jointRanges = []
    ee_link_index = None

    for i in range(numJoints):
        jointInfo = p.getJointInfo(robotId, i)
        jointName = jointInfo[1].decode('utf-8')
        jointType = jointInfo[2]

        if jointName == "gripper_joint":
            ee_link_index = i
            print(f"Found end-effector link 'gripper_link' at index {ee_link_index}")

        if jointType == p.JOINT_REVOLUTE or jointType == p.JOINT_PRISMATIC:
            lowerLimit = jointInfo[8]
            upperLimit = jointInfo[9]

            if upperLimit > lowerLimit and (upperLimit - lowerLimit) > 0.01:
                jointIds.append(i)
                jointNames.append(jointName)
                jointRanges.append((lowerLimit, upperLimit))
                print(f"Joint {i}: {jointName}, Range: [{lowerLimit:.3f}, {upperLimit:.3f}]")

    paramIds = []
    for i, (jointId, jointName, (lower, upper)) in enumerate(zip(jointIds, jointNames, jointRanges)):
        if i < 20:
            paramId = p.addUserDebugParameter(jointName, lower, upper, (lower + upper) / 2)
            paramIds.append(paramId)
        else:
            paramIds.append(None)

    print(f"Created {len([p for p in paramIds if p is not None])} sliders")

    if ee_link_index is None:
        print("⚠️ Could not find the end-effector link 'gripper_link'. Make sure the name is correct.")
        return

    ee_marker_id = None

    try:
        while True:
            # Set joint positions from sliders
            for i, (jointId, paramId) in enumerate(zip(jointIds, paramIds)):
                if paramId is not None:
                    targetPos = p.readUserDebugParameter(paramId)
                    p.setJointMotorControl2(
                        robotId,
                        jointId,
                        p.POSITION_CONTROL,
                        targetPosition=targetPos,
                        force=500
                    )

            # Step simulation
            p.stepSimulation()
            time.sleep(1. / 240.)

            # --- Get and print end-effector position ---
            link_state = p.getLinkState(robotId, ee_link_index)
            ee_position = link_state[4]  # worldLinkFramePosition
            ee_orientation = link_state[5]  # worldLinkFrameOrientation

            print(f"End-effector position (x, y, z): {ee_position}")

            # Optional: Draw a green sphere at the end-effector
            if ee_marker_id is not None:
                p.removeUserDebugItem(ee_marker_id)
            ee_marker_id = p.addUserDebugLine(
                ee_position, 
                [ee_position[0], ee_position[1], ee_position[2] + 0.1],
                [0, 1, 0],  # green
                lineWidth=5,
                lifeTime=0.05
            )

    except KeyboardInterrupt:
        print("Simulation stopped by user")
    finally:
        p.disconnect()

if __name__ == "__main__":
    main()
