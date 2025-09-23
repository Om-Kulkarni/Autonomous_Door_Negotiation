import os
import time

import pybullet as p
import pybullet_data


def main():
    # Connect to PyBullet
    p.connect(p.GUI) # Use p.DIRECT if you don’t want graphics.
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Set up the simulation
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf") # adds an infinite ground plane at z=0 so things don’t fall forever.

    # Load the TiAGO robot
    urdf_path = "tiago_rl/assets/tiago_pal_gripper.urdf"

    if not os.path.exists(urdf_path):
        print(f"URDF file not found at: {urdf_path}")
        print("Current working directory:", os.getcwd())
        return

    startPos = [0, 0, 0.1] # starting pos of what? - of the robot, just slightly above the ground, so that it doesn't intersect the plane.
    startOrientation = p.getQuaternionFromEuler([0, 0, 0]) # start orientation of what? - of the robot, and rpy has been set to .

    try:
        robotId = p.loadURDF(urdf_path, startPos, startOrientation, useFixedBase=False) # what does fixing and not fixing base do? - enables / disables movement of robot (terrestrially).
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
        jointInfo = p.getJointInfo(robotId, i) # what all informations does the getJoingtInfo get us?
        jointName = jointInfo[1].decode("utf-8")
        jointType = jointInfo[2]

        if jointName == "gripper_joint":
            ee_link_index = i # I don't understand what's happening in here, please explain !!!
            print(f"Found end-effector link 'gripper_link' at index {ee_link_index}")

        if jointType == p.JOINT_REVOLUTE or jointType == p.JOINT_PRISMATIC:
            lowerLimit = jointInfo[8] # Getting back to what all infos the getJointInfo function gets us...
            upperLimit = jointInfo[9]

            if upperLimit > lowerLimit and (upperLimit - lowerLimit) > 0.01:
                jointIds.append(i)
                jointNames.append(jointName)
                jointRanges.append((lowerLimit, upperLimit))
                print(f"Joint {i}: {jointName}, Range: [{lowerLimit:.3f}, {upperLimit:.3f}]")

    # Build sliders for joints
    paramIds = []
    for i, (jointId, jointName, (lower, upper)) in enumerate(
        zip(jointIds, jointNames, jointRanges)
    ):
        if i < 20:
            paramId = p.addUserDebugParameter(jointName, lower, upper, (lower + upper) / 2) # start at the middle of the range for each joint.
            paramIds.append(paramId)
        else:
            paramIds.append(None)

    print(f"Created {len([p for p in paramIds if p is not None])} sliders")

    ### Here we're checking, jointName == "gripper_joint" but print about 'gripper_link'. Check URDF to confirm names.
    if ee_link_index is None:
        print(
            "⚠️ Could not find the end-effector link 'gripper_link'. Make sure the name is correct."
        )
        return

    ee_marker_id = None

    try:
        while True:
            # Set joint positions from sliders
            for i, (jointId, paramId) in enumerate(zip(jointIds, paramIds)):
                if paramId is not None:
                    targetPos = p.readUserDebugParameter(paramId) # read the value of the slider
                    p.setJointMotorControl2( # set the joint to the value of the slider
                        robotId,
                        jointId,
                        p.POSITION_CONTROL,
                        targetPosition=targetPos,
                        force=500, # max force or torque that can be applied to the joint
                    )

            # Step simulation
            p.stepSimulation()
            time.sleep(1.0 / 240.0) # 240 Hz simulation (PyBullet default)

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
                lifeTime=0.05,
            )

    except KeyboardInterrupt:
        print("Simulation stopped by user")
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()
