import os
import time

import pybullet as p
import pybullet_data


def main():
    # Connect to PyBullet
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Set up the simulation
    p.setGravity(0, 0, -9.81)

    # Load ground plane
    p.loadURDF("plane.urdf")

    # Load the TiAGO robot
    # Make sure the path is correct relative to your working directory
    urdf_path = "tiago_rl/assets/tiago_pal_gripper.urdf"

    # Check if file exists
    if not os.path.exists(urdf_path):
        print(f"URDF file not found at: {urdf_path}")
        print("Current working directory:", os.getcwd())
        print("Please check the path to your URDF file")
        return

    # Load robot at a reasonable height above ground
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

    # Find controllable joints and create sliders
    jointIds = []
    jointNames = []
    jointRanges = []

    for i in range(numJoints):
        jointInfo = p.getJointInfo(robotId, i)
        jointName = jointInfo[1].decode("utf-8")
        jointType = jointInfo[2]

        # Only add revolute and prismatic joints
        if jointType == p.JOINT_REVOLUTE or jointType == p.JOINT_PRISMATIC:
            lowerLimit = jointInfo[8]
            upperLimit = jointInfo[9]

            # Skip joints with no limits or very small range
            if upperLimit > lowerLimit and (upperLimit - lowerLimit) > 0.01:
                jointIds.append(i)
                jointNames.append(jointName)
                jointRanges.append((lowerLimit, upperLimit))
                print(f"Joint {i}: {jointName}, Range: [{lowerLimit:.3f}, {upperLimit:.3f}]")

    # Create parameter sliders for controllable joints
    paramIds = []
    for i, (jointId, jointName, (lower, upper)) in enumerate(
        zip(jointIds, jointNames, jointRanges)
    ):
        # Limit the number of sliders to avoid GUI clutter
        if i < 20:  # Only show first 20 controllable joints
            paramId = p.addUserDebugParameter(
                jointName, lower, upper, (lower + upper) / 2  # Start at middle position
            )
            paramIds.append(paramId)
        else:
            paramIds.append(None)

    print(f"Created {len([p for p in paramIds if p is not None])} sliders")

    # Main simulation loop
    try:
        while True:
            # Read slider values and set joint positions
            for i, (jointId, paramId) in enumerate(zip(jointIds[: len(paramIds)], paramIds)):
                if paramId is not None:
                    targetPos = p.readUserDebugParameter(paramId)
                    p.setJointMotorControl2(
                        robotId,
                        jointId,
                        p.POSITION_CONTROL,
                        targetPosition=targetPos,
                        force=500,  # Adjust force as needed
                    )

            # Step simulation
            p.stepSimulation()
            time.sleep(1.0 / 240.0)  # 240 FPS

    except KeyboardInterrupt:
        print("Simulation stopped by user")
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()
