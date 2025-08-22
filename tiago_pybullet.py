import pybullet as p
import pybullet_data
import time
import os

# Connect to PyBullet GUI
physicsClient = p.connect(p.DIRECT)  # Use GUI for debugging; switch to DIRECT for headless
p.setGravity(0, 0, -9.81)

# Optional: Add search path for default PyBullet URDFs
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load a plane
planeId = p.loadURDF("plane.urdf")

# Load your URDF
urdf_path = "tiago_rl/assets/tiago_pal_gripper.urdf"

# Make sure it's an absolute path
urdf_path = os.path.abspath(urdf_path)

# Load the robot
start_pos = [0, 0, 0.1]
start_orientation = p.getQuaternionFromEuler([0, 0, 0])
robot_id = p.loadURDF(urdf_path, start_pos, start_orientation, useFixedBase=True)

# Create sliders for all joints
num_joints = p.getNumJoints(robot_id)
slider_ids = []

print("Creating sliders for joints:")
for joint_index in range(num_joints):
    joint_info = p.getJointInfo(robot_id, joint_index)
    joint_name = joint_info[1].decode('utf-8')
    joint_lower_limit = joint_info[8]
    joint_upper_limit = joint_info[9]
    
    if joint_lower_limit > joint_upper_limit:
        # Infinite or continuous joint, give it a range
        joint_lower_limit = -3.14
        joint_upper_limit = 3.14

    # Create slider and handle potential errors
    try:
        slider = p.addUserDebugParameter(joint_name, joint_lower_limit, joint_upper_limit, 0)
        slider_ids.append(slider)
        print(f"  {joint_name}: range=({joint_lower_limit}, {joint_upper_limit})")
    except Exception as e:
        print(f"Failed to create slider for joint {joint_name}: {e}")

# Simulation loop
while True:
    p.stepSimulation()
    
    for i in range(len(slider_ids)):  # Ensure we only iterate over valid sliders
        try:
            target_pos = p.readUserDebugParameter(slider_ids[i])
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=target_pos)
        except Exception as e:
            print(f"Failed to read or set parameter for joint {i}: {e}")

    time.sleep(1./240.)