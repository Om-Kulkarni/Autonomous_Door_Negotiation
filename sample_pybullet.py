import pybullet as p
import time

# Start PyBullet with GUI

physicalClient = p.connect(p.GUI)#, options="--opengl2")

# Optional: load a plane
planeId = p.loadURDF("plane.urdf")

# Load a simple URDF (e.g., r2d2)
robotId = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 1])

# Set gravity
p.setGravity(0, 0, -9.8)

# Simulate for 5 seconds
for i in range(240):  # ~5 seconds at 48Hz
    p.stepSimulation()
    time.sleep(1.0 / 48)

# Disconnect when done
p.disconnect()
