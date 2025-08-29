import pybullet as p
import time

# Connect to PyBullet in GUI mode
p.connect(p.GUI)

# Load the plane
p.setGravity(0, 0, -9.8)
plane_id = p.loadURDF("plane.urdf")

# Keep the simulation running for a few seconds to see the plane
time.sleep(3)

# Disconnect from PyBullet
p.disconnect()
