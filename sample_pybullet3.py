import time
import pybullet as p

def main():
    # Connect to PyBullet and set the environment
    physics_client = p.connect(p.GUI)  # Connect to the PyBullet GUI
    p.setGravity(0, 0, -9.8)  # Set gravity

    # Load the plane URDF
    planeId = p.loadURDF("plane.urdf")  # Default plane in PyBullet

    # Load your custom URDF (Tiago Pal Gripper)
    urdf_path = "tiago_rl/assets/tiago_pal_gripper.urdf"
    tiagoId = p.loadURDF(urdf_path, basePosition=[0, 0, 0.1])  # Place at (0, 0, 0.1)

    # Optionally, you can set a joint control for the gripper or robot, if needed.

    try:
        # Run the simulation loop
        while True:
            p.stepSimulation()  # Step the simulation
            time.sleep(1.0 / 240.0)  # Sleep to simulate real-time physics

    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt caught: stopping simulation.")

    except Exception as e:
        # If any other unexpected exception happens
        print(f"[ERROR] Unexpected exception: {e}")
        # Optionally re-raise or handle it

    finally:
        # Always executed: cleaning up
        print("[INFO] Disconnecting from PyBullet.")
        p.disconnect()

if __name__ == "__main__":
    main()
