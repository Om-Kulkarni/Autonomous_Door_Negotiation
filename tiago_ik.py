import os
import time

import numpy as np
import pink
import pinocchio as pin
import pybullet as p
import pybullet_data
from pink import solve_ik
from pink.tasks import FrameTask, PostureTask


class TiagoIKController:
    def __init__(self, urdf_path):
        self.urdf_path = urdf_path
        self.robot_id = None
        self.configuration = None
        self.viz = None

        # Pink/Pinocchio setup
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = pin.Data(self.model)

        # Get joint limits
        self.q_min = self.model.lowerPositionLimit
        self.q_max = self.model.upperPositionLimit

        # Initialize configuration to neutral position
        self.q = pin.neutral(self.model)

        # Define important frame names (adjust based on your robot)
        self.end_effector_frame = "gripper_link"  # Adjust if needed
        self.head_frame = "head_2_link"  # Optional

        # Store joint name to index mapping
        self.joint_name_to_id = {
            name: i for i, name in enumerate(self.model.names[1:])  # Skip universe
        }

    def setup_pybullet(self):
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")

        start_pos = [0, 0, 0.1]
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robot_id = p.loadURDF(self.urdf_path, start_pos, start_orientation, useFixedBase=False)

        print(f"Loaded TiAGO robot with ID: {self.robot_id}")

        self.pb_joint_info = {}
        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode("utf-8")
            self.pb_joint_info[joint_name] = {
                "id": i,
                "type": joint_info[2],
                "lower_limit": joint_info[8],
                "upper_limit": joint_info[9],
            }

    def setup_ik_tasks(self):
        self.tasks = []

        # End effector task
        if self.end_effector_frame in [frame.name for frame in self.model.frames]:
            self.ee_task = FrameTask(
                self.end_effector_frame,
                position_cost=1.0,
                orientation_cost=1.0,
            )
            self.tasks.append(self.ee_task)
            print(f"Added end effector task for frame: {self.end_effector_frame}")

        # Head task (optional)
        if self.head_frame in [frame.name for frame in self.model.frames]:
            self.head_task = FrameTask(
                self.head_frame,
                position_cost=0.1,
                orientation_cost=0.5,
            )
            self.tasks.append(self.head_task)
            print(f"Added head task for frame: {self.head_frame}")

        # Posture task with target
        self.posture_task = PostureTask(cost=1e-3)
        self.posture_task.set_target(self.q)  # 💡 THIS FIXES THE ERROR
        self.tasks.append(self.posture_task)
        print("Added posture regularization task with neutral target posture")

    def setup_gui_controls(self):
        self.gui_params = {}

        self.gui_params["target_x"] = p.addUserDebugParameter("Target X", -2.0, 2.0, 0.5)
        self.gui_params["target_y"] = p.addUserDebugParameter("Target Y", -2.0, 2.0, 0.0)
        self.gui_params["target_z"] = p.addUserDebugParameter("Target Z", 0.0, 2.0, 0.5)

        self.gui_params["target_roll"] = p.addUserDebugParameter("Target Roll", -np.pi, np.pi, 0.0)
        self.gui_params["target_pitch"] = p.addUserDebugParameter(
            "Target Pitch", -np.pi, np.pi, 0.0
        )
        self.gui_params["target_yaw"] = p.addUserDebugParameter("Target Yaw", -np.pi, np.pi, 0.0)

        self.gui_params["head_pan"] = p.addUserDebugParameter("Head Pan", -1.5, 1.5, 0.0)
        self.gui_params["head_tilt"] = p.addUserDebugParameter("Head Tilt", -1.0, 1.0, 0.0)

        self.gui_params["ik_active"] = p.addUserDebugParameter("IK Active", 0, 1, 1)
        self.gui_params["dt"] = p.addUserDebugParameter("dt", 0.001, 0.1, 0.01)

    def update_ik_targets(self):
        if not hasattr(self, "gui_params"):
            return

        target_pos = np.array(
            [
                p.readUserDebugParameter(self.gui_params["target_x"]),
                p.readUserDebugParameter(self.gui_params["target_y"]),
                p.readUserDebugParameter(self.gui_params["target_z"]),
            ]
        )

        target_euler = np.array(
            [
                p.readUserDebugParameter(self.gui_params["target_roll"]),
                p.readUserDebugParameter(self.gui_params["target_pitch"]),
                p.readUserDebugParameter(self.gui_params["target_yaw"]),
            ]
        )

        target_rotation = pin.rpy.rpyToMatrix(*target_euler)
        target_transform = pin.SE3(target_rotation, target_pos)

        if hasattr(self, "ee_task"):
            print("EE task")
            self.ee_task.set_target(target_transform)

        if hasattr(self, "head_task"):
            print("Head task")
            head_pan = p.readUserDebugParameter(self.gui_params["head_pan"])
            head_tilt = p.readUserDebugParameter(self.gui_params["head_tilt"])
            head_rotation = pin.rpy.rpyToMatrix(0, head_tilt, head_pan)
            head_transform = pin.SE3(head_rotation, np.zeros(3))
            self.head_task.set_target(head_transform)
        # print('Target')
        # print(target_pos)
        # print(target_rotation)

    def solve_ik_step(self):
        dt = p.readUserDebugParameter(self.gui_params["dt"])

        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)

        self.configuration = pink.Configuration(self.model, self.data, self.q)

        try:
            self.posture_task.set_target(self.q)  # Optional: updates to current posture
            velocity = solve_ik(self.configuration, self.tasks, dt, solver="quadprog")
            # print(velocity)
            # print(self.tasks)
            self.q = pin.integrate(self.model, self.q, velocity * dt)
            self.q = np.clip(self.q, self.q_min, self.q_max)
            # print(self.q)
        except Exception as e:
            print(f"IK solve failed: {e}")
            return False

        return True

    def sync_to_pybullet(self):
        for i, q_val in enumerate(self.q):
            if i < len(self.model.names) - 1:
                joint_name = self.model.names[i + 1]
                if joint_name in self.pb_joint_info:
                    pb_joint_id = self.pb_joint_info[joint_name]["id"]
                    joint_type = self.pb_joint_info[joint_name]["type"]
                    if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
                        p.setJointMotorControl2(
                            self.robot_id,
                            pb_joint_id,
                            p.POSITION_CONTROL,
                            targetPosition=q_val,
                            force=500,
                        )

    def draw_target_frame(self):
        if not hasattr(self, "gui_params"):
            return

        target_pos = np.array(
            [
                p.readUserDebugParameter(self.gui_params["target_x"]),
                p.readUserDebugParameter(self.gui_params["target_y"]),
                p.readUserDebugParameter(self.gui_params["target_z"]),
            ]
        )

        target_euler = np.array(
            [
                p.readUserDebugParameter(self.gui_params["target_roll"]),
                p.readUserDebugParameter(self.gui_params["target_pitch"]),
                p.readUserDebugParameter(self.gui_params["target_yaw"]),
            ]
        )

        axis_length = 0.1
        line_width = 3

        x_end = target_pos + axis_length * np.array(
            [np.cos(target_euler[2]), np.sin(target_euler[2]), 0]
        )
        y_end = target_pos + axis_length * np.array(
            [-np.sin(target_euler[2]), np.cos(target_euler[2]), 0]
        )
        z_end = target_pos + axis_length * np.array([0, 0, 1])

        p.addUserDebugLine(target_pos, x_end, [1, 0, 0], lineWidth=line_width, lifeTime=0.1)
        p.addUserDebugLine(target_pos, y_end, [0, 1, 0], lineWidth=line_width, lifeTime=0.1)
        p.addUserDebugLine(target_pos, z_end, [0, 0, 1], lineWidth=line_width, lifeTime=0.1)

    def run_simulation(self):
        print("Starting IK-enabled simulation...")
        print("Use the GUI sliders to set target poses for the end effector")

        try:
            while True:
                self.update_ik_targets()

                ik_active = p.readUserDebugParameter(self.gui_params["ik_active"])

                if ik_active > 0.5:
                    if self.solve_ik_step():
                        self.sync_to_pybullet()

                self.draw_target_frame()
                p.stepSimulation()
                time.sleep(1.0 / 240.0)

        except KeyboardInterrupt:
            print("Simulation stopped by user")
        finally:
            p.disconnect()


def main():
    urdf_path = "tiago_rl/assets/tiago_pal_gripper.urdf"

    if not os.path.exists(urdf_path):
        print(f"URDF file not found at: {urdf_path}")
        print("Current working directory:", os.getcwd())
        return

    try:
        controller = TiagoIKController(urdf_path)
        controller.setup_pybullet()
        controller.setup_ik_tasks()
        controller.setup_gui_controls()
        controller.run_simulation()
        time.sleep(10)
    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    main()
