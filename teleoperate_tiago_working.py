# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Simple script to directly control a simulated Tiago robot arm using a
teleoperator (e.g., SO101 leader).
"""

import logging
import time
from dataclasses import asdict, dataclass
from pprint import pformat

import draccus
import pybullet as p
import pybullet_data

from lerobot.teleoperators import (
    Teleoperator,
    TeleoperatorConfig,
    make_teleoperator_from_config,
    so101_leader,
)
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import init_logging, move_cursor_up


@dataclass
class TiagoTeleopConfig:
    teleop: TeleoperatorConfig
    # Limit the maximum frames per second.
    fps: int = 60
    teleop_time_s: float | None = None


# def map_leader_to_tiago(action, tiago_joint_limits):
#     """
#     Maps SO101 leader arm actions to Tiago robot arm joints.
#     Excludes arm_3_joint and arm_5_joint, which are fixed.
#     """
#     mapped_action = {}
#     for leader_joint, tiago_joint in [
#         ("shoulder_pan.pos", "arm_1_joint"),
#         ("shoulder_lift.pos", "arm_2_joint"),
#         ("elbow_flex.pos", "arm_4_joint"),
#         ("wrist_flex.pos", "arm_6_joint"),
#         ("wrist_roll.pos", "arm_7_joint"),
#         ("gripper.pos", "gripper_right_finger_joint"),
#     ]:
#         if leader_joint in action:
#             # Unpack min_limit, max_limit, and joint_id
#             min_limit, max_limit, joint_id = tiago_joint_limits[tiago_joint]
#             if joint_id is not None:
#                 # Scale the leader's action (which is a percentage) to the Tiago's joint range
#                 mapped_action[tiago_joint] = (
#                     action[leader_joint] / 100.0 * (max_limit - min_limit) + min_limit
#                 )
#     return mapped_action


def map_leader_to_tiago(action, tiago_joint_limits):
    """
    Maps SO101 leader arm actions to Tiago robot arm joints with proper phase alignment.
    Excludes arm_3_joint and arm_5_joint, which are fixed.
    """
    mapped_action = {}
    
    # Define mapping with potential adjustments for each joint
    # (leader_joint, tiago_joint, invert, offset)
    mappings = [
        ("shoulder_pan.pos", "arm_1_joint", True, -1.5),      # Invert shoulder pan
        ("shoulder_lift.pos", "arm_2_joint", True, 0.8),    # Direct mapping
        ("elbow_flex.pos", "arm_4_joint", False, 1.57),        # Invert elbow flex
        ("wrist_flex.pos", "arm_6_joint", True, 0),        # Invert wrist flex
        ("wrist_roll.pos", "arm_7_joint", True, 0),       # Direct mapping
        ("gripper.pos", "gripper_right_finger_joint", False, 0),  # Direct mapping
    ]
    
    for leader_joint, tiago_joint, invert, offset in mappings:
        if leader_joint in action:
            # Unpack min_limit, max_limit, and joint_id
            min_limit, max_limit, joint_id = tiago_joint_limits[tiago_joint]
            if joint_id is not None:  # Skip fixed joints
                # Normalize the leader value from 0-100 to 0-1
                normalized_value = action[leader_joint] / 100.0
                
                # Invert if needed (0 becomes 1, 1 becomes 0)
                if invert:
                    normalized_value = 1.0 - normalized_value
                
                # Scale to Tiago joint range and add offset
                mapped_action[tiago_joint] = (
                    normalized_value * (max_limit - min_limit) + min_limit + offset
                )
    
    return mapped_action

def teleop_loop(
    teleop: Teleoperator, robot_id: int, tiago_joint_limits, fps: int, duration: float | None = None
):
    """
    Continuously gets actions from the teleoperator and applies them to the Tiago robot.
    """
    display_len = max(len(key) for key in tiago_joint_limits.keys() if tiago_joint_limits[key][2] is not None)
    start = time.perf_counter()
    while True:
        loop_start = time.perf_counter()

        # Get the latest action from the teleoperator
        action = teleop.get_action()

        # Map the action to the Tiago's joint limits
        mapped_action = map_leader_to_tiago(action, tiago_joint_limits)

        # Apply the mapped positions to the robot's joints
        for joint_name, target_pos in mapped_action.items():
            joint_id = tiago_joint_limits[joint_name][2]
            p.setJointMotorControl2(
                robot_id,
                joint_id,
                p.POSITION_CONTROL,
                targetPosition=target_pos,
                force=500,
            )

        p.stepSimulation()
        dt_s = time.perf_counter() - loop_start
        busy_wait(1 / fps - dt_s)

        loop_s = time.perf_counter() - loop_start

        # Get current joint positions from PyBullet
        current_positions = {}
        for joint_name, (_, _, joint_id) in tiago_joint_limits.items():
            if joint_id is not None:
                joint_state = p.getJointState(robot_id, joint_id)
                current_positions[joint_name] = joint_state[0]  # Position is the first element

        print("\n" + "-" * (display_len + 35))  # Wider separator for both columns
        print(f"{'NAME':<{display_len}} | {'COMMANDED':>15} | {'ACTUAL':>15}")
        for joint_name in mapped_action.keys():
            commanded = mapped_action.get(joint_name, 0.0)
            actual = current_positions.get(joint_name, 0.0)
            print(f"{joint_name:<{display_len}} | {commanded:>15.4f} | {actual:>15.4f}")
        print(f"\ntime: {loop_s * 1e3:.2f}ms ({1 / loop_s:.0f} Hz)")

        if duration is not None and time.perf_counter() - start >= duration:
            return

        move_cursor_up(len(mapped_action) + 5)


@draccus.wrap()
def teleoperate(cfg: TiagoTeleopConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))

    # Connect to the PyBullet physics server in GUI mode
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    # Load the Tiago robot model
    urdf_path = "tiago_rl/assets/tiago_pal_gripper.urdf"
    start_pos = [0, 0, 0.1]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot_id = p.loadURDF(urdf_path, start_pos, start_orientation, useFixedBase=False)

    # Get joint limits and IDs for the Tiago robot
    tiago_joint_limits = {}
    for i in range(p.getNumJoints(robot_id)):
        joint_info = p.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode("utf-8")
        joint_type = joint_info[2]
        if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
            tiago_joint_limits[joint_name] = (joint_info[8], joint_info[9], i)
            
    # Add fixed joints with their correct values
    tiago_joint_limits["arm_3_joint"] = (3.14, 3.14, None)
    tiago_joint_limits["arm_5_joint"] = (0.0, 0.0, None)

    # Define the SO101 leader configuration
    teleop = make_teleoperator_from_config(cfg.teleop)
    teleop.connect()

    try:
        teleop_loop(teleop, robot_id, tiago_joint_limits, cfg.fps, duration=cfg.teleop_time_s)
    except KeyboardInterrupt:
        pass
    finally:
        teleop.disconnect()
        p.disconnect()


def main():
    teleoperate()


if __name__ == "__main__":
    main()