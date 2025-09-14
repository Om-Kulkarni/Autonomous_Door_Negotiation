import logging
import time
from dataclasses import asdict, dataclass
from pprint import pformat

import draccus
import pybullet as p
import pybullet_data

# Import teleoperator configurations
from lerobot.teleoperators import (
    Teleoperator,
    TeleoperatorConfig,
    make_teleoperator_from_config,
)

# Import utility functions
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import init_logging

# Import robot configurations (if needed for Tiago)


@dataclass
class TeleoperateConfig:
    teleop: TeleoperatorConfig
    fps: int = 60
    teleop_time_s: float | None = None


def map_leader_to_tiago(action, tiago_joint_limits):
    """
    Maps SO101 leader arm actions to Tiago robot arm joints.
    Excludes arm_3_joint and arm_5_joint, which are fixed.
    """
    mapped_action = {}
    for leader_joint, tiago_joint in [
        ("shoulder_pan.pos", "arm_1_joint"),
        ("shoulder_lift.pos", "arm_2_joint"),
        ("elbow_flex.pos", "arm_4_joint"),
        ("wrist_flex.pos", "arm_6_joint"),
        ("wrist_roll.pos", "arm_7_joint"),
        ("gripper.pos", "gripper_right_finger_joint"),
    ]:
        if leader_joint in action:
            # Unpack min_limit, max_limit, and joint_id
            min_limit, max_limit, joint_id = tiago_joint_limits[tiago_joint]
            if joint_id is not None:  # Skip fixed joints (e.g., arm_3_joint and arm_5_joint)
                mapped_action[tiago_joint] = (
                    action[leader_joint] / 100.0 * (max_limit - min_limit) + min_limit
                )
    return mapped_action


def teleop_loop(
    teleop: Teleoperator,
    robot_id: int,
    tiago_joint_limits,
    fps: int,
    duration: float | None = None,
):
    start = time.perf_counter()
    while True:
        loop_start = time.perf_counter()
        action = teleop.get_action()
        mapped_action = map_leader_to_tiago(action, tiago_joint_limits)

        for joint_name, target_pos in mapped_action.items():
            joint_id = tiago_joint_limits[joint_name][2]  # Get joint ID from limits
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

        if duration is not None and time.perf_counter() - start >= duration:
            return


@draccus.wrap()
def teleoperate(cfg: TeleoperateConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))

    # Connect to PyBullet
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    # Load Tiago robot
    urdf_path = "tiago_rl/assets/tiago_pal_gripper.urdf"
    start_pos = [0, 0, 0.1]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot_id = p.loadURDF(urdf_path, start_pos, start_orientation, useFixedBase=False)

    # Get Tiago joint limits
    tiago_joint_limits = {}
    for i in range(p.getNumJoints(robot_id)):
        joint_info = p.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode("utf-8")
        joint_type = joint_info[2]
        if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
            tiago_joint_limits[joint_name] = (joint_info[8], joint_info[9], i)

    # Add fixed joints
    tiago_joint_limits["arm_3_joint"] = (3.14, 3.14, None)
    tiago_joint_limits["arm_5_joint"] = (3.57, 3.57, None)

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
