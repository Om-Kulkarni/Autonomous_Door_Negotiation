import logging
import time
from dataclasses import asdict, dataclass
from pprint import pformat

import draccus
from lerobot.teleoperators import (  # noqa: F401
    Teleoperator,
    TeleoperatorConfig,
    bi_so100_leader,
    gamepad,
    homunculus,
    koch_leader,
    make_teleoperator_from_config,
    so100_leader,
    so101_leader,
)
from lerobot.utils.robot_utils import busy_wait


@dataclass
class LeaderConfig:
    teleop: TeleoperatorConfig


def teleop_loop(teleop: Teleoperator):
    fps = 30
    min_range = {
        "shoulder_pan.pos": -0.2500893176134298,
        "shoulder_lift.pos": -99.59399106780349,
        "elbow_flex.pos": 98.85361552028218,
        "wrist_flex.pos": 51.22918318794606,
        "wrist_roll.pos": -48.62240881658357,
        "gripper.pos": 0.0,
    }
    max_range = {
        "shoulder_pan.pos": -0.2500893176134298,
        "shoulder_lift.pos": -99.59399106780349,
        "elbow_flex.pos": 98.85361552028218,
        "wrist_flex.pos": 51.22918318794606,
        "wrist_roll.pos": -48.62240881658357,
        "gripper.pos": 0.0,
    }

    while True:
        loop_start = time.perf_counter()
        action = teleop.get_action()
        for k, v in action.items():
            min_range[k] = min(min_range[k], v)
            max_range[k] = max(max_range[k], v)

        # print(f"action: {action}")
        # print(f"range: {range}")

        print(f"min_range: {min_range}")
        print(f"max_range: {max_range}")

        dt_s = time.perf_counter() - loop_start
        busy_wait(1 / fps - dt_s)


@draccus.wrap()
def teleoperate(cfg: LeaderConfig):
    logging.info(pformat(asdict(cfg)))
    teleop = make_teleoperator_from_config(cfg.teleop)
    teleop.connect()

    try:
        teleop_loop(teleop)
    except KeyboardInterrupt:
        pass
    finally:
        teleop.disconnect()


def main():
    teleoperate()


if __name__ == "__main__":
    main()


"""
min_range: {'shoulder_pan.pos': -100.0, 'shoulder_lift.pos': -100.0, 'elbow_flex.pos': -99.73544973544973, 'wrist_flex.pos': -100.0, 'wrist_roll.pos': -100.0, 'gripper.pos': 0.0}
max_range: {'shoulder_pan.pos': 100.0, 'shoulder_lift.pos': 100.0, 'elbow_flex.pos': 100.0, 'wrist_flex.pos': 100.0, 'wrist_roll.pos': 100.0, 'gripper.pos': 100.0}
shoulder_pan_pos = 0 to 0.274 -> arm1
shoulder_lift.pos = -1.57 to 1.09 -> arm2
elbow_flex.pos = -0.39 to 2.35  -> arm4
wrist_flex.pos = -1.414 to 1.414  -> arm6
wrist_roll.pos = -2.094 to 2.094  -> arm7
gripper.pos = 0 to 0.45 -> gripper_right_finger_joint

"""
