#!/usr/bin/env python3
"""
tiago_nav_bullet.py

Spawn TIAGo on a plane, then loop forever stepping simulation.
Prints some link/joint info periodically. Exits only via Ctrl+C.

Exit codes:
  0  — clean exit (KeyboardInterrupt)
  1  — failure to connect to PyBullet
  2  — failure to load URDF
  3  — other runtime error
"""
import os
import sys
import time
import traceback

import pybullet as p
import pybullet_data

DEFAULT_URDF = "tiago_rl/assets/tiago_pal_gripper.urdf"


def connect(gui: bool = True):
    mode = p.GUI if gui else p.DIRECT
    cid = p.connect(mode)
    if cid < 0:
        print("ERROR: Could not connect to PyBullet.", file=sys.stderr)
        return None
    return cid


def load_plane():
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0.0, 0.0, -9.81)
    plane_id = p.loadURDF("plane.urdf")
    return plane_id


def load_tiago(urdf_path: str):
    if not os.path.exists(urdf_path):
        print(f"ERROR: URDF file not found: {urdf_path}", file=sys.stderr)
        return None
    start_pos = [0.0, 0.0, 0.1]
    start_orn = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
    try:
        robot_id = p.loadURDF(urdf_path, start_pos, start_orn, useFixedBase=False)
        return robot_id
    except Exception as e:
        print(f"ERROR: Failed to load URDF '{urdf_path}': {e}", file=sys.stderr)
        return None


def print_periodic_info(robot_id: int, interval: int = 240, counter: int = 0):
    """
    Print joint/link info every `interval` iterations of the simulation loop.
    `counter` is current iteration count; returns new counter.
    """
    if counter % interval == 0:
        num_j = p.getNumJoints(robot_id)
        print(f"[Info] Joints count: {num_j}")
        # show pose of first link to verify world transform
        try:
            ls = p.getLinkState(robot_id, 0)
            pos, orn = ls[4], ls[5]
            print(f"[Info] Link[0] world position: {pos}, orientation (quat): {orn}")
        except Exception as e:
            print(f"WARNING: Could not get Link[0] state: {e}", file=sys.stderr)
    return counter + 1


def main():
    # optional args in future e.g. --direct, --urdf, etc.; minimal for now
    gui = True
    urdf = DEFAULT_URDF

    cid = connect(gui=gui)
    if cid is None:
        sys.exit(1)

    try:
        load_plane()
        robot_id = load_tiago(urdf)
        if robot_id is None:
            sys.exit(2)

        # main loop — runs until interrupted
        iter_count = 0
        while True:
            p.stepSimulation()
            iter_count = print_periodic_info(robot_id, interval=240, counter=iter_count)
            if gui:
                # small sleep so you can see frame updates and allow interrupt
                time.sleep(1.0 / 240.0)

    except KeyboardInterrupt:
        print("\nReceived KeyboardInterrupt. Exiting cleanly.")
        sys.exit(0)
    except Exception as e:
        print("ERROR: Runtime exception encountered:")
        traceback.print_exc()
        sys.exit(3)
    finally:
        try:
            p.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
