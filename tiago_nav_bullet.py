#!/usr/bin/env python3
"""
tiago_nav_bullet.py

Spawn TIAGo on a plane (legacy) OR in a room-with-door test env.
"""

import os
import sys
import time
import traceback
import math

import pybullet as p
import pybullet_data

# ---- toggles -------------------------------------------------
USE_ROOM_ENV = True          # set False to run the legacy plane-only test
SPAWN_DOOR    = True         # only used if USE_ROOM_ENV is True
# --------------------------------------------------------------

DEFAULT_URDF = "tiago_rl/assets/tiago_pal_gripper.urdf"

# Optional import of the environment module
if USE_ROOM_ENV:
    from world_room_door import RoomDoorEnv

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
    if counter % interval == 0:
        num_j = p.getNumJoints(robot_id)
        print(f"[Info] Joints count: {num_j}")
        try:
            ls = p.getLinkState(robot_id, 0)
            pos, orn = ls[4], ls[5]
            print(f"[Info] Link[0] world position: {pos}, orientation (quat): {orn}")
        except Exception as e:
            print(f"WARNING: Could not get Link[0] state: {e}", file=sys.stderr)
    return counter + 1

def main():
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

        # --- Step 2 env (optional) ---
        env = None
        if USE_ROOM_ENV:
            env = RoomDoorEnv(client_id=cid, doorway_width=1.0, clearance_eps=0.01)
            env.build_room()
            # Hinge position computed from the room + doorway geometry:
            door_x = env.room_size_xy[0]*0.5 - env.wall_t*0.5 + env.clearance_eps
            # Choose which jamb you want:
            y_left  = env.doorway_center_y - env.doorway_width*0.5   # LEFT hinge
            # y_right = env.doorway_center_y + env.doorway_width*0.5  # RIGHT hinge (use this instead if needed)
            if SPAWN_DOOR:
                # Place door at +X wall side, hinge along Z, initially 20 deg open
                env.spawn_door(
                    base_pos=[door_x, y_left, 0.0],           # pivot on the LEFT jamb
                    base_orn_rpy=(0.0, 0.0, math.pi/2),       # yaw 90° ⇒ 0° = closed, positive angles = opening
                    size_xyz=(0.90, 0.04, 2.0),
                    initial_angle_deg=20.0,                   # start 20° open
                    hinge_axis="z",
                )

        # main loop — runs until interrupted
        iter_count = 0
        while True:
            p.stepSimulation()
            iter_count = print_periodic_info(robot_id, interval=240, counter=iter_count)
            if gui:
                time.sleep(1.0 / 240.0)

    except KeyboardInterrupt:
        print("\nReceived KeyboardInterrupt. Exiting cleanly.")
        sys.exit(0)
    except Exception:
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
