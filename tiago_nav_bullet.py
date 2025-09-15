#!/usr/bin/env python3
"""
tiago_nav_bullet.py

Spawn TIAGo in PyBullet:
- Legacy test: plane-only
- Implementation: room-with-door environment (default)

Optionally attach an RGB-D camera using the reusable RGBDCameraBullet class.
"""

import os
import sys
import time
import math
import traceback

import pybullet as p
import pybullet_data

from rgbd_camera import RGBDCameraBullet, RGBDCameraConfig

# ---------------------------------------------------------------------------
# User toggles
# ---------------------------------------------------------------------------
GUI = True

# Implementation (always part of the code)
USE_ROOM_ENV = True
SPAWN_DOOR = True

# Legacy test toggle
LEGACY_PLANE_ONLY_TEST = False

# RGB-D camera streaming
STREAM_RGBD = True

# ---------------------------------------------------------------------------
DEFAULT_URDF = "tiago_rl/assets/tiago_pal_gripper.urdf"

if USE_ROOM_ENV and not LEGACY_PLANE_ONLY_TEST:
    from world_room_door import RoomDoorEnv


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
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
    return p.loadURDF("plane.urdf")


def load_tiago(urdf_path: str):
    if not os.path.exists(urdf_path):
        print(f"ERROR: URDF not found: {urdf_path}", file=sys.stderr)
        return None
    start_pos = [0.0, 0.0, 0.1]
    start_orn = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
    try:
        robot_id = p.loadURDF(
            urdf_path, start_pos, start_orn,
            useFixedBase=False, flags=p.URDF_USE_INERTIA_FROM_FILE
        )
        return robot_id
    except Exception as e:
        print(f"ERROR: Failed to load URDF '{urdf_path}': {e}", file=sys.stderr)
        return None


def print_periodic_info(robot_id: int, interval: int = 240, counter: int = 0):
    """Every `interval` steps, print joint count and link[0] state."""
    if counter % interval == 0:
        num_j = p.getNumJoints(robot_id)
        print(f"[Info] Joints count: {num_j}")
        try:
            ls = p.getLinkState(robot_id, 0)
            pos, orn = ls[4], ls[5]
            print(f"[Info] Link[0] position: {pos}, orientation (quat): {orn}")
        except Exception as e:
            print(f"WARNING: Could not get Link[0] state: {e}", file=sys.stderr)
    return counter + 1


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    cid = connect(gui=GUI)
    if cid is None:
        sys.exit(1)

    try:
        # Base plane (always present)
        load_plane()

        # Room-with-door (implementation) unless legacy test chosen
        env = None
        if USE_ROOM_ENV and not LEGACY_PLANE_ONLY_TEST:
            env = RoomDoorEnv(client_id=cid, doorway_width=1.0, clearance_eps=0.01)
            env.build_room()
            if SPAWN_DOOR:
                door_x = env.room_size_xy[0] * 0.5 - env.wall_t * 0.5 + env.clearance_eps
                y_left = env.doorway_center_y - env.doorway_width * 0.5
                env.spawn_door(
                    base_pos=[door_x, y_left, 0.0],
                    base_orn_rpy=(0.0, 0.0, math.pi/2),
                    size_xyz=(0.90, 0.04, 2.0),
                    initial_angle_deg=20.0,
                    hinge_axis="z",
                )

        # Load TIAGo
        robot_id = load_tiago(DEFAULT_URDF)
        if robot_id is None:
            sys.exit(2)

        # Optional RGB-D camera
        cam = None
        if STREAM_RGBD:
            cam_cfg = RGBDCameraConfig(width=640, height=480, fov_deg=70.0, near=0.05, far=8.0)
            cam = RGBDCameraBullet(
                client_id=cid,
                robot_id=robot_id,
                link_name="xtion_rgb_optical_frame",  # or None to auto-resolve
                flip_fwd=(0, 0, 1),    # try (1,0,0), (0,1,0), (0,0,1), etc.
                flip_up=(0, -1, 0),    # e.g. invert Y if needed
                flip_right=(1, 0, 0),  # optional, just for debugging reference
                use_fixed_camera=False
            )            
            cam.set_gui_mode(GUI)

        # Main loop
        iter_count = 0
        while True:
            p.stepSimulation()
            iter_count = print_periodic_info(robot_id, interval=240, counter=iter_count)

            if STREAM_RGBD and cam is not None and iter_count % 120 == 0:
                rgb, depth_m, seg = cam.get_frame(follow=True)
                h, w = depth_m.shape
                mid = float(depth_m[h // 2, w // 2])
                print(f"[RGBD] rgb={rgb.shape}, depth(m)={depth_m.shape}, mid={mid:.3f}m")

            if GUI:
                time.sleep(1.0 / 240.0)

    except KeyboardInterrupt:
        print("\n[Exit] KeyboardInterrupt.")
        sys.exit(0)
    except Exception:
        print("ERROR: Runtime exception:")
        traceback.print_exc()
        sys.exit(3)
    finally:
        try:
            p.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
