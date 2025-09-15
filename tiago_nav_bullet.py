#!/usr/bin/env python3
"""
tiago_nav_bullet.py

Spawn TIAGo in PyBullet:
- Legacy test: plane-only
- Implementation: room-with-door environment (default)

Optionally attach an RGB-D camera using the reusable RGBDCameraBullet class.
Includes door pose detection via segmentation mask or appearance-based method.
"""

import os
import sys
import time
import math
import traceback
import numpy as np

import pybullet as p
import pybullet_data

from rgbd_camera import RGBDCameraBullet, RGBDCameraConfig
from detect_door import (
    DoorPoseEstimator,
    CameraPose,
    CameraIntrinsics,
    pinhole_from_fov,
)

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

# Door detection
DOOR_DETECTION = True
DETECTION_STRATEGY = "segmentation"  # "segmentation" or "appearance"
DOOR_COLOR_BGR = (20, 180, 240)      # only used for "appearance"
DETECTION_PERIOD = 6                 # run detector every N frames

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


def quat_to_rot(q_xyzw):
    """Quaternion (x,y,z,w) -> rotation matrix 3x3."""
    x, y, z, w = q_xyzw
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    R = np.array([
        [1 - 2*(yy+zz),     2*(xy - wz),     2*(xz + wy)],
        [    2*(xy + wz), 1 - 2*(xx+zz),     2*(yz - wx)],
        [    2*(xz - wy),     2*(yz + wx), 1 - 2*(xx+yy)],
    ], dtype=np.float32)
    return R


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
        door_body_id = None
        if USE_ROOM_ENV and not LEGACY_PLANE_ONLY_TEST:
            env = RoomDoorEnv(client_id=cid, doorway_width=1.0, clearance_eps=0.01)
            env.build_room()
            if SPAWN_DOOR:
                door_x = env.room_size_xy[0] * 0.5 - env.wall_t * 0.5 + env.clearance_eps
                y_left = env.doorway_center_y - env.doorway_width * 0.5
                maybe_id = env.spawn_door(
                    base_pos=[door_x, y_left, 0.0],
                    base_orn_rpy=(0.0, 0.0, math.pi/2),
                    size_xyz=(0.90, 0.04, 2.0),
                    initial_angle_deg=0.0,
                    hinge_axis="z",
                )
                # if isinstance(maybe_id, int):
                #     door_body_id = maybe_id
                # elif hasattr(env, "door_id"):              # <<< use the correct attribute name
                #     door_body_id = getattr(env, "door_id")

                # print(f"[DBG] door_body_id = {door_body_id}")  # one-time sanity check

                door_body_id = maybe_id if isinstance(maybe_id, int) else getattr(env, "door_id", None)
                print(f"[DBG] door_body_id = {door_body_id}")  # should print 1 (or a small int), NOT 6

        # Load TIAGo
        robot_id = load_tiago(DEFAULT_URDF)
        if robot_id is None:
            sys.exit(2)

        # Optional RGB-D camera
        cam = None
        detector = None
        cam_intr: CameraIntrinsics | None = None

        if STREAM_RGBD:
            cam_cfg = RGBDCameraConfig(width=640, height=480, fov_deg=70.0, near=0.05, far=8.0)
            cam = RGBDCameraBullet(
                client_id=cid,
                robot_id=robot_id,
                link_name="xtion_rgb_optical_frame",
                flip_fwd=(0, 0, 1),
                flip_up=(0, -1, 0),
                flip_right=(1, 0, 0),
                use_fixed_camera=False
            )
            cam.set_gui_mode(GUI)

            cam_intr = pinhole_from_fov(
                cam_cfg.width, cam_cfg.height, cam_cfg.fov_deg, cam_cfg.near, cam_cfg.far
            )

            if DOOR_DETECTION:
                detector = DoorPoseEstimator(
                    strategy=DETECTION_STRATEGY,
                    door_body_id=door_body_id,
                    rgb_color_bgr=DOOR_COLOR_BGR,
                    rgb_thresh=30,
                    min_pixels=200,
                )

        # Main loop
        iter_count = 0
        while True:
            p.stepSimulation()
            # iter_count = print_periodic_info(robot_id, interval=240, counter=iter_count)
            iter_count += 1

            if STREAM_RGBD and cam is not None and iter_count % 2 == 0:
                rgb, depth_m, seg = cam.get_frame(follow=True)

                # --- SANITY CHECK: runs once near the beginning ---
                if iter_count == 2:  # print once
                    import numpy as np
                    print("[DBG] seg is None? ->", seg is None)
                    if seg is not None:
                        u = np.unique(seg)
                        print("[DBG] unique raw seg ids (first 20):", u[:20])
                        high = np.unique(seg >> 24)
                        print("[DBG] objectUniqueIds present:", high)
                    print("[DBG] door_body_id before remap:", door_body_id)
                # ---------------------------------------------------

                # --- AUTO-REMAP: ensure door_body_id matches an objectUniqueId ---
                if seg is not None:
                    obj_ids = np.unique(seg >> 24)
                    if (door_body_id is None) or (door_body_id not in obj_ids):
                        ids_flat = (seg >> 24).ravel()
                        ids_flat = ids_flat[ids_flat > 0]  # ignore background
                        if ids_flat.size:
                            vals, counts = np.unique(ids_flat, return_counts=True)
                            remap = int(vals[np.argmax(counts)])
                            print(f"[DBG] remapping door_body_id {door_body_id} -> {remap}")
                            door_body_id = remap
                            if detector is not None:
                                detector.door_body_id = door_body_id
                # -----------------------------------------------------------------

                if iter_count % 120 == 0:
                    h, w = depth_m.shape
                    mid = float(depth_m[h // 2, w // 2])
                    print(f"[RGBD] rgb={rgb.shape}, depth(m)={depth_m.shape}, mid={mid:.3f}m")

                if DOOR_DETECTION and detector is not None and cam_intr is not None and (iter_count % DETECTION_PERIOD == 0):
                    # Build camera pose from camera class / link
                    link_idx = getattr(cam, "link_index", None)
                    if link_idx is None:
                        # Try to resolve from name if the camera exposes it
                        link_name = getattr(cam, "link_name", None)
                        link_idx = -1
                        if link_name is not None:
                            n = p.getNumJoints(robot_id)
                            for i in range(n):
                                if p.getJointInfo(robot_id, i)[12].decode("utf-8") == link_name:
                                    link_idx = i
                                    break

                    if link_idx is not None and link_idx >= 0:
                        ls = p.getLinkState(robot_id, link_idx, computeForwardKinematics=True)
                        pos_w, orn_wxyzw = ls[4], ls[5]
                    else:
                        pos_w, orn_wxyzw = p.getBasePositionAndOrientation(robot_id)

                    rot_wc = quat_to_rot(orn_wxyzw)

                    cam_pose = CameraPose(
                        pos_w=np.asarray(pos_w, dtype=np.float32),
                        rot_wc=rot_wc.astype(np.float32)
                    )

                    result = detector.update(rgb, depth_m, seg, cam_intr, cam_pose)
                    if result is not None:
                        c_w = result.centroid_w
                        n_w = result.normal_w
                        print(f"[Door] {result.strategy} | pts={result.num_points} | "
                            f"centroid_w=({c_w[0]:.3f},{c_w[1]:.3f},{c_w[2]:.3f}) "
                            f"normal_w=({n_w[0]:.2f},{n_w[1]:.2f},{n_w[2]:.2f})")
                        try:
                            p.addUserDebugLine(
                                lineFromXYZ=c_w.tolist(),
                                lineToXYZ=(c_w + 0.5 * n_w).tolist(),
                                lineColorRGB=[0, 1, 0],
                                lifeTime=0.2,
                                lineWidth=2.0
                            )
                        except Exception:
                            pass

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
