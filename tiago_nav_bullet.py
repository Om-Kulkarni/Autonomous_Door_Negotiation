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
import random  # <— needed for respawn key

import pybullet as p
import pybullet_data

from rgbd_camera import RGBDCameraBullet, RGBDCameraConfig
from detect_door import (
    DoorPoseEstimator,
    CameraPose,
    CameraIntrinsics,
    pinhole_from_fov,
)
# NEW: import the autonomous navigator
from nav_autonomous import DoorAutoNavigator

# === Teleop additions (BEGIN) ================================================
import json
import socket
import threading
from typing import Tuple

class BaseTeleopReceiver:
    """
    Lightweight UDP receiver for base teleop commands (vx, wz).
    Non-blocking, thread-safe, zero dependency beyond stdlib.
    - Sender: teleop_base_keyboard.py (UDP JSON: {"vx": float, "wz": float})
    - This class keeps the latest command and offers get_command().
    - If no packet arrives for 'timeout_s', command decays to zeros.
    """

    def __init__(self, bind_host: str = "127.0.0.1", bind_port: int = 9999, timeout_s: float = 0.5):
        self.addr = (bind_host, bind_port)
        self.timeout_s = timeout_s
        self._vx = 0.0
        self._wz = 0.0
        self._last_rx = 0.0
        self._lock = threading.Lock()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # allow quick restarts
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(self.addr)
        self._sock.setblocking(False)
        self._run = True
        self._thr = threading.Thread(target=self._loop, daemon=True)
        self._thr.start()
        print(f"[TeleopRX] Listening on {self.addr[0]}:{self.addr[1]} (UDP).")

    def _loop(self):
        while self._run:
            try:
                data, _ = self._sock.recvfrom(1024)
                obj = json.loads(data.decode("utf-8"))
                vx = float(obj.get("vx", 0.0))
                wz = float(obj.get("wz", 0.0))
                with self._lock:
                    self._vx, self._wz = vx, wz
                    self._last_rx = time.time()
            except BlockingIOError:
                time.sleep(0.005)
            except Exception:
                # swallow malformed packets
                time.sleep(0.005)

    def get_command(self) -> Tuple[float, float]:
        """Return (vx, wz); zeros if stale beyond timeout."""
        with self._lock:
            stale = (time.time() - self._last_rx) > self.timeout_s
            if stale:
                return 0.0, 0.0
            return float(self._vx), float(self._wz)

    def shutdown(self):
        self._run = False
        try:
            self._thr.join(timeout=0.5)
        except Exception:
            pass
        try:
            self._sock.close()
        except Exception:
            pass
# === Teleop additions (END) ==================================================

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

# --- NEW: autonomy speed controls (constant speeds) --------------------------
AUTON_VX = 4.0    # m/s forward (body frame) — increase/decrease as you like
AUTON_WZ = 1.50   # rad/s yaw rate         — increase/decrease as you like
# ----------------------------------------------------------------------------

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

# Yaw helper stays here (used by the main loop)
def yaw_from_quat_xyzw(q_xyzw):
    return p.getEulerFromQuaternion(q_xyzw)[2]

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    cid = connect(gui=GUI)
    if cid is None:
        sys.exit(1)

    # === Teleop: start lightweight UDP receiver (non-blocking) ===
    teleop_rx = BaseTeleopReceiver(bind_host="127.0.0.1", bind_port=9999, timeout_s=0.6)

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

        # === Auto-nav wiring (one-time) ======================================
        STANDOFF_M = 1.0  # desired distance from door plane when goal is set
        navigator = DoorAutoNavigator(
            standoff_m=STANDOFF_M,
            vx_const=AUTON_VX,
            wz_const=AUTON_WZ,
        )
        auto_mode = False  # toggled with 'N'
        last_key_toggle_time = 0.0
        key_debounce_s = 0.20
        # =====================================================================

        # Main loop
        iter_count = 0
        while True:
            # Keyboard: 'N' toggle auton, 'R' respawn door at random angle (unchanged)
            try:
                keys = p.getKeyboardEvents()
                now = time.time()
                if keys:
                    if (ord('n') in keys or ord('N') in keys) and (now - last_key_toggle_time > key_debounce_s):
                        auto_mode = not auto_mode
                        navigator.enabled = auto_mode
                        state = "ON" if auto_mode else "OFF"
                        print(f"[AutoNav] Autonomy {state}")
                        last_key_toggle_time = now

                    if (ord('r') in keys or ord('R') in keys) and (now - last_key_toggle_time > key_debounce_s):
                        if env is not None and SPAWN_DOOR:
                            try:
                                if isinstance(door_body_id, int) and door_body_id >= 0:
                                    p.removeBody(door_body_id)
                            except Exception:
                                pass
                            door_x = env.room_size_xy[0] * 0.5 - env.wall_t * 0.5 + env.clearance_eps
                            y_left = env.doorway_center_y - env.doorway_width * 0.5
                            ang_deg = random.uniform(0.0, 80.0)
                            maybe_id = env.spawn_door(
                                base_pos=[door_x, y_left, 0.0],
                                base_orn_rpy=(0.0, 0.0, math.pi/2),
                                size_xyz=(0.90, 0.04, 2.0),
                                initial_angle_deg=ang_deg,
                                hinge_axis="z",
                            )
                            door_body_id = maybe_id if isinstance(maybe_id, int) else getattr(env, "door_id", None)
                            if detector is not None:
                                detector.door_body_id = door_body_id
                            print(f"[AutoNav] Door re-spawned at {ang_deg:.1f}° | id={door_body_id}")
                            navigator.clear()
                            last_key_toggle_time = now
            except Exception:
                pass

            # Command selection (teleop or autonomy)
            vx_cmd, wz_cmd = teleop_rx.get_command()
            if auto_mode and navigator is not None:
                try:
                    base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
                    base_yaw = yaw_from_quat_xyzw(base_orn)
                    vx_auto, wz_auto, done = navigator.update(base_pos, base_yaw)
                    vx_cmd, wz_cmd = vx_auto, wz_auto
                except Exception:
                    pass

            # Apply (rotate body-forward vx into world XY)
            try:
                base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
                base_yaw = yaw_from_quat_xyzw(base_orn)
                vx_world = vx_cmd * math.cos(base_yaw)
                vy_world = vx_cmd * math.sin(base_yaw)
                if iter_count % 30 == 0:
                    print(f"[Cmd] auto={auto_mode} stage={getattr(navigator,'stage','-')} "
                          f"vx_body={vx_cmd:.2f} wz={wz_cmd:.2f} | world=({vx_world:.2f},{vy_world:.2f})")
                p.resetBaseVelocity(
                    robot_id,
                    linearVelocity=[vx_world, vy_world, 0.0],
                    angularVelocity=[0.0, 0.0, wz_cmd],
                )
            except Exception:
                pass

            p.stepSimulation()
            iter_count += 1

            if STREAM_RGBD and cam is not None and iter_count % 2 == 0:
                rgb, depth_m, seg = cam.get_frame(follow=True)

                # (unchanged sanity + remap + logging)
                if iter_count == 2:
                    import numpy as np
                    print("[DBG] seg is None? ->", seg is None)
                    if seg is not None:
                        u = np.unique(seg)
                        print("[DBG] unique raw seg ids (first 20):", u[:20])
                        high = np.unique(seg >> 24)
                        print("[DBG] objectUniqueIds present:", high)
                    print("[DBG] door_body_id before remap:", door_body_id)

                if seg is not None:
                    import numpy as np
                    obj_ids = np.unique(seg >> 24)
                    if (door_body_id is None) or (door_body_id not in obj_ids):
                        ids_flat = (seg >> 24).ravel()
                        ids_flat = ids_flat[ids_flat > 0]
                        if ids_flat.size:
                            vals, counts = np.unique(ids_flat, return_counts=True)
                            remap = int(vals[np.argmax(counts)])
                            print(f"[DBG] remapping door_body_id {door_body_id} -> {remap}")
                            door_body_id = remap
                            if detector is not None:
                                detector.door_body_id = door_body_id

                if iter_count % 120 == 0:
                    h, w = depth_m.shape
                    mid = float(depth_m[h // 2, w // 2])
                    print(f"[RGBD] rgb={rgb.shape}, depth(m)={depth_m.shape}, mid={mid:.3f}m")

                if DOOR_DETECTION and detector is not None and cam_intr is not None and (iter_count % DETECTION_PERIOD == 0):
                    link_idx = getattr(cam, "link_index", None)
                    if link_idx is None:
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

                        # Seed/update auton goal from detection
                        try:
                            base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
                            base_yaw = yaw_from_quat_xyzw(base_orn)
                            base_xy = np.array([base_pos[0], base_pos[1]], dtype=np.float32)

                            navigator.set_goal_from_detection(
                                base_xy=base_xy,
                                base_yaw=base_yaw,
                                door_centroid_w=c_w,
                                door_normal_w=n_w,
                                standoff_m=STANDOFF_M,
                            )
                            # optional auto-start
                            if not auto_mode:
                                auto_mode = True
                                navigator.enabled = True
                                print("[AutoNav] Autonomy ON (auto-start on goal)")
                        except Exception:
                            pass
                    else:
                        print("[Door] Not detected (no matching segmentation / appearance evidence on this frame).")

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
            teleop_rx.shutdown()
        except Exception:
            pass
        try:
            p.disconnect()
        except Exception:
            pass

if __name__ == "__main__":
    main()
