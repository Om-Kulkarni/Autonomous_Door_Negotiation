#!/usr/bin/env python3
"""
nav_autonomous.py

Autonomous navigation helper for TIAGo in PyBullet.
- Rotate-to-face, then translate-to-standoff with constant speeds.
- Designed to be imported by tiago_nav_bullet.py without changing other behavior.
"""

from dataclasses import dataclass
from typing import Optional
import math
import numpy as np
import pybullet as p  # used only for debug lines

def angle_wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

@dataclass
class NavGoal:
    target_xy: np.ndarray         # [2]
    face_yaw: float               # base heading to face
    door_centroid_w: np.ndarray   # [3]
    door_normal_w: np.ndarray     # [3]
    standoff_m: float

class DoorAutoNavigator:
    """
    Super-simple navigator:
      1) ROTATE until facing the door plane (constant |wz|)
      2) TRANSLATE toward the waypoint at a constant |vx| (with zero-or-constant heading correction)
      3) Stop at ~standoff distance from the door plane
    """
    def __init__(self,
                 standoff_m: float = 0.30,
                 vx_const: float = 0.60,
                 wz_const: float = 1.20,
                 yaw_tol: float = 0.06,
                 dist_tol: float = 0.05,
                 draw_debug: bool = True):
        self.enabled = False
        self.goal: Optional[NavGoal] = None
        self.stage = 'idle'
        self.standoff_m = standoff_m
        self.vx_const = abs(float(vx_const))
        self.wz_const = abs(float(wz_const))
        self.yaw_tol = float(yaw_tol)
        self.dist_tol = float(dist_tol)
        self.draw_debug = draw_debug
        self._last_stage = self.stage

    # ---------------- goal handling ----------------

    def clear(self):
        self.goal = None
        self.stage = 'idle'

    def _choose_normal_sign(self, base_xy, door_c_w, door_n_w):
        """Ensure we approach the door (flip normal if it points away)."""
        n_xy = np.asarray([door_n_w[0], door_n_w[1]], dtype=np.float32)
        if np.linalg.norm(n_xy) < 1e-6:
            n_xy = np.array([1.0, 0.0], dtype=np.float32)
        n_xy /= (np.linalg.norm(n_xy) + 1e-8)
        approach_xy = np.asarray([door_c_w[0], door_c_w[1]], dtype=np.float32) - base_xy
        if np.linalg.norm(approach_xy) < 1e-6:
            return door_n_w
        approach_xy /= np.linalg.norm(approach_xy)
        return door_n_w if float(np.dot(n_xy, approach_xy)) >= 0.0 else -door_n_w

    def set_goal_from_detection(self, base_xy, base_yaw, door_centroid_w, door_normal_w, standoff_m=None):
        d = float(self.standoff_m if standoff_m is None else standoff_m)
        n_use = self._choose_normal_sign(base_xy, door_centroid_w, door_normal_w)
        n_xy = np.array([n_use[0], n_use[1]], dtype=np.float32)
        n_norm = np.linalg.norm(n_xy) + 1e-8
        n_xy /= n_norm

        tgt_xy = np.array([door_centroid_w[0], door_centroid_w[1]], dtype=np.float32) - d * n_xy
        face_yaw = math.atan2(n_xy[1], n_xy[0])

        self.goal = NavGoal(
            target_xy=tgt_xy,
            face_yaw=face_yaw,
            door_centroid_w=np.asarray(door_centroid_w, dtype=np.float32),
            door_normal_w=np.asarray(n_use, dtype=np.float32),
            standoff_m=d
        )
        self.stage = 'rotate'
        print(f"[AutoNav] New goal set | standoff={d:.2f} m | "
              f"tgt=({tgt_xy[0]:.2f},{tgt_xy[1]:.2f}) | face_yaw={face_yaw:.2f} rad")

    # ---------------- runtime ----------------

    def _draw_debug(self, robot_xy):
        if not self.draw_debug or self.goal is None:
            return
        try:
            tgt = self.goal.target_xy
            # base -> target (blue)
            p.addUserDebugLine([robot_xy[0], robot_xy[1], 0.05],
                               [tgt[0],     tgt[1],     0.05],
                               [0, 0, 1], lifeTime=0.2, lineWidth=2.0)
            # door normal (green)
            c = self.goal.door_centroid_w
            n = self.goal.door_normal_w
            p.addUserDebugLine(c.tolist(), (c + 0.5*n).tolist(), [0,1,0], lifeTime=0.2, lineWidth=2.0)
        except Exception:
            pass

    def update(self, base_pos_w, base_yaw):
        """
        Return (vx, wz, done).
        - vx is forward in BODY frame; caller should rotate to world.
        - wz is yaw rate in world Z.
        """
        if not self.enabled or self.goal is None:
            return 0.0, 0.0, False

        if self.stage != self._last_stage:
            print(f"[AutoNav] Stage: {self._last_stage} -> {self.stage}")
            self._last_stage = self.stage

        base_xy = np.array([base_pos_w[0], base_pos_w[1]], dtype=np.float32)
        tgt_xy  = self.goal.target_xy

        # Draw helpers
        self._draw_debug(base_xy)

        # Distances & angles
        dir_to_tgt = tgt_xy - base_xy
        dist = float(np.linalg.norm(dir_to_tgt))
        hdg_to_tgt = math.atan2(dir_to_tgt[1], dir_to_tgt[0]) if dist > 1e-6 else base_yaw

        n = self.goal.door_normal_w
        c = self.goal.door_centroid_w
        # s_plane = float((base_pos_w[0] - c[0]) * n[0] + (base_pos_w[1] - c[1]) * n[1])
        s_approach = -float((base_pos_w[0] - c[0]) * n[0] + (base_pos_w[1] - c[1]) * n[1])

        # Safety: do not cross the standoff plane
        # if s_plane > self.goal.standoff_m * 0.98:
        #     self.stage = 'done'
        if s_approach <= self.goal.standoff_m:
            self.stage = 'done'

        # ROTATE: constant |wz|
        if self.stage == 'rotate':
            yaw_err = angle_wrap(self.goal.face_yaw - base_yaw)
            if abs(yaw_err) < self.yaw_tol:
                self.stage = 'translate'   # fall-through
            else:
                wz = self.wz_const * (1.0 if yaw_err > 0.0 else -1.0)
                return 0.0, wz, False

        # TRANSLATE: constant |vx|, optional constant correction if heading is off
        if self.stage == 'translate':
            yaw_err = angle_wrap(hdg_to_tgt - base_yaw)
            vx = self.vx_const
            wz = 0.0
            if abs(yaw_err) > self.yaw_tol * 2.0:
                wz = 0.5 * self.wz_const * (1.0 if yaw_err > 0.0 else -1.0)

            if (dist < self.dist_tol) or (s_approach <= self.goal.standoff_m):
                self.stage = 'done'
                print(f"[AutoNav] Reached waypoint | dist={dist:.3f} m | s_approach={s_approach:.3f} m")
                return 0.0, 0.0, True
            return vx, wz, False

        # DONE
        return 0.0, 0.0, True
