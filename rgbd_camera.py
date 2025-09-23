#!/usr/bin/env python3
"""
rgbd_camera.py

Reusable RGB-D camera utility for PyBullet:
- Attach to a robot link (preferred) or use a fixed camera pose.
- Streams RGB, depth (in meters), and optional segmentation mask.
- Full control over forward/up/right axes so you can flip independently.

Usage:
    from rgbd_camera import RGBDCameraBullet, RGBDCameraConfig

    cam_cfg = RGBDCameraConfig(width=640, height=480, fov_deg=70.0, near=0.05, far=8.0)
    cam = RGBDCameraBullet(client_id=cid,
                           robot_id=robot_id,
                           link_name="xtion_rgb_optical_frame",
                           flip_fwd=(0,0,1),    # Z forward
                           flip_up=(0,-1,0),    # -Y up
                           flip_right=(1,0,0))  # X right
    rgb, depth_m, seg = cam.get_frame(follow=True)
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional, List, Tuple
import numpy as np
import pybullet as p


# ---------------------------------------------------------------------------
# Config dataclass
# ---------------------------------------------------------------------------
@dataclass
class RGBDCameraConfig:
    width: int = 640
    height: int = 480
    fov_deg: float = 70.0
    near: float = 0.05
    far: float = 8.0
    use_segmentation: bool = True
    renderer_if_gui: int = p.ER_BULLET_HARDWARE_OPENGL
    renderer_if_headless: int = p.ER_TINY_RENDERER


# ---------------------------------------------------------------------------
# Camera class
# ---------------------------------------------------------------------------
@dataclass
class RGBDCameraBullet:
    client_id: int
    # Attach to robot link
    robot_id: Optional[int] = None
    link_index: Optional[int] = None
    link_name: Optional[str] = None
    link_name_hints: List[str] = field(default_factory=lambda: [
        "xtion_rgb_optical_frame", "xtion_optical_frame",
        "head_2_link", "head_1_link"
    ])

    # Axis control: set forward/up/right vectors independently
    flip_fwd: Tuple[int, int, int] = (1, 0, 0)   # default +X forward
    flip_up: Tuple[int, int, int] = (0, 0, 1)    # default +Z up
    flip_right: Tuple[int, int, int] = (0, 1, 0) # right axis (not used in view matrix, for debug)

    # Fixed camera (legacy/test mode)
    use_fixed_camera: bool = False
    fixed_target: Tuple[float, float, float] = (0.0, 0.0, 0.6)
    fixed_distance: float = 2.2
    fixed_yaw_pitch_roll: Tuple[float, float, float] = (50.0, -20.0, 0.0)

    cfg: RGBDCameraConfig = field(default_factory=RGBDCameraConfig)
    _view: Optional[List[float]] = None
    _proj: Optional[List[float]] = None
    _gui: bool = True

    # -----------------------------------------------------------------------
    def set_gui_mode(self, is_gui: bool) -> None:
        self._gui = bool(is_gui)

    def set_link_by_name(self, name: str) -> None:
        if self.robot_id is None:
            raise ValueError("Cannot set link by name without a robot_id")
        idx = self._find_link_index_by_name(self.robot_id, name)
        if idx is None:
            raise ValueError(f"Link name '{name}' not found on robot.")
        self.link_index = idx
        self.link_name = name

    # -----------------------------------------------------------------------
    # Main API
    # -----------------------------------------------------------------------
    def get_frame(self, follow: bool = True):
        """
        Returns (rgb_uint8[h,w,3], depth_m_float[h,w], seg_int[h,w] or None).
        If follow=True and attached to a link, refresh view each call.
        """
        if self._view is None or self._proj is None or (follow and not self.use_fixed_camera):
            self._view, self._proj = self._compute_mats()

        width, height, rgb, depth, seg = p.getCameraImage(
            width=self.cfg.width,
            height=self.cfg.height,
            viewMatrix=self._view,
            projectionMatrix=self._proj,
            renderer=self.cfg.renderer_if_gui if self._gui else self.cfg.renderer_if_headless,
            flags=(p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX if self.cfg.use_segmentation else 0)
        )

        # Convert to numpy
        rgb_np = np.reshape(np.array(rgb, dtype=np.uint8), (height, width, 4))[:, :, :3]
        depth_buf = np.reshape(np.array(depth, dtype=np.float32), (height, width))
        depth_m = self._depth_buffer_to_meters(depth_buf, self.cfg.near, self.cfg.far)

        seg_np = None
        if self.cfg.use_segmentation:
            seg_np = np.reshape(np.array(seg, dtype=np.int32), (height, width))

        return rgb_np, depth_m, seg_np

    # -----------------------------------------------------------------------
    # Internals
    # -----------------------------------------------------------------------
    def _compute_mats(self):
        if self.use_fixed_camera or self.robot_id is None:
            return self._fixed_camera_mats()
        if self.link_index is None:
            self.link_index = self._resolve_link_index()
        return self._camera_mats_from_link(self.robot_id, self.link_index)

    def _fixed_camera_mats(self):
        view = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=self.fixed_target,
            distance=self.fixed_distance,
            yaw=self.fixed_yaw_pitch_roll[0],
            pitch=self.fixed_yaw_pitch_roll[1],
            roll=self.fixed_yaw_pitch_roll[2],
            upAxisIndex=2
        )
        aspect = float(self.cfg.width) / float(self.cfg.height)
        proj = p.computeProjectionMatrixFOV(
            fov=self.cfg.fov_deg, aspect=aspect,
            nearVal=self.cfg.near, farVal=self.cfg.far
        )
        return view, proj

    def _camera_mats_from_link(self, robot_id: int, link_idx: Optional[int]):
        if link_idx is None:
            pos, orn = p.getBasePositionAndOrientation(robot_id)
        else:
            ls = p.getLinkState(robot_id, link_idx, computeForwardKinematics=True)
            pos, orn = ls[4], ls[5]

        rot = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)

        fwd_local = np.array(self.flip_fwd, dtype=float)
        up_local = np.array(self.flip_up, dtype=float)

        fwd = rot @ fwd_local
        up = rot @ up_local

        cam_pos = np.array(pos)
        target = cam_pos + 1.0 * fwd

        view = p.computeViewMatrix(cam_pos.tolist(), target.tolist(), up.tolist())
        aspect = float(self.cfg.width) / float(self.cfg.height)
        proj = p.computeProjectionMatrixFOV(
            fov=self.cfg.fov_deg, aspect=aspect,
            nearVal=self.cfg.near, farVal=self.cfg.far
        )
        return view, proj

    def _resolve_link_index(self) -> Optional[int]:
        if self.robot_id is None:
            return None
        if self.link_name:
            idx = self._find_link_index_by_name(self.robot_id, self.link_name)
            if idx is not None:
                return idx
        names = self._list_links(self.robot_id)
        low = [n.lower() for n in names]
        for hint in self.link_name_hints:
            h = hint.lower()
            for i, nm in enumerate(low):
                if h in nm:
                    self.link_name = names[i]
                    return i
        return None

    @staticmethod
    def _list_links(robot_id: int) -> List[str]:
        return [p.getJointInfo(robot_id, j)[12].decode("utf-8")
                for j in range(p.getNumJoints(robot_id))]

    @staticmethod
    def _find_link_index_by_name(robot_id: int, name: str) -> Optional[int]:
        for j in range(p.getNumJoints(robot_id)):
            info = p.getJointInfo(robot_id, j)
            if info[12].decode("utf-8") == name:
                return j
        return None

    @staticmethod
    def _depth_buffer_to_meters(depth_buf: np.ndarray, near: float, far: float) -> np.ndarray:
        z_ndc = 2.0 * depth_buf - 1.0
        denom = (far + near) - z_ndc * (far - near)
        denom = np.where(np.abs(denom) < 1e-6, 1e-6, denom)
        return (2.0 * near * far) / denom
