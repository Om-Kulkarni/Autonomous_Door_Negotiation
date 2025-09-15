# detect_door.py
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Literal, Optional, Tuple

import numpy as np


Strategy = Literal["segmentation", "appearance"]


@dataclass
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    near: float
    far: float


@dataclass
class CameraPose:
    # World-from-camera transform
    pos_w: np.ndarray          # shape (3,)
    rot_wc: np.ndarray         # shape (3,3) rotation matrix (R_wc): p_w = R_wc @ p_c + t_w
    # If you only have quaternion, convert to rot before calling


@dataclass
class DoorPose:
    # In camera frame
    centroid_c: np.ndarray     # (3,)
    normal_c: np.ndarray       # (3,)
    # In world frame
    centroid_w: np.ndarray     # (3,)
    normal_w: np.ndarray       # (3,)
    # Debug
    num_points: int
    strategy: Strategy


class DoorPoseEstimator:
    """
    Simple, sim-friendly door pose estimator.

    Supports two strategies:
      - 'segmentation': use PyBullet segmentation mask and a known door body id
      - 'appearance'  : color-threshold + morphology + plane fit (RGB fallback)

    API:
      - update(...) -> Optional[DoorPose]
    """

    def __init__(
        self,
        strategy: Strategy = "segmentation",
        door_body_id: Optional[int] = None,
        rgb_color_bgr: Optional[Tuple[int, int, int]] = None,
        rgb_thresh: int = 30,
        min_pixels: int = 200,
    ):
        self.strategy = strategy
        self.door_body_id = door_body_id          # needed for 'segmentation'
        self.rgb_color_bgr = rgb_color_bgr        # needed for 'appearance'
        self.rgb_thresh = int(rgb_thresh)
        self.min_pixels = int(min_pixels)

    # ---------- public ----------

    def update(
        self,
        rgb: np.ndarray,            # HxWx3 uint8
        depth_m: np.ndarray,        # HxW float32/float64 in meters
        seg: Optional[np.ndarray],  # HxW int (PyBullet segmentation)
        intr: CameraIntrinsics,
        cam_pose: CameraPose,
    ) -> Optional[DoorPose]:
        mask = None
        if self.strategy == "segmentation":
            if seg is None:
                return None
            if self.door_body_id is None:
                return None
            mask = self._mask_from_seg(seg, self.door_body_id)
        elif self.strategy == "appearance":
            mask = self._mask_from_rgb(rgb, self.rgb_color_bgr, self.rgb_thresh)
        else:
            raise ValueError(f"Unknown strategy: {self.strategy}")

        if mask is None:
            return None

        # Depth sanity & point cloud extraction
        valid = np.isfinite(depth_m) & (depth_m > 0.0)
        mask = mask & valid
        if mask.sum() < self.min_pixels:
            return None

        pts_c = self._points_from_depth_mask(depth_m, mask, intr)   # Nx3 in camera frame
        if pts_c.shape[0] < 3:
            return None

        centroid_c, normal_c = self._fit_plane_svd(pts_c)

        # World transform
        centroid_w = cam_pose.rot_wc @ centroid_c + cam_pose.pos_w
        normal_w = cam_pose.rot_wc @ normal_c
        normal_w = normal_w / (np.linalg.norm(normal_w) + 1e-9)

        return DoorPose(
            centroid_c=centroid_c,
            normal_c=normal_c,
            centroid_w=centroid_w,
            normal_w=normal_w,
            num_points=int(pts_c.shape[0]),
            strategy=self.strategy,
        )

    # ---------- helpers ----------

    @staticmethod
    def _mask_from_seg(seg: np.ndarray, door_body_id: int) -> np.ndarray:
        """
        Build a boolean mask for the door from a PyBullet segmentation image.

        PyBullet may encode (objectUniqueId << 24) + linkIndex when using the
        ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX flag. Handle both cases.
        """
        seg = np.asarray(seg)
        # Case 1: values are objectUniqueId directly
        direct = (seg == door_body_id)

        # Case 2: high 8 bits store objectUniqueId
        high_id = (seg >> 24)
        packed = (high_id == door_body_id)

        mask = direct | packed
        return mask

    @staticmethod
    def _mask_from_rgb(
        rgb: np.ndarray,
        color_bgr: Optional[Tuple[int, int, int]],
        thresh: int,
    ) -> Optional[np.ndarray]:
        """
        Coarse color threshold: pick pixels close to a known BGR color.
        If color_bgr is None, return None.
        """
        if color_bgr is None:
            return None
        # Convert RGB->BGR diff (since PyBullet camera typically returns RGB)
        target_rgb = np.array(color_bgr[::-1], dtype=np.float32)  # convert BGR->RGB
        img = rgb.astype(np.float32)
        diff = np.linalg.norm(img - target_rgb, axis=2)
        mask = diff < float(thresh)

        # Simple largest-component keep (no OpenCV dependency)
        # A quick-and-dirty connected component using numpy (4-neighbor BFS)
        # to avoid pulling in cv2. For speed/robustness, feel free to swap with cv2 later.
        return DoorPoseEstimator._largest_component(mask)

    @staticmethod
    def _largest_component(mask: np.ndarray) -> np.ndarray:
        H, W = mask.shape
        visited = np.zeros_like(mask, dtype=bool)
        best_count = 0
        best = None

        # iterate sparse: only where mask True
        ys, xs = np.where(mask)
        seen = set()
        for y, x in zip(ys, xs):
            if visited[y, x]:
                continue
            # BFS
            q = [(y, x)]
            comp = []
            visited[y, x] = True
            while q:
                cy, cx = q.pop()
                comp.append((cy, cx))
                for ny, nx in ((cy-1, cx), (cy+1, cx), (cy, cx-1), (cy, cx+1)):
                    if 0 <= ny < H and 0 <= nx < W and (not visited[ny, nx]) and mask[ny, nx]:
                        visited[ny, nx] = True
                        q.append((ny, nx))
            if len(comp) > best_count:
                best_count = len(comp)
                best = comp

        if best is None or best_count == 0:
            return np.zeros_like(mask, dtype=bool)

        out = np.zeros_like(mask, dtype=bool)
        for (yy, xx) in best:
            out[yy, xx] = True
        return out

    @staticmethod
    def _points_from_depth_mask(depth_m: np.ndarray, mask: np.ndarray, intr: CameraIntrinsics) -> np.ndarray:
        """
        Back-project depth to 3D in the camera frame.
        """
        ys, xs = np.where(mask)
        z = depth_m[ys, xs]
        x = (xs - intr.cx) * (z / intr.fx)
        y = (ys - intr.cy) * (z / intr.fy)
        pts = np.stack([x, y, z], axis=1).astype(np.float32)
        return pts

    @staticmethod
    def _fit_plane_svd(pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Fit plane via SVD on centered points. Returns (centroid, normal) in camera frame.
        Normal points along smallest singular vector direction (unit length).
        """
        c = pts.mean(axis=0)
        X = pts - c
        _, _, vh = np.linalg.svd(X, full_matrices=False)
        n = vh[-1, :]
        n = n / (np.linalg.norm(n) + 1e-9)
        # Ensure normal roughly faces the camera (positive Z away from camera)
        if n[2] > 0:
            n = -n
        return c, n


# ------------ convenience ---------------

def pinhole_from_fov(width: int, height: int, fov_deg: float, near: float, far: float) -> CameraIntrinsics:
    """
    Build a plausible pinhole intrinsics model from a symmetric vertical FOV.
    If your camera uses horizontal FOV, adjust accordingly.
    """
    fov = math.radians(float(fov_deg))
    fy = 0.5 * height / math.tan(0.5 * fov)
    fx = fy
    cx = 0.5 * (width - 1)
    cy = 0.5 * (height - 1)
    return CameraIntrinsics(width, height, fx, fy, cx, cy, near, far)
