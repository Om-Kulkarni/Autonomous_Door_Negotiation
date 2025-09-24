import math
import time
from collections import deque

import numpy as np
import cv2
import pyrealsense2 as rs

# ===========================
# User-tunable parameters
# ===========================
DEPTH_WIDTH, DEPTH_HEIGHT, FPS = 848, 480, 30     # D455-friendly mode
PIXEL_STRIDE = 4                                  # sample every Nth pixel to save CPU
MAX_RANGE_M = 6.0                                 # ignore returns beyond this
MIN_RANGE_M = 0.2                                 # ignore too-close noise
CAMERA_HEIGHT_M = 0.15                            # camera height above ground (m) (low mount)
CAMERA_PITCH_DOWN_DEG = 10.0                      # camera pitched downward around robot Y (deg)

# Grid settings (egocentric grid, robot at center bottom)
RES_M = 0.05                                      # cell resolution (m/cell)
GRID_W_M, GRID_H_M = 12.0, 12.0                   # grid extents (forward/back and left/right)
GRID_W = int(GRID_W_M / RES_M)
GRID_H = int(GRID_H_M / RES_M)

# Log-odds / inverse sensor model
L0 = 0.0       # prior log-odds (p=0.5)
L_OCC = 0.85   # evidence for occupied hit
L_FREE = 0.4   # evidence for free along ray
L_MIN, L_MAX = -4.0, 4.0

# Ground removal
GROUND_CLEARANCE = 0.05   # keep obstacles whose height above ground >= this (m)

# Visualization
DRAW_SCALE = 2     # upscaling when showing the grid
SHOW_WINDOW = True

# ===========================
# Helpers: frames & transforms
# ===========================
def rot_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[ c, 0,  s],
                     [ 0, 1,  0],
                     [-s, 0,  c]], dtype=np.float32)

# RealSense camera coords: +x right, +y down, +z forward (Intel docs)
# Robot frame we’ll use:  +x forward, +y left, +z up
# R_cam2robot maps camera->robot when there is NO tilt.
# Mapping: x_r =  z_c ; y_r = -x_c ; z_r = -y_c
R_cam2robot = np.array([[0,  0, 1],
                        [-1, 0, 0],
                        [0, -1, 0]], dtype=np.float32)

R_pitch = rot_y(np.deg2rad(CAMERA_PITCH_DOWN_DEG))   # downward pitch around robot Y
t_cam_in_robot = np.array([0.0, 0.0, CAMERA_HEIGHT_M], dtype=np.float32)

def cam_to_robot(Pc):
    """Pc: (...,3) points in camera frame -> robot frame (x fwd, y left, z up) with pitch + translation."""
    # First to robot (no tilt), then apply pitch in robot frame, then translate
    Pr_no_pitch = Pc @ R_cam2robot.T
    Pr = Pr_no_pitch @ R_pitch.T + t_cam_in_robot
    return Pr

# ===========================
# Occupancy Grid
# ===========================
class OccupancyGrid:
    def __init__(self, w, h, res_m):
        self.w, self.h, self.res = w, h, res_m
        self.logodds = np.zeros((h, w), dtype=np.float32)
        # We'll put robot at (x=middle, y=bottom margin)
        self.cx = w // 2
        self.cy = h - 1  # bottom row is robot position

    def world_to_grid(self, xr, yr):
        """xr forward (m), yr left (m) -> (gx, gy) grid idx (int)"""
        gx = int(self.cx + yr / self.res)
        gy = int(self.cy - xr / self.res)
        return gx, gy

    def bresenham(self, x0, y0, x1, y1):
        """Grid ray from (x0,y0) to (x1,y1) inclusive; returns list of (x,y)."""
        points = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        while True:
            points.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy
        return points

    def update_ray(self, gx_hit, gy_hit, free_until_last=True):
        """Update along a ray from robot cell to endpoint."""
        x0, y0 = self.cx, self.cy
        pts = self.bresenham(x0, y0, gx_hit, gy_hit)
        if len(pts) == 0:
            return
        # free cells: all except final cell
        for (x, y) in pts[:-1] if free_until_last else pts:
            if 0 <= x < self.w and 0 <= y < self.h:
                self.logodds[y, x] = np.clip(self.logodds[y, x] - L_FREE, L_MIN, L_MAX)
        # occupied at hit
        xh, yh = pts[-1]
        if 0 <= xh < self.w and 0 <= yh < self.h:
            self.logodds[yh, xh] = np.clip(self.logodds[yh, xh] + L_OCC, L_MIN, L_MAX)

    def to_uint8(self):
        """Convert to 0..255 for display: occupied→black, free→white, unknown→mid."""
        # p = 1 - 1/(1+exp(l)) ; but simpler vis: map log-odds to [0,255]
        p = 1.0 - 1.0 / (1.0 + np.exp(self.logodds))  # p(occupied)
        img = (255.0 * (1.0 - p)).astype(np.uint8)    # occupied dark, free bright
        # draw robot
        cv2.circle(img, (self.cx, self.cy), 2, 128, -1)
        return img

# ===========================
# RealSense setup
# ===========================
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, FPS)
profile = pipe.start(cfg)

# Depth scale (meters per unit in Z16)
# If we use depth_frame.get_distance(x,y), it's already in meters.
# For bulk numpy, multiply raw 16U values by depth_scale.
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Get intrinsics
depth_stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()
intr = depth_stream.get_intrinsics()
fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy

# Precompute a grid of pixel coordinates (stride sampling)
uu = np.arange(0, DEPTH_WIDTH, PIXEL_STRIDE, dtype=np.float32)
vv = np.arange(0, DEPTH_HEIGHT, PIXEL_STRIDE, dtype=np.float32)
U, V = np.meshgrid(uu, vv)
# Unit "directions" in camera frame scaled by depth later:
# X = (u - cx)/fx * Z ; Y = (v - cy)/fy * Z ; Z = Z
Xnorm = (U - cx) / fx
Ynorm = (V - cy) / fy

grid = OccupancyGrid(GRID_W, GRID_H, RES_M)

try:
    fps_hist = deque(maxlen=30)
    while True:
        t0 = time.time()
        frames = pipe.wait_for_frames()
        depth = frames.get_depth_frame()
        if not depth:
            continue

        # Get raw depth as numpy
        depth_data = np.asanyarray(depth.get_data())  # uint16
        # Sampled depth map
        D = depth_data[0:DEPTH_HEIGHT:PIXEL_STRIDE, 0:DEPTH_WIDTH:PIXEL_STRIDE].astype(np.float32) * depth_scale

        # Range filter
        valid = (D > MIN_RANGE_M) & (D < MAX_RANGE_M)

        # Back-project to 3D camera frame (vectorized)
        Zc = D
        Xc = Xnorm * Zc
        Yc = Ynorm * Zc
        # Stack and mask invalid
        Pc = np.stack([Xc, Yc, Zc], axis=-1)
        Pc = Pc[valid]

        if Pc.shape[0] == 0:
            continue

        # Transform to robot frame; translate so z=0 is ground
        Pr = cam_to_robot(Pc)

        # Remove ground points (keep obstacles above threshold)
        # Robot frame: z up. Keep points with z >= GROUND_CLEARANCE and in front of robot (x>0)
        keep = (Pr[:, 2] >= GROUND_CLEARANCE) & (Pr[:, 0] > 0.0)
        Pr = Pr[keep]

        # Update occupancy grid by ray-casting to each hit point
        # (Subsample further if very dense)
        if Pr.shape[0] > 5000:
            Pr = Pr[::int(Pr.shape[0] / 5000) + 1]

        for p in Pr:
            xr, yr = float(p[0]), float(p[1])
            gx, gy = grid.world_to_grid(xr, yr)
            # Only update if endpoint is inside the grid
            if 0 <= gx < grid.w and 0 <= gy < grid.h:
                grid.update_ray(gx, gy, free_until_last=True)

        # Visualize
        if SHOW_WINDOW:
            img = grid.to_uint8()
            vis = cv2.resize(img, (img.shape[1]*DRAW_SCALE, img.shape[0]*DRAW_SCALE),
                             interpolation=cv2.INTER_NEAREST)
            cv2.imshow("OGM (white=free, black=occupied, gray=unknown)", vis)
            key = cv2.waitKey(1)
            if key == 27 or key == ord('q'):
                break

        # crude FPS
        fps = 1.0 / max(1e-6, (time.time() - t0))
        fps_hist.append(fps)
        if len(fps_hist) == fps_hist.maxlen:
            avg = sum(fps_hist) / len(fps_hist)
            # print(f"FPS ~ {avg:.1f}")

finally:
    pipe.stop()
    if SHOW_WINDOW:
        cv2.destroyAllWindows()
