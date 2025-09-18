#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from collections import deque

import numpy as np
import cv2
import pyrealsense2 as rs
import matplotlib
# Prefer Tk; fall back safely if missing
try:
    import tkinter  # noqa
    matplotlib.use("TkAgg")
except Exception:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


# ============== Math helpers (quaternions & rotations) ==============
def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dtype=np.float64)

def quat_normalize(q):
    n = np.linalg.norm(q)
    return q if n == 0 else q/n

def quat_from_omega_dt(omega, dt):
    theta = np.linalg.norm(omega) * dt
    if theta < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    axis = omega / (np.linalg.norm(omega) + 1e-12)
    half = 0.5 * theta
    return quat_normalize(np.array([math.cos(half), *(math.sin(half) * axis)], dtype=np.float64))

def quat_to_rotmat(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
        [2*(x*y + z*w),   1-2*(x*x+z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w),   2*(y*z + x*w),   1-2*(x*x+y*y)]
    ], dtype=np.float64)

def quat_from_two_vectors(a, b):
    """Shortest-arc quaternion rotating vector a to b (both 3D)."""
    a = a / (np.linalg.norm(a) + 1e-12)
    b = b / (np.linalg.norm(b) + 1e-12)
    v = np.cross(a, b)
    w = 1.0 + np.dot(a, b)
    if w < 1e-6:  # opposite vectors
        # pick orthogonal axis
        axis = np.array([1, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1, 0])
        v = np.cross(a, axis)
        w = 0.0
    q = np.array([w, v[0], v[1], v[2]], dtype=np.float64)
    return quat_normalize(q)


# ============== Mahony filter (IMU-only attitude) ==============
class Mahony:
    def __init__(self, kp=1.0, ki=0.05):
        self.kp = kp
        self.ki = ki
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.int_err = np.zeros(3, dtype=np.float64)

    def update_imu(self, gyr, acc, dt):
        # Normalize accelerometer (gravity direction)
        a = acc
        an = np.linalg.norm(a)
        if an < 1e-6:
            # accel unusable -> pure gyro
            dq = quat_from_omega_dt(gyr, dt)
            self.q = quat_normalize(quat_mul(self.q, dq))
            return self.q

        a = a / an

        # Estimated gravity in body from current attitude:
        R = quat_to_rotmat(self.q)
        g_est_b = R.T @ np.array([0, 0, 1.0])  # world g=(0,0,1) -> body

        # Error is cross between measured gravity and estimated gravity
        err = np.cross(g_est_b, a)

        # Integrator (bias compensation)
        self.int_err += err * dt * self.ki

        # Corrected gyro
        omega = gyr + self.kp*err + self.int_err

        # Integrate
        dq = quat_from_omega_dt(omega, dt)
        self.q = quat_normalize(quat_mul(self.q, dq))
        return self.q


# ============== Simple ZUPT Kalman (state: p, v) ==============
class ZUPTKalman:
    """
    Constant-acceleration model on position/velocity with process noise q_acc.
    State x = [p(3), v(3)]^T, dim=6. When stationary is detected, apply measurement v=0.
    """
    def __init__(self, q_acc=0.5, r_zupt=0.01):
        self.x = np.zeros(6)        # [px,py,pz,vx,vy,vz]
        self.P = np.eye(6) * 1e-3
        self.q_acc = q_acc
        self.r_zupt = r_zupt

    def predict(self, a_w, dt):
        # a_w is world-frame specific force (acc minus gravity) [m/s^2]
        F = np.eye(6)
        F[0,3] = dt; F[1,4] = dt; F[2,5] = dt

        B = np.zeros((6,3))
        B[3,0] = dt; B[4,1] = dt; B[5,2] = dt  # v_k+1 = v_k + a*dt
        C = np.zeros((6,3))
        C[0,0] = 0.5*dt*dt; C[1,1] = 0.5*dt*dt; C[2,2] = 0.5*dt*dt  # p += 0.5*a*dt^2

        # Process noise
        Qv = (self.q_acc**2) * np.eye(3)
        Q = B @ Qv @ B.T + C @ Qv @ C.T

        # Predict
        self.x = F @ self.x + C @ a_w + B @ a_w  # combine p and v updates
        self.P = F @ self.P @ F.T + Q

    def zupt_update(self):
        # Measurement: z = v = 0
        H = np.zeros((3,6))
        H[0,3] = 1; H[1,4] = 1; H[2,5] = 1
        R = (self.r_zupt**2) * np.eye(3)
        z = np.zeros(3)

        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

    @property
    def p(self): return self.x[:3]
    @property
    def v(self): return self.x[3:]


# ============== RealSense helpers ==============
def pick_imu_profiles(profile):
    dev = profile.get_device()
    acc_fps, gyr_fps = 62, 200
    accs, gyrs = [], []
    for s in dev.sensors:
        for sp in s.get_stream_profiles():
            st, fps = sp.stream_type(), sp.fps()
            if st == rs.stream.accel: accs.append(fps)
            if st == rs.stream.gyro:  gyrs.append(fps)
    if accs: acc_fps = sorted(accs, reverse=True)[0]
    if gyrs: gyr_fps = sorted(gyrs, reverse=True)[0]
    return int(round(acc_fps)), int(round(gyr_fps))


# ============== Stationary detection (for ZUPT) ==============
class StationaryDetector:
    def __init__(self, acc_thresh=0.12, gyro_thresh=0.10, win=20):
        """
        acc_thresh: | ||acc|| - g | < acc_thresh  (m/s^2)
        gyro_thresh: ||gyro|| < gyro_thresh       (rad/s)
        win: consecutive frames required
        """
        self.acc_thresh = acc_thresh
        self.gyro_thresh = gyro_thresh
        self.win = win
        self.buf = deque(maxlen=win)
        self.g = 9.81

    def update(self, acc_b, gyro_b):
        cond = (abs(np.linalg.norm(acc_b) - self.g) < self.acc_thresh) and \
               (np.linalg.norm(gyro_b) < self.gyro_thresh)
        self.buf.append(1 if cond else 0)
        return sum(self.buf) == self.win


# ============== Main ==============
def main():
    # ---------- Configure RealSense ----------
    pipeline = rs.pipeline()
    cfg = rs.config()

    wrapper = rs.pipeline_wrapper(pipeline)
    pre = cfg.resolve(wrapper)
    acc_fps, gyr_fps = pick_imu_profiles(pre)  # prefer highest valid rates

    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, acc_fps)
    cfg.enable_stream(rs.stream.gyro,  rs.format.motion_xyz32f, gyr_fps)

    profile = pipeline.start(cfg)
    align = rs.align(rs.stream.color)

    # ---------- Calibration (first N samples while stationary) ----------
    N_CAL = 1000
    print(f"[Calib] Collecting {N_CAL} IMU samples. Keep the camera still...")
    gyro_samples = []
    accel_samples = []
    g_world = np.array([0, 0, 9.81], dtype=np.float64)

    while len(gyro_samples) < N_CAL or len(accel_samples) < N_CAL:
        frames = pipeline.wait_for_frames()
        gf = frames.first_or_default(rs.stream.gyro)
        af = frames.first_or_default(rs.stream.accel)
        if gf:
            g = gf.as_motion_frame().get_motion_data()
            gyro_samples.append([g.x, g.y, g.z])
        if af:
            a = af.as_motion_frame().get_motion_data()
            accel_samples.append([a.x, a.y, a.z])

    gyro_bias = np.mean(np.array(gyro_samples), axis=0)
    accel_mean = np.mean(np.array(accel_samples), axis=0)
    # Initial attitude: rotate measured accel (body) to +Z (world)
    q_wb = quat_from_two_vectors(accel_mean, g_world)
    q_wb = quat_normalize(q_wb)
    print(f"[Calib] gyro_bias = {gyro_bias}, |accel_mean| = {np.linalg.norm(accel_mean):.3f}")

    # ---------- Filters / state ----------
    mahony = Mahony(kp=1.2, ki=0.02)
    mahony.q = q_wb.copy()
    zuptkf = ZUPTKalman(q_acc=0.8, r_zupt=0.03)
    stat = StationaryDetector(acc_thresh=0.12, gyro_thresh=0.10, win=20)

    # ---------- UI ----------
    win = "D435i | Color (left) + Depth (right) [640x480] — 'q' to quit"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    plt.ion()
    fig = plt.figure("ZUPT-aided IMU DR — D435i")
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]"); ax.set_zlabel("Z [m]")
    traj_line, = ax.plot([], [], [], lw=2)
    ax.scatter([0],[0],[0], s=30)

    # ---------- Loop ----------
    last_gyro_ts = None
    last_acc_ts = None
    p_hist = []

    try:
        while True:
            frames = pipeline.wait_for_frames()

            # --- IMU ---
            gf = frames.first_or_default(rs.stream.gyro)
            af = frames.first_or_default(rs.stream.accel)

            if gf:
                md = gf.as_motion_frame().get_motion_data()
                omega_b = np.array([md.x, md.y, md.z]) - gyro_bias
                t = gf.get_timestamp()*1e-3
                if last_gyro_ts is not None:
                    dt_g = max(1e-6, t - last_gyro_ts)
                else:
                    dt_g = 0.0
                last_gyro_ts = t
            else:
                omega_b = None
                dt_g = 0.0

            if af:
                md = af.as_motion_frame().get_motion_data()
                acc_b = np.array([md.x, md.y, md.z])
                t = af.get_timestamp()*1e-3
                if last_acc_ts is not None:
                    dt_a = max(1e-6, t - last_acc_ts)
                else:
                    dt_a = 0.0
                last_acc_ts = t
            else:
                acc_b = None
                dt_a = 0.0

            # --- Attitude & INS ---
            if omega_b is not None and acc_b is not None and dt_g > 0:
                # Mahony attitude update
                q_wb = mahony.update_imu(omega_b, acc_b, dt_g)
                R_wb = quat_to_rotmat(q_wb)

            if acc_b is not None and dt_a > 0:
                # World specific force (gravity-compensated)
                a_w = R_wb @ acc_b - np.array([0,0,9.81])
                zuptkf.predict(a_w, dt_a)

                # Stationary detection -> ZUPT
                if stat.update(acc_b, omega_b if omega_b is not None else np.zeros(3)):
                    zuptkf.zupt_update()

                p = zuptkf.p.copy()
                p_hist.append(p)

            # --- Color + Depth panel ---
            aligned = align.process(frames)
            d = aligned.get_depth_frame()
            c = aligned.get_color_frame()
            if d and c:
                depth = np.asanyarray(d.get_data())
                color = np.asanyarray(c.get_data())
                depth_viz = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
                color_ = cv2.resize(color, (320,480), interpolation=cv2.INTER_AREA)
                depth_ = cv2.resize(depth_viz, (320,480), interpolation=cv2.INTER_NEAREST)
                panel = np.hstack([color_, depth_])

                # HUD
                h, w = depth.shape
                dist = d.get_distance(w//2, h//2)
                vel = np.linalg.norm(zuptkf.v)
                cv2.putText(panel, f"Depth@center: {dist:.3f} m | |v|: {vel:.2f} m/s",
                            (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
                cv2.imshow(win, panel)

            # --- Trajectory plot ---
            if p_hist:
                T = np.array(p_hist)
                traj_line.set_data(T[:,0], T[:,1])
                traj_line.set_3d_properties(T[:,2])
                cx, cy, cz = T[-1]
                span = 2.0
                ax.set_xlim(cx-span, cx+span)
                ax.set_ylim(cy-span, cy+span)
                ax.set_zlim(cz-0.5*span, cz+1.5*span)
                plt.pause(0.001)

            # Quit
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        plt.ioff(); plt.show()


if __name__ == "__main__":
    main()
