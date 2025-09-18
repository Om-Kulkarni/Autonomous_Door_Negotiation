#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pos_goal_client.py
Host-side (Python 3) linear distance controller for a diff-drive TIAGo.
Sends Twist-like JSON {"vx":..., "wz":...} over TCP to the robot's Python2 server.

Usage:
  python3 pos_goal_client.py --robot-ip 10.68.0.1 --port 5005 --meters 1.2
  # or interactive (no --meters): it will prompt for distances repeatedly

Control:
  - Trapezoidal speed profile with jerk-free phase changes
  - Internal dead-reckoned progress (no robot-side feedback required)
  - Wheelbase used for helper conversions only (v,w -> wheel speeds), not required by cmd_vel

Safety:
  - E-stop on Ctrl+C (sends zeros)
  - Hard clamps on v, w
"""

import argparse
import json
import math
import socket
import sys
import time

# ---- Robot/network config ----
DEFAULT_IP = "10.68.0.1"
DEFAULT_PORT = 5005

# ---- Geometry (diff-drive) ----
WHEELBASE_M = 0.508  # 20 inches

# ---- Motion limits (tune as needed) ----
V_MAX = 0.35      # m/s    (linear speed limit)
A_MAX = 0.6       # m/s^2  (linear accel limit)
W_MAX = 1.0       # rad/s  (angular cap for safety; we won't rotate here)
SEND_HZ = 40.0    # command streaming rate
STOP_HOLD_SEC = 0.15  # send zeros a bit at the end

def clamp(v, lo, hi):
    return min(max(v, lo), hi)

def diffdrive_fwd_kin(v, w):
    """Return (v_l, v_r) wheel linear speeds for debugging/logging."""
    L = WHEELBASE_M
    v_l = v - (w * L / 2.0)
    v_r = v + (w * L / 2.0)
    return v_l, v_r

def connect(ip, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    s.connect((ip, port))
    return s

def stream_zero(s, duration=STOP_HOLD_SEC):
    t0 = time.time()
    pkt = (json.dumps({"vx": 0.0, "wz": 0.0}) + "\n").encode("utf-8")
    while time.time() - t0 < duration:
        s.sendall(pkt)
        time.sleep(1.0 / SEND_HZ)

def trapezoid_timings(dist, v_max, a_max):
    """
    Return (t_acc, t_cruise, t_dec, v_peak).
    If distance too short to reach v_max, it's a triangle (t_cruise=0).
    """
    D = abs(dist)
    t_acc = v_max / a_max
    d_acc = 0.5 * a_max * t_acc * t_acc  # distance covered during accel
    if 2.0 * d_acc >= D:
        # triangular profile
        v_peak = math.sqrt(D * a_max)
        t_acc = v_peak / a_max
        t_cruise = 0.0
        t_dec = t_acc
        return t_acc, t_cruise, t_dec, v_peak
    else:
        # trapezoidal profile
        d_cruise = D - 2.0 * d_acc
        t_cruise = d_cruise / v_max
        t_dec = t_acc
        v_peak = v_max
        return t_acc, t_cruise, t_dec, v_peak

def run_one_distance(s, meters):
    """Drive forward/backward by 'meters' then stop."""
    if abs(meters) < 1e-6:
        stream_zero(s)
        return

    sign = 1.0 if meters >= 0.0 else -1.0
    t_acc, t_cruise, t_dec, v_peak = trapezoid_timings(meters, V_MAX, A_MAX)
    t_total = t_acc + t_cruise + t_dec

    dt = 1.0 / SEND_HZ
    vx = 0.0
    wz = 0.0

    # Internal progress (dead-reckoned)
    x_prog = 0.0

    t0 = time.time()
    last = t0
    while True:
        now = time.time()
        tau = now - t0
        dtt = now - last
        last = now

        if tau <= t_acc:
            # accelerate
            v = A_MAX * tau
        elif tau <= (t_acc + t_cruise):
            # cruise
            v = v_peak
        elif tau <= (t_acc + t_cruise + t_dec):
            # decelerate
            t_d = tau - (t_acc + t_cruise)
            v = max(v_peak - A_MAX * t_d, 0.0)
        else:
            break

        vx = sign * clamp(v, 0.0, V_MAX)
        # update internal progress (integrate)
        x_prog += vx * dtt

        # goal check: if we've reached or exceeded the target, stop
        if abs(x_prog) >= abs(meters) * 0.999:
            break

        # send packet
        vx_cmd = clamp(vx, -V_MAX, V_MAX)
        wz_cmd = 0.0  # hold heading
        pkt = (json.dumps({"vx": vx_cmd, "wz": wz_cmd}) + "\n").encode("utf-8")
        s.sendall(pkt)

        # pace
        sleep_left = dt - (time.time() - now)
        if sleep_left > 0:
            time.sleep(sleep_left)

    # hard stop at goal
    stream_zero(s)

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-ip", default=DEFAULT_IP)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--meters", type=float, default=None,
                    help="Distance to move (m). If omitted, prompt interactively.")
    return ap.parse_args()

def main():
    args = parse_args()
    print("Connecting to {}:{} ...".format(args.robot_ip, args.port))
    s = connect(args.robot_ip, args.port)
    print("Connected.")

    try:
        if args.meters is not None:
            run_one_distance(s, args.meters)
            print("Done: moved {:.3f} m".format(args.meters))
        else:
            while True:
                line = input("Enter distance in meters (e.g., 0.5, -0.3) or 'q': ").strip()
                if line.lower() in ("q", "quit", "exit"):
                    break
                try:
                    d = float(line)
                except ValueError:
                    print("Not a number.")
                    continue
                run_one_distance(s, d)
                print("Done: moved {:.3f} m".format(d))
    except KeyboardInterrupt:
        print("\nE-stop!")
        stream_zero(s, duration=0.25)
    finally:
        try:
            s.close()
        except:
            pass

if __name__ == "__main__":
    main()
