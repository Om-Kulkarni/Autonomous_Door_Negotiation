#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
yaw_goal_client.py
Host-side (Python 3) in-place rotation controller for TIAGo (diff-drive).
Sends Twist-like JSON {"vx":..., "wz":...} to the robot TCP server.

Usage:
  python3 yaw_goal_client.py --robot-ip 10.68.0.1 --port 5005 --angle 1.57
  # degrees input:
  python3 yaw_goal_client.py --angle 90 --deg
  # interactive (no --angle): it will prompt repeatedly
"""

import argparse
import json
import math
import socket
import sys
import time

DEFAULT_IP = "10.68.0.1"
DEFAULT_PORT = 5005

WHEELBASE_M = 0.508  # 20 inches (helper only)

# Angular motion limits
W_MAX = 1.0      # rad/s
ALPHA_MAX = 1.6  # rad/s^2
V_MAX = 0.0      # keep linear 0 for in-place rotation
SEND_HZ = 40.0
STOP_HOLD_SEC = 0.15

def clamp(v, lo, hi):
    return min(max(v, lo), hi)

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

def trapezoid_timings_angle(theta, w_max, alpha_max):
    """
    Return (t_acc, t_cruise, t_dec, w_peak) for rotating by |theta|.
    """
    A = abs(theta)
    t_acc = w_max / alpha_max
    a_acc = 0.5 * alpha_max * t_acc * t_acc  # angle covered in accel
    if 2.0 * a_acc >= A:
        # triangle
        w_peak = math.sqrt(A * alpha_max)
        t_acc = w_peak / alpha_max
        t_cruise = 0.0
        t_dec = t_acc
        return t_acc, t_cruise, t_dec, w_peak
    else:
        a_cruise = A - 2.0 * a_acc
        t_cruise = a_cruise / w_max
        t_dec = t_acc
        w_peak = w_max
        return t_acc, t_cruise, t_dec, w_peak

def run_one_rotation(s, angle_rad):
    """Rotate by angle_rad (rad). Positive = CCW (left)."""
    if abs(angle_rad) < 1e-6:
        stream_zero(s)
        return

    sign = 1.0 if angle_rad >= 0.0 else -1.0
    t_acc, t_cruise, t_dec, w_peak = trapezoid_timings_angle(angle_rad, W_MAX, ALPHA_MAX)

    dt = 1.0 / SEND_HZ
    yaw_prog = 0.0
    t0 = time.time()
    last = t0

    while True:
        now = time.time()
        tau = now - t0
        dtt = now - last
        last = now

        # phase
        if tau <= t_acc:
            w = ALPHA_MAX * tau
        elif tau <= (t_acc + t_cruise):
            w = w_peak
        elif tau <= (t_acc + t_cruise + t_dec):
            t_d = tau - (t_acc + t_cruise)
            w = max(w_peak - ALPHA_MAX * t_d, 0.0)
        else:
            break

        wz = sign * clamp(w, 0.0, W_MAX)
        yaw_prog += wz * dtt

        # goal reached?
        if abs(yaw_prog) >= abs(angle_rad) * 0.999:
            break

        pkt = (json.dumps({"vx": 0.0, "wz": wz}) + "\n").encode("utf-8")
        s.sendall(pkt)

        sleep_left = dt - (time.time() - now)
        if sleep_left > 0:
            time.sleep(sleep_left)

    # stop
    stream_zero(s)

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-ip", default=DEFAULT_IP)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--angle", type=float, default=None, help="Rotation amount (radians by default)")
    ap.add_argument("--deg", action="store_true", help="Interpret --angle in degrees")
    return ap.parse_args()

def main():
    args = parse_args()
    print("Connecting to {}:{} ...".format(args.robot_ip, args.port))
    s = connect(args.robot_ip, args.port)
    print("Connected.")

    try:
        if args.angle is not None:
            ang = math.radians(args.angle) if args.deg else args.angle
            run_one_rotation(s, ang)
            print("Done: rotated {:.3f} rad".format(ang))
        else:
            while True:
                line = input("Enter angle (rad; prefix 'd ' for degrees), or 'q': ").strip()
                if line.lower() in ("q", "quit", "exit"):
                    break
                try:
                    if line.lower().startswith("d "):
                        ang = math.radians(float(line.split()[1]))
                    else:
                        ang = float(line)
                except Exception:
                    print("Not a valid number.")
                    continue
                run_one_rotation(s, ang)
                print("Done: rotated {:.3f} rad".format(ang))
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
