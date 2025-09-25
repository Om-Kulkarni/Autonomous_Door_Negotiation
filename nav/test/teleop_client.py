#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
client.py (Python 3)
Keyboard teleop:
  Base:   w/s/a/d, '+' (x2 speed), '-' (/2 speed), space/x stop
  Torso:  t (up), g (down), ']' (x2 torso step), '[' (/2 torso step)
  Gripper:o (open), p (close), '}' (x2 gripper step), '{' (/2 gripper step)
  q to quit

Protocol (per frame, big-endian): >ffff
  linear.x, angular.z, torso_lift (m), gripper_finger_pos (m)
"""

import socket
import struct
import sys
import time
import threading

# ---------- Config ----------
SERVER_IP = '10.68.0.1'   # <-- set to the robot's IP
SERVER_PORT = 65433
SEND_HZ = 20.0

# Base speeds
LIN_SPEED = 0.2   # m/s
ANG_SPEED = 0.6   # rad/s
LIN_MAX = 1.5
ANG_MAX = 3.0
LIN_MIN = 0.01
ANG_MIN = 0.05

# Torso (absolute position in meters)
TORSO_MIN = 0.00
TORSO_MAX = 0.35
TORSO_STEP = 0.01    # increment per keypress
TORSO_STEP_MIN = 0.0025
TORSO_STEP_MAX = 0.05
TORSO_DEFAULT = 0.30

# Gripper finger joints (absolute position in meters, symmetric L/R)
GRIP_MIN = 0.00
GRIP_MAX = 0.045
GRIP_STEP = 0.005
GRIP_STEP_MIN = 0.001
GRIP_STEP_MAX = 0.02
GRIP_DEFAULT = 0.045
# ----------------------------

# Cross-platform non-blocking key input
if sys.platform.startswith('win'):
    import msvcrt
    def get_key():
        if msvcrt.kbhit():
            ch = msvcrt.getch()
            if ch in (b'\x00', b'\xe0'):
                _ = msvcrt.getch()
                return None
            try:
                return ch.decode('utf-8')
            except Exception:
                return None
        return None
else:
    import termios
    import tty
    import select

    def get_key():
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            return sys.stdin.read(1)
        return None

    class RawTerminal(object):
        def __enter__(self):
            self.fd = sys.stdin.fileno()
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            return self
        def __exit__(self, exc_type, exc_value, traceback):
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

HELP = """
ASWD + torso & gripper teleop (streaming at {hz:.0f} Hz)

BASE:
  w : forward          s : backward
  a : turn left (CCW)  d : turn right (CW)
  space / x : STOP
  + : DOUBLE base speeds (x2)
  - : HALVE  base speeds (/2)

TORSO (absolute position, meters):
  t : raise torso      g : lower torso
  ] : DOUBLE torso step     [ : HALVE torso step

GRIPPER (finger joints, absolute position, meters):
  o : open gripper     p : close gripper
  }} : DOUBLE gripper step   {{ : HALVE gripper step

q : quit
""".format(hz=SEND_HZ)

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

class TeleopClient(object):
    def __init__(self, ip, port):
        self.addr = (ip, port)
        self.sock = None

        # base
        self.lin = 0.0
        self.ang = 0.0
        self.lin_speed = LIN_SPEED
        self.ang_speed = ANG_SPEED

        # torso / gripper absolute targets
        self.torso = TORSO_DEFAULT
        self.torso_step = TORSO_STEP
        self.grip = GRIP_DEFAULT
        self.grip_step = GRIP_STEP

        self.running = True
        self._lock = threading.Lock()

    def connect(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5.0)
        s.connect(self.addr)
        s.settimeout(None)
        self.sock = s
        print("Connected to {}:{}".format(*self.addr))
        self._print_status()

    def _print_status(self):
        print("[base speed] lin: {:.3f} m/s, ang: {:.3f} rad/s | "
              "[torso] {:.3f} m (step {:.3f}) | [grip] {:.3f} m (step {:.3f})"
              .format(self.lin_speed, self.ang_speed, self.torso, self.torso_step, self.grip, self.grip_step))

    def sender_loop(self):
        period = 1.0 / SEND_HZ
        pack = struct.Struct('>ffff').pack
        next_t = time.time()
        while self.running:
            with self._lock:
                frame = pack(self.lin, self.ang, self.torso, self.grip)
            try:
                self.sock.sendall(frame)
            except Exception as e:
                print("\n[ERROR] Lost connection to server: {}".format(e))
                self.running = False
                break
            next_t += period
            sleep = next_t - time.time()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.time()

    def handle_key(self, key):
        if not key:
            return
        k = key.lower()
        with self._lock:
            # ---- base ----
            if k == 'w':
                self.lin =  self.lin_speed
            elif k == 's':
                self.lin = -self.lin_speed
            elif k == 'a':
                self.ang =  self.ang_speed
            elif k == 'd':
                self.ang = -self.ang_speed
            elif k in (' ', 'x'):
                self.lin = 0.0
                self.ang = 0.0
            elif k == '+':
                self.lin_speed = clamp(self.lin_speed * 2.0, LIN_MIN, LIN_MAX)
                self.ang_speed = clamp(self.ang_speed * 2.0, ANG_MIN, ANG_MAX)
                print("[base speed] lin: {:.3f}, ang: {:.3f}".format(self.lin_speed, self.ang_speed))
            elif k == '-':
                self.lin_speed = clamp(self.lin_speed / 2.0, LIN_MIN, LIN_MAX)
                self.ang_speed = clamp(self.ang_speed / 2.0, ANG_MIN, ANG_MAX)
                print("[base speed] lin: {:.3f}, ang: {:.3f}".format(self.lin_speed, self.ang_speed))

            # ---- torso absolute target ----
            elif k == 't':
                self.torso = clamp(self.torso + self.torso_step, TORSO_MIN, TORSO_MAX)
                print("[torso] {:.3f} m".format(self.torso))
            elif k == 'g':
                self.torso = clamp(self.torso - self.torso_step, TORSO_MIN, TORSO_MAX)
                print("[torso] {:.3f} m".format(self.torso))
            elif k == ']':
                self.torso_step = clamp(self.torso_step * 2.0, TORSO_STEP_MIN, TORSO_STEP_MAX)
                print("[torso step] {:.3f} m".format(self.torso_step))
            elif k == '[':
                self.torso_step = clamp(self.torso_step / 2.0, TORSO_STEP_MIN, TORSO_STEP_MAX)
                print("[torso step] {:.3f} m".format(self.torso_step))

            # ---- gripper absolute target ----
            elif k == 'o':
                self.grip = clamp(self.grip + self.grip_step, GRIP_MIN, GRIP_MAX)
                print("[gripper] {:.3f} m".format(self.grip))
            elif k == 'p':
                self.grip = clamp(self.grip - self.grip_step, GRIP_MIN, GRIP_MAX)
                print("[gripper] {:.3f} m".format(self.grip))
            elif k == '}':
                self.grip_step = clamp(self.grip_step * 2.0, GRIP_STEP_MIN, GRIP_STEP_MAX)
                print("[gripper step] {:.3f} m".format(self.grip_step))
            elif k == '{':
                self.grip_step = clamp(self.grip_step / 2.0, GRIP_STEP_MIN, GRIP_STEP_MAX)
                print("[gripper step] {:.3f} m".format(self.grip_step))

            elif k == 'q':
                self.running = False

    def stop(self):
        with self._lock:
            self.lin = 0.0
            self.ang = 0.0
        try:
            self.sock.sendall(struct.pack('>ffff', 0.0, 0.0, self.torso, self.grip))
        except Exception:
            pass

def main():
    print(HELP)
    client = TeleopClient(SERVER_IP, SERVER_PORT)
    try:
        client.connect()
    except Exception as e:
        print("[ERROR] Could not connect to server at {}:{}: {}".format(SERVER_IP, SERVER_PORT, e))
        sys.exit(1)

    sender = threading.Thread(target=client.sender_loop)
    sender.daemon = True
    sender.start()

    try:
        if sys.platform.startswith('win'):
            while client.running:
                key = get_key()
                if key:
                    client.handle_key(key)
                time.sleep(0.01)
        else:
            with RawTerminal():
                while client.running:
                    key = get_key()
                    if key:
                        client.handle_key(key)
                    time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        client.stop()
        client.running = False
        try:
            client.sock.close()
        except Exception:
            pass
        print("\nExiting teleop.")

if __name__ == '__main__':
    main()
