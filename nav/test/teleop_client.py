#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
client.py (Python 3)
Laptop-side teleop client: ASWD to steer, '+' to double speed, '-' to halve speed, 'space' or 'x' to stop, 'q' to quit.
Streams (linear.x, angular.z) as big-endian floats to the server at 20 Hz.
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
# initial speeds
LIN_SPEED = 0.2   # m/s
ANG_SPEED = 0.6   # rad/s
LIN_MAX = 1.5
ANG_MAX = 3.0
LIN_MIN = 0.01
ANG_MIN = 0.05
# ----------------------------

# Cross-platform non-blocking key input
if sys.platform.startswith('win'):
    import msvcrt
    def get_key():
        if msvcrt.kbhit():
            ch = msvcrt.getch()
            # handle arrow keys / extended keys consuming second byte
            if ch in (b'\x00', b'\xe0'):
                _ = msvcrt.getch()
                return None
            try:
                return ch.decode('utf-8').lower()
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
            return sys.stdin.read(1).lower()
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
ASWD teleop (streaming at {hz:.0f} Hz)
  w : forward
  s : backward
  a : turn left (CCW)
  d : turn right (CW)
  space / x : STOP
  + : DOUBLE both linear & angular speeds (x2)
  - : HALVE  both linear & angular speeds (/2)
  q : quit

Current speeds are shown live (m/s, rad/s). Commands persist until changed.
""".format(hz=SEND_HZ)

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

class TeleopClient(object):
    def __init__(self, ip, port):
        self.addr = (ip, port)
        self.sock = None
        self.lin = 0.0
        self.ang = 0.0
        self.lin_speed = LIN_SPEED
        self.ang_speed = ANG_SPEED
        self.running = True
        self._lock = threading.Lock()

    def connect(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5.0)
        s.connect(self.addr)
        s.settimeout(None)
        self.sock = s
        print("Connected to {}:{}".format(*self.addr))

    def sender_loop(self):
        period = 1.0 / SEND_HZ
        pack = struct.Struct('>ff').pack
        next_t = time.time()
        while self.running:
            with self._lock:
                lin, ang = self.lin, self.ang
            try:
                self.sock.sendall(pack(lin, ang))
            except Exception as e:
                print("\n[ERROR] Lost connection to server: {}".format(e))
                self.running = False
                break
            next_t += period
            sleep = next_t - time.time()
            if sleep > 0:
                time.sleep(sleep)
            else:
                # if we're behind, reset the schedule
                next_t = time.time()

    def update_velocity(self, key):
        with self._lock:
            if key == 'w':
                self.lin =  self.lin_speed
            elif key == 's':
                self.lin = -self.lin_speed
            elif key == 'a':
                self.ang =  self.ang_speed
            elif key == 'd':
                self.ang = -self.ang_speed
            elif key in (' ', 'x'):
                self.lin = 0.0
                self.ang = 0.0
            elif key == '+':
                self.lin_speed = clamp(self.lin_speed * 2.0, LIN_MIN, LIN_MAX)
                self.ang_speed = clamp(self.ang_speed * 2.0, ANG_MIN, ANG_MAX)
                print("[speed] lin: {:.3f} m/s, ang: {:.3f} rad/s".format(self.lin_speed, self.ang_speed))
            elif key == '-':
                self.lin_speed = clamp(self.lin_speed / 2.0, LIN_MIN, LIN_MAX)
                self.ang_speed = clamp(self.ang_speed / 2.0, ANG_MIN, ANG_MAX)
                print("[speed] lin: {:.3f} m/s, ang: {:.3f} rad/s".format(self.lin_speed, self.ang_speed))
            elif key == 'q':
                self.running = False

    def stop(self):
        with self._lock:
            self.lin = 0.0
            self.ang = 0.0
        try:
            # send one last zero
            self.sock.sendall(struct.pack('>ff', 0.0, 0.0))
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
                    client.update_velocity(key)
                time.sleep(0.01)
        else:
            with RawTerminal():
                while client.running:
                    key = get_key()
                    if key:
                        client.update_velocity(key)
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
