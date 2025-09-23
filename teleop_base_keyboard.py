#!/usr/bin/env python3
"""
teleop_base_keyboard.py

Class-based keyboard teleop for TIAGo mobile base in PyBullet.
Sends planar base commands (vx [m/s], wz [rad/s]) to tiago_nav_bullet.py
over UDP localhost.

Controls (WASD / arrows):
  W / Up     : +vx
  S / Down   : -vx
  A / Left   : +wz (rotate CCW)
  D / Right  : -wz (rotate CW)
  Space      : stop (vx = 0, wz = 0)
  Z / X      : halve / double max speeds (nudges)
  Q          : quit

Requires no extra deps. Works in a regular terminal (POSIX). On Windows,
you may want to run under WSL or use Python's msvcrt fallback.
"""

import sys
import time
import json
import socket
import threading

# Cross-platform-ish nonblocking key reader
class _KeyReader:
    def __init__(self):
        try:
            import termios, tty, select
            self._posix = True
            self._termios = termios
            self._tty = tty
            self._select = select
            self._fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        except Exception:
            # Windows fallback: msvcrt (blocking-ish, but okay)
            self._posix = False
            import msvcrt  # type: ignore
            self._msvcrt = msvcrt

    def getch(self, timeout=0.02):
        if self._posix:
            dr, _, _ = self._select.select([sys.stdin], [], [], timeout)
            if dr:
                ch = sys.stdin.read(1)
                # handle ANSI arrows (esc-[A/B/C/D)
                if ch == "\x1b":
                    if self._select.select([sys.stdin], [], [], 0.0)[0]:
                        if sys.stdin.read(1) == "[":
                            c = sys.stdin.read(1)
                            return {"A": "UP", "B": "DOWN", "C": "RIGHT", "D": "LEFT"}.get(c, None)
                return ch
            return None
        else:
            if self._msvcrt.kbhit():
                ch = self._msvcrt.getch()
                try:
                    ch = ch.decode("utf-8")
                except Exception:
                    ch = None
                return ch
            return None

    def restore(self):
        if self._posix:
            self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old)


class BaseTeleopClient:
    """
    Keyboard teleop client.
    - Use member functions to adjust speed limits, deadman, etc.
    - Call run() to start the event loop (blocking).
    """

    def __init__(self,
                 dest_host: str = "127.0.0.1",
                 dest_port: int = 9999,
                 send_hz: float = 20.0,
                 vx_step: float = 0.05,
                 wz_step: float = 0.10,
                 vx_max: float = 0.6,
                 wz_max: float = 1.5):
        self.addr = (dest_host, dest_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.send_dt = 1.0 / max(1e-3, send_hz)

        self.vx_step = vx_step
        self.wz_step = wz_step
        self.vx_max = vx_max
        self.wz_max = wz_max

        self.vx = 0.0
        self.wz = 0.0
        self._running = False
        self._lock = threading.Lock()

    # ---------------- Public API ----------------

    def stop(self):
        with self._lock:
            self.vx, self.wz = 0.0, 0.0

    def scale_speed(self, factor: float):
        """Half/double max velocity limits."""
        with self._lock:
            self.vx_max = max(0.05, self.vx_max * factor)
            self.wz_max = max(0.2,  self.wz_max * factor)

    def nudge_vx(self, sign: float):
        with self._lock:
            self.vx += sign * self.vx_step
            self.vx = max(-self.vx_max, min(self.vx_max, self.vx))

    def nudge_wz(self, sign: float):
        with self._lock:
            self.wz += sign * self.wz_step
            self.wz = max(-self.wz_max, min(self.wz_max, self.wz))

    def current_cmd(self):
        with self._lock:
            return float(self.vx), float(self.wz)

    def run(self):
        print("[Teleop] Sending to {}:{} (UDP).".format(*self.addr))
        print("[Teleop] Keys: W/S forward/back, A/D rotate, arrows OK, Space stop, Z/X scale speeds, Q quit.")
        kr = _KeyReader()
        self._running = True
        t_last = 0.0
        try:
            while self._running:
                # 1) Poll keyboard (nonblocking)
                ch = kr.getch(timeout=0.01)
                if ch:
                    if ch in ("q", "Q"):
                        self._running = False
                        break
                    elif ch in ("w", "W", "UP"):
                        self.nudge_vx(+1.0)
                    elif ch in ("s", "S", "DOWN"):
                        self.nudge_vx(-1.0)
                    elif ch in ("a", "A", "LEFT"):
                        self.nudge_wz(+1.0)
                    elif ch in ("d", "D", "RIGHT"):
                        self.nudge_wz(-1.0)
                    elif ch == " ":
                        self.stop()
                    elif ch in ("z", "Z"):
                        self.scale_speed(0.5)
                        print(f"[Teleop] New vmax: vx={self.vx_max:.2f} m/s, wz={self.wz_max:.2f} rad/s")
                    elif ch in ("x", "X"):
                        self.scale_speed(2.0)
                        print(f"[Teleop] New vmax: vx={self.vx_max:.2f} m/s, wz={self.wz_max:.2f} rad/s")

                # 2) Periodic send
                now = time.time()
                if now - t_last >= self.send_dt:
                    vx, wz = self.current_cmd()
                    pkt = json.dumps({"vx": vx, "wz": wz}).encode("utf-8")
                    try:
                        self.sock.sendto(pkt, self.addr)
                    except Exception as e:
                        # Non-fatal
                        pass
                    t_last = now

        finally:
            kr.restore()
            self.stop()
            try:
                self.sock.close()
            except Exception:
                pass
            print("\n[Teleop] Exit.")


if __name__ == "__main__":
    client = BaseTeleopClient()
    client.run()
