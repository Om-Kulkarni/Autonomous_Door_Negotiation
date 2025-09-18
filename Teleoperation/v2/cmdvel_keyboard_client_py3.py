#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Python 3 keyboard client for TIAGo base.
Connects to robot TCP server and sends newline-delimited JSON: {"vx":..,"wz":..}
Keys:
  w/s  : +/− linear x
  a/d  : +/− angular z (left/right)
  space: emergency stop (zero)
  r    : reset speeds to zero
  q    : quit
Arrows also work (↑/↓=linear, ←/→=angular).
"""

import socket
import sys
import json
import time
import curses

ROBOT_IP = "10.68.0.1"   # change if needed (see repo README network section)
ROBOT_PORT = 5005

VX_STEP = 0.05   # m/s increment per keypress
WZ_STEP = 0.10   # rad/s increment per keypress
MAX_ABS_VX = 0.5
MAX_ABS_WZ = 1.0
TX_HZ = 20.0     # send rate while keys are held / loop runs

def clamp(v, lo, hi):
    return min(max(v, lo), hi)

def connect():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    s.settimeout(5.0)
    s.connect((ROBOT_IP, ROBOT_PORT))
    s.settimeout(None)
    return s

def ui(stdscr):
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.keypad(True)
    curses.noecho()

    vx = 0.0
    wz = 0.0
    last_sent = 0.0

    while True:
        stdscr.clear()
        stdscr.addstr(0, 0, "TIAGo Base Teleop (TCP -> robot)")
        stdscr.addstr(1, 0, "w/s: linear +/- | a/d: angular +/- | space: STOP | r: reset | q: quit")
        stdscr.addstr(2, 0, "Arrow keys also supported.")
        stdscr.addstr(4, 0, "vx = %.3f m/s   wz = %.3f rad/s" % (vx, wz))
        stdscr.refresh()

        try:
            ch = stdscr.getch()
        except KeyboardInterrupt:
            raise

        if ch != curses.ERR:
            if ch in (ord('q'), ord('Q')):
                return None
            elif ch == ord(' '):
                vx, wz = 0.0, 0.0
            elif ch in (ord('r'), ord('R')):
                vx, wz = 0.0, 0.0
            elif ch in (ord('w'), curses.KEY_UP):
                vx = clamp(vx + VX_STEP, -MAX_ABS_VX, MAX_ABS_VX)
            elif ch in (ord('s'), curses.KEY_DOWN):
                vx = clamp(vx - VX_STEP, -MAX_ABS_VX, MAX_ABS_VX)
            elif ch in (ord('a'), curses.KEY_LEFT):
                wz = clamp(wz + WZ_STEP, -MAX_ABS_WZ, MAX_ABS_WZ)
            elif ch in (ord('d'), curses.KEY_RIGHT):
                wz = clamp(wz - WZ_STEP, -MAX_ABS_WZ, MAX_ABS_WZ)

        now = time.time()
        if now - last_sent >= 1.0 / TX_HZ:
            yield {"vx": vx, "wz": wz}
            last_sent = now

def main():
    # retry loop on disconnect
    while True:
        try:
            s = connect()
            break
        except Exception as e:
            print("Connection failed: %s; retrying in 2s..." % e)
            time.sleep(2)

    try:
        for msg in curses.wrapper(ui):
            if msg is None:
                break
            line = json.dumps(msg, separators=(',', ':')) + "\n"
            s.sendall(line.encode('utf-8'))
    except KeyboardInterrupt:
        pass
    finally:
        try:
            s.close()
        except:
            pass

if __name__ == "__main__":
    main()
