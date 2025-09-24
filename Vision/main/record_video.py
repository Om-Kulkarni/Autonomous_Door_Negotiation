#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import numpy as np
import pyrealsense2 as rs

def rotate_img(img, rot_state):
    """
    rot_state: 0=0°, 1=90° CW, 2=180°, 3=90° CCW
    """
    if rot_state == 0:
        return img
    elif rot_state == 1:
        return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    elif rot_state == 2:
        return cv2.rotate(img, cv2.ROTATE_180)
    elif rot_state == 3:
        return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return img

def main():
    # --- RealSense setup ---
    pipeline = rs.pipeline()
    cfg = rs.config()
    w, h, fps = 640, 480, 30
    cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
    cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)

    profile = pipeline.start(cfg)
    align = rs.align(rs.stream.color)

    # --- Output dir & writers (created on demand) ---
    out_dir = "data/real/d435i/pull"
    os.makedirs(out_dir, exist_ok=True)
    rgb_path   = os.path.join(out_dir, "video_rgb_r1.mp4")
    depth_path = os.path.join(out_dir, "video_depth_r1.mp4")

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    rgb_writer = None
    depth_writer = None

    # --- UI / state ---
    win = "RealSense: color | depth   [r=start, s=stop, a=rotate, q=quit]"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    recording = False

    # Rotation state: 0=0°, 1=90° CW, 2=180°, 3=90° CCW
    rot_state = 1  # default to 90° clockwise per your request
    rot_labels = {0: "0°", 1: "90° CW", 2: "180°", 3: "90° CCW"}

    try:
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)
            d = frames.get_depth_frame()
            c = frames.get_color_frame()
            if not d or not c:
                continue

            color = np.asanyarray(c.get_data())
            depth = np.asanyarray(d.get_data())

            # Depth visualization for display & saving (MP4 expects 8-bit 3ch)
            depth_viz = cv2.applyColorMap(
                cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET
            )

            # Apply rotation to both frames
            color_rot = rotate_img(color, rot_state)
            depth_rot = rotate_img(depth_viz, rot_state)

            # Compose side-by-side preview AFTER rotation (sizes match)
            panel = np.hstack([color_rot, depth_rot])

            # Overlay status text
            status = "REC" if recording else "PAUSED"
            info = f"Status: {status}  |  Rotation: {rot_labels[rot_state]}  [r=start, s=stop, a=rotate, q=quit]"
            cv2.putText(panel, info, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

            # Show
            cv2.imshow(win, panel)

            # Initialize writers on first need to guarantee width/height/fps
            if recording and rgb_writer is None:
                frame_size = (color_rot.shape[1], color_rot.shape[0])  # (width, height) after rotation
                rgb_writer = cv2.VideoWriter(rgb_path, fourcc, fps, frame_size, True)
                depth_writer = cv2.VideoWriter(depth_path, fourcc, fps, frame_size, True)

            # If recording, write current frames (rotated)
            if recording and rgb_writer is not None and depth_writer is not None:
                rgb_writer.write(color_rot)
                depth_writer.write(depth_rot)

            # Key handling
            key = cv2.waitKey(1) & 0xFF
            if key == ord('r'):
                if not recording:
                    print("[INFO] Recording started.")
                    # (Re)create writers matching current rotation/frame size
                    if rgb_writer is not None:
                        rgb_writer.release()
                        rgb_writer = None
                    if depth_writer is not None:
                        depth_writer.release()
                        depth_writer = None
                    recording = True
                else:
                    print("[INFO] Already recording.")
            elif key == ord('s'):
                if recording:
                    print("[INFO] Recording stopped.")
                    recording = False
                    if rgb_writer is not None:
                        rgb_writer.release()
                        rgb_writer = None
                    if depth_writer is not None:
                        depth_writer.release()
                        depth_writer = None
                else:
                    print("[INFO] Not recording; nothing to stop.")
            elif key == ord('a'):
                if recording:
                    print("[WARN] Rotation change ignored while recording. Press 's' to stop first.")
                else:
                    rot_state = (rot_state + 1) % 4
                    print(f"[INFO] Rotation set to {rot_labels[rot_state]}")
            elif key == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        # Ensure writers are released
        if rgb_writer is not None:
            rgb_writer.release()
        if depth_writer is not None:
            depth_writer.release()
        print(f"[SAVED] RGB video:   {rgb_path}")
        print(f"[SAVED] Depth video: {depth_path} (colorized depth)")

if __name__ == "__main__":
    main()
