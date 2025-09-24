# object_pose_est.py
# Minimal, importable module that mirrors simple_test_relsense.py:
# - Opens Intel RealSense RGB + Depth streams at 640x480@30
# - Aligns depth to color
# - Shows color and depth (color-mapped) side-by-side
# - Rotates both views 90° clockwise (matching the original)
# - Quit with 'q'
#
# Usage:
#   from object_pose_est import run_realsense_view
#   run_realsense_view()  # opens the viewer, nothing is saved

import pyrealsense2 as rs
import numpy as np
import cv2


def rotate_frame(frame, rotation: int = 90):
    """
    Rotate an image by 90, 180, or 270 degrees clockwise.
    Any other value returns the frame unchanged.
    """
    if rotation == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    elif rotation == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    elif rotation == 270:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    else:
        return frame  # no rotation


def run_realsense_view(
    color_size=(640, 480),
    depth_size=(640, 480),
    fps: int = 30,
    rotation: int = 90,
    window_name: str = "color | depth (rotated)",
):
    """
    Open RealSense RGB + Depth, align depth to color, and display both.
    Press 'q' to quit.

    Args:
        color_size: (width, height) for color stream.
        depth_size: (width, height) for depth stream.
        fps: frames per second for both streams.
        rotation: rotation to apply to both views (degrees, clockwise).
        window_name: OpenCV window title.
    """
    # -------- RealSense setup --------
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, color_size[0], color_size[1], rs.format.bgr8, fps)
    cfg.enable_stream(rs.stream.depth, depth_size[0], depth_size[1], rs.format.z16, fps)

    profile = pipeline.start(cfg)
    align = rs.align(rs.stream.color)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                # If either stream is missing, skip this iteration
                continue

            depth = np.asanyarray(depth_frame.get_data())
            color = np.asanyarray(color_frame.get_data())

            # Depth visualization (uint16 -> 8-bit + colormap)
            depth_viz = cv2.applyColorMap(
                cv2.convertScaleAbs(depth, alpha=0.03),
                cv2.COLORMAP_JET
            )

            # Apply rotation (default 90° clockwise)
            color_rot = rotate_frame(color, rotation=rotation)
            depth_rot = rotate_frame(depth_viz, rotation=rotation)

            # Show side-by-side
            stacked = np.hstack([color_rot, depth_rot])
            cv2.imshow(window_name, stacked)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

            # Handle window closed manually (if supported)
            try:
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except Exception:
                pass
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    # Run directly for quick testing
    run_realsense_view()
