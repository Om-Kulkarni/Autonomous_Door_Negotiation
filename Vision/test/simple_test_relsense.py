import pyrealsense2 as rs
import numpy as np, cv2

def rotate_frame(frame, rotation=90):
    """
    Rotate an image by 90, 180, or 270 degrees clockwise.
    """
    if rotation == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    elif rotation == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    elif rotation == 270:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    else:
        return frame  # no rotation

# -------- RealSense setup --------
p = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

prof = p.start(cfg)
align = rs.align(rs.stream.color)

try:
    while True:
        frames = p.wait_for_frames()
        frames = align.process(frames)
        d = frames.get_depth_frame()
        c = frames.get_color_frame()
        if not d or not c:
            continue

        depth = np.asanyarray(d.get_data())
        color = np.asanyarray(c.get_data())

        # Depth visualization
        depth_viz = cv2.applyColorMap(
            cv2.convertScaleAbs(depth, alpha=0.03),
            cv2.COLORMAP_JET
        )

        # ---- Apply rotation control ----
        color_rot = rotate_frame(color, rotation=90)      # 90° clockwise
        depth_rot = rotate_frame(depth_viz, rotation=90)  # 90° clockwise

        # Show side by side
        cv2.imshow("color | depth (rotated)", np.hstack([color_rot, depth_rot]))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    p.stop()
    cv2.destroyAllWindows()
