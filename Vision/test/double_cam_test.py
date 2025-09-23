import pyrealsense2 as rs
import numpy as np
import cv2
import time
import sys

# ---------- Settings ----------
FRAME_WIDTH, FRAME_HEIGHT, FPS = 640, 480, 30
ROTATION_DEG = 90  # 0, 90, 180, 270
WINDOW_435I = "D435i: color | depth (rotated)"
WINDOW_457  = "D457:  color | depth (rotated)"
TIMEOUT_MS = 200  # wait_for_frames timeout per pipeline

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

def draw_fps_top_right(img, fps_text, margin=8, font=cv2.FONT_HERSHEY_SIMPLEX, scale=0.6, thickness=2):
    """
    Draw text at the top-right with a subtle background for readability.
    """
    (text_w, text_h), baseline = cv2.getTextSize(fps_text, font, scale, thickness)
    x2 = img.shape[1] - margin
    y1 = margin
    x1 = x2 - text_w - 2*margin
    y2 = y1 + text_h + 2*margin

    # Background box
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 0), -1)
    # Text (baseline adjusted)
    cv2.putText(img, fps_text, (x1 + margin, y2 - margin - baseline), font, scale, (255, 255, 255), thickness, cv2.LINE_AA)

def colorize_depth(depth_frame):
    """
    Convert depth frame to a colored visualization.
    """
    depth = np.asanyarray(depth_frame.get_data())
    depth_vis = cv2.applyColorMap(
        cv2.convertScaleAbs(depth, alpha=0.03),  # scale for visualization
        cv2.COLORMAP_JET
    )
    return depth_vis

def start_pipeline_for_serial(serial, width=FRAME_WIDTH, height=FRAME_HEIGHT, fps=FPS):
    """
    Create and start a RealSense pipeline bound to a specific device serial.
    Returns (pipeline, align_to_color).
    """
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    pipeline_profile = pipeline.start(cfg)
    align = rs.align(rs.stream.color)
    return pipeline, align

def find_serials_for_models(targets=("D435I", "D457")):
    """
    Scan connected RealSense devices and return a dict {model_key: serial}.
    Matching is case-insensitive substring search on camera name.
    """
    ctx = rs.context()
    if len(ctx.devices) == 0:
        print("No RealSense devices found.")
        sys.exit(1)

    found = {}
    targets_upper = [t.upper() for t in targets]

    for dev in ctx.query_devices():
        name = dev.get_info(rs.camera_info.name).upper()
        serial = dev.get_info(rs.camera_info.serial_number)
        for t in targets_upper:
            # accept either exact token or substring in the reported name
            if t in name and t not in found:
                found[t] = serial

    # Friendly error if any target missing
    missing = [t for t in targets_upper if t not in found]
    if missing:
        msg = "Could not find required devices: " + ", ".join(missing)
        # Also list what's connected for debugging
        connected = [f"{d.get_info(rs.camera_info.name)} [{d.get_info(rs.camera_info.serial_number)}]" for d in ctx.query_devices()]
        msg += "\nConnected devices:\n  - " + "\n  - ".join(connected)
        print(msg)
        sys.exit(1)

    return {k: found[k] for k in targets_upper}

def main():
    # ---- Discover the D435i and D457 by name and get their serials ----
    serials = find_serials_for_models(("D435I", "D457"))
    serial_435i = serials["D435I"]
    serial_457  = serials["D457"]

    # ---- Start pipelines ----
    p435i, align435i = start_pipeline_for_serial(serial_435i)
    p457,  align457  = start_pipeline_for_serial(serial_457)

    # FPS trackers
    last_t_435i = time.time()
    last_t_457  = time.time()
    fps_435i = 0.0
    fps_457  = 0.0

    cv2.namedWindow(WINDOW_435I, cv2.WINDOW_NORMAL)
    cv2.namedWindow(WINDOW_457,  cv2.WINDOW_NORMAL)

    try:
        while True:
            # -------- Read both cameras (with timeout to keep UI responsive) --------
            frames_435i = p435i.wait_for_frames(TIMEOUT_MS)
            frames_457  = p457.wait_for_frames(TIMEOUT_MS)

            # Align to color for both
            frames_435i = align435i.process(frames_435i)
            frames_457  = align457.process(frames_457)

            d435i = frames_435i.get_depth_frame()
            c435i = frames_435i.get_color_frame()
            d457  = frames_457.get_depth_frame()
            c457  = frames_457.get_color_frame()

            if not (d435i and c435i and d457 and c457):
                # If any stream missing, skip this loop iteration
                # (could also handle fallbacks here).
                continue

            # -------- Convert to numpy --------
            color_435i = np.asanyarray(c435i.get_data())
            depth_vis_435i = colorize_depth(d435i)

            color_457 = np.asanyarray(c457.get_data())
            depth_vis_457 = colorize_depth(d457)

            # -------- Rotate if requested --------
            color_435i = rotate_frame(color_435i, ROTATION_DEG)
            depth_vis_435i = rotate_frame(depth_vis_435i, ROTATION_DEG)
            color_457 = rotate_frame(color_457, ROTATION_DEG)
            depth_vis_457 = rotate_frame(depth_vis_457, ROTATION_DEG)

            # -------- Compute FPS (per camera) --------
            now = time.time()
            dt_435i = now - last_t_435i
            dt_457  = now - last_t_457
            if dt_435i > 0:
                fps_435i = 0.9 * fps_435i + 0.1 * (1.0 / dt_435i) if fps_435i > 0 else (1.0 / dt_435i)
            if dt_457 > 0:
                fps_457  = 0.9 * fps_457  + 0.1 * (1.0 / dt_457)  if fps_457  > 0 else (1.0 / dt_457)
            last_t_435i = now
            last_t_457  = now

            # -------- Compose side-by-side panels --------
            panel_435i = np.hstack([color_435i, depth_vis_435i])
            panel_457  = np.hstack([color_457,  depth_vis_457])

            # -------- Overlay FPS (top-right) --------
            draw_fps_top_right(panel_435i, f"FPS: {fps_435i:5.1f}")
            draw_fps_top_right(panel_457,  f"FPS: {fps_457:5.1f}")

            # -------- Show windows --------
            cv2.imshow(WINDOW_435I, panel_435i)
            cv2.imshow(WINDOW_457,  panel_457)

            # Quit on 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Clean up
        try:
            p435i.stop()
        except Exception:
            pass
        try:
            p457.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
