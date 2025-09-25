# baseline_navigation.py
# Simple state machine to align to a detected door using YOLO on RealSense RGB.
# All indications (TURN LEFT / TURN RIGHT / MOVE STRAIGHT) are drawn on the image window.
#
# Utilizes helpers from object_pose_est.py:
#   - rotate_frame
#   - _open_realsense (RealSense RGB+Depth pipeline start)
#   - _bbox_centroid (centroid from bbox)
#   - _class_name (map cls id -> label)
# And vision._select_device_and_precision for device/half selection.
#
# Run:
#   python baseline_navigation.py --model /path/to/best.pt [--tol-px 60] [--rotation 90] [--imgsz 512]

from pathlib import Path
import argparse
import numpy as np
import cv2
import pyrealsense2 as rs

# ---- Bring in utilities from modules.* if present, else local files ----
try:
    from modules.vision import _select_device_and_precision
except Exception:
    from vision import _select_device_and_precision  # type: ignore

try:
    from modules.object_pose_est import (
        rotate_frame,
        _open_realsense,
        _bbox_centroid,
        _class_name,
    )
    _HAVE_MODULES_PREFIX = True
except Exception:
    from object_pose_est import rotate_frame, _bbox_centroid, _class_name  # type: ignore
    # Fallback: minimal local RS opener if object_pose_est is flat
    def _open_realsense(color_size=(640, 480), depth_size=(640, 480), fps: int = 30):
        pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, color_size[0], color_size[1], rs.format.bgr8, fps)
        cfg.enable_stream(rs.stream.depth, depth_size[0], depth_size[1], rs.format.z16, fps)
        pipeline.start(cfg)
        align = rs.align(rs.stream.color)
        return pipeline, align


# ---------------------- Label helpers ---------------------------------------

def _is_door_label(label: str) -> bool:
    l = label.lower()
    return "door" in l


# ---------------------- Detection selection ---------------------------------

def _choose_door_detection(result):
    """
    From a Ultralytics result, pick ONE door detection (largest area preferred).
    Returns (centroid_xy, bbox_xyxy, label) in the *rotated* frame coordinates,
    or (None, None, None) if no door found.
    """
    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return None, None, None

    try:
        xyxy = boxes.xyxy.detach().cpu().numpy()  # (N,4)
        cls_ids = boxes.cls.detach().cpu().numpy() if getattr(boxes, "cls", None) is not None else None
    except Exception:
        return None, None, None

    names = getattr(result, "names", {})

    candidates = []
    for i, bb in enumerate(xyxy):
        label = _class_name(cls_ids[i], names) if cls_ids is not None else "obj"
        if not _is_door_label(label):
            continue
        x1, y1, x2, y2 = map(float, bb)
        area = (x2 - x1) * (y2 - y1)
        c = _bbox_centroid(bb)
        candidates.append((area, c, bb, label))

    if not candidates:
        return None, None, None

    candidates.sort(key=lambda t: t[0], reverse=True)
    _, centroid, bb, label = candidates[0]
    return centroid, bb, label


# ---------------------- State logic -----------------------------------------

def _decide_action(cx: int | None, img_w: int, tol_px: int) -> str:
    """
    If the door centroid is LEFT of the left tolerance limit  -> TURN LEFT
    If the door centroid is RIGHT of the right tolerance limit -> TURN RIGHT
    If centroid is within the tolerance band                   -> MOVE STRAIGHT
    If no centroid (cx is None)                                -> SEARCH (turn right by default)
    """
    if cx is None:
        return "SEARCH"

    center_x = img_w // 2
    left_limit = center_x - tol_px
    right_limit = center_x + tol_px

    if cx < left_limit:
        return "TURN LEFT"
    elif cx > right_limit:
        return "TURN RIGHT"
    else:
        return "MOVE STRAIGHT"


def _overlay_status(img, action: str, cx: int | None, tol_px: int):
    """Draw current action, center line, tolerance band, and centroid marker."""
    h, w = img.shape[:2]
    mid = w // 2

    # center line
    cv2.line(img, (mid, 0), (mid, h), (255, 255, 255), 1)
    # tolerance band
    cv2.rectangle(img, (mid - tol_px, 0), (mid + tol_px, h), (200, 200, 200), 1)

    # centroid marker
    if cx is not None:
        cv2.circle(img, (int(cx), h // 2), 6, (0, 255, 255), -1)
        cv2.line(img, (int(cx), 0), (int(cx), h), (0, 255, 255), 1)

    # action text HUD
    cv2.putText(img, f"ACTION: {action}", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.85, (50, 220, 50), 2, cv2.LINE_AA)


# ---------------------- Main -------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Baseline door-centering state machine (on-window indications).")
    ap.add_argument("--model", type=Path, required=True, help="Path to YOLO .pt")
    ap.add_argument("--imgsz", type=int, default=512, help="YOLO image size")
    ap.add_argument("--conf", type=float, default=0.25, help="YOLO confidence threshold")
    ap.add_argument("--iou", type=float, default=0.45, help="YOLO IoU threshold")
    ap.add_argument("--rotation", type=int, default=90, choices=[0, 90, 180, 270],
                    help="Rotate RGB before inference/display")
    ap.add_argument("--tol-px", type=int, default=60, help="Tolerance in pixels around image center")
    ap.add_argument("--show-depth", action="store_true", help="Show depth panel beside RGB (informational)")
    args = ap.parse_args()

    if not args.model.exists():
        raise FileNotFoundError(f"Model not found: {args.model}")

    device, half, use_gpu = _select_device_and_precision(400)
    print(f"[YOLO] Device: {'GPU' if use_gpu else 'CPU'} | half={half}")
    print(f"[YOLO] Loading model: {args.model}")

    from ultralytics import YOLO
    import torch  # noqa: F401 (used in GPU cache clear try/except pattern)

    model = YOLO(str(args.model))

    # Optional warm-up
    try:
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        _ = model.predict(
            source=dummy, imgsz=args.imgsz, conf=args.conf, iou=args.iou,
            device=device, half=half, save=False, show=False, retina_masks=False,
            max_det=8, stream=False, verbose=False
        )
        print("[YOLO] Warm-up complete.")
    except Exception as e:
        print(f"[YOLO] Warm-up skipped: {e}")

    # Open RealSense (aligned depth to color)
    pipeline, align = _open_realsense()

    try:
        print("Running… Press 'q' or 'Esc' to quit.")
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            depth = np.asanyarray(depth_frame.get_data())
            color = np.asanyarray(color_frame.get_data())

            # Rotate BEFORE inference so YOLO runs on what we display
            color_rot = rotate_frame(color, rotation=args.rotation)
            H, W = color_rot.shape[:2]

            # YOLO on rotated RGB
            results = model.predict(
                source=color_rot,
                imgsz=args.imgsz, conf=args.conf, iou=args.iou,
                device=device, half=half, save=False, show=False,
                retina_masks=False, max_det=8, stream=False, verbose=False
            )
            result = results[0] if results else None
            vis = result.plot() if result is not None else color_rot.copy()

            # Optional depth viz panel (informational only)
            if args.show_depth:
                depth_viz = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
                depth_rot = rotate_frame(depth_viz, rotation=args.rotation)

            # Decide action from largest door detection (centroid x)
            cx = None
            if result is not None:
                centroid, bb, label = _choose_door_detection(result)
                if centroid is not None:
                    cx, cy = centroid
                    # draw explicit centroid + label
                    cv2.circle(vis, (cx, cy), 6, (0, 255, 255), -1)
                    cv2.putText(vis, f"{label}", (cx + 8, cy - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

            action = _decide_action(cx, W, args.tol_px)
            _overlay_status(vis, action, cx, args.tol_px)

            # Compose window
            if args.show_depth:
                view = np.hstack([vis, depth_rot])
            else:
                view = vis

            cv2.imshow("Baseline Door Alignment", view)
            # Also print the action in console (optional)
            print(f"[ACTION] {action}")

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            try:
                if cv2.getWindowProperty("Baseline Door Alignment", cv2.WND_PROP_VISIBLE) < 1:
                    break
            except Exception:
                pass

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
