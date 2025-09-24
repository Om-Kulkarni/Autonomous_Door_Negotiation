# baseline_state_machine.py
# Minimal baseline state machine using object_pose_est.py helpers:
# - Turn until the "door" is centered (within tolerance), then "move straight".
# - Actions are DISPLAY | ROTATE_LEFT | ROTATE_RIGHT | MOVE_STRAIGHT (text only).
# - Uses RealSense color+depth (depth optional) but runs YOLO *only on RGB*.
#
# Requirements:
#   - modules/object_pose_est.py present (with rotate_frame and realsense helpers)
#   - vision.py present (provides _select_device_and_precision)
#
# Run:
#   python baseline_state_machine.py --model /path/to/best.pt [--tol-px 40] [--rotation 90]

from pathlib import Path
import argparse
import numpy as np
import cv2
import pyrealsense2 as rs

# --- Bring in your existing utilities from object_pose_est & vision ---
try:
    from modules.vision import _select_device_and_precision
except Exception:
    from vision import _select_device_and_precision

try:
    # Helpers from object_pose_est.py
    from modules.object_pose_est import (
        rotate_frame,
    )
    # We’ll reimplement a tiny RS opener here to avoid importing private underscores.
    _HAVE_MODULES_PREFIX = True
except Exception:
    from object_pose_est import rotate_frame
    _HAVE_MODULES_PREFIX = False


def open_realsense(color_size=(640, 480), depth_size=(640, 480), fps: int = 30):
    """Start RealSense color+depth and return (pipeline, align)."""
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, color_size[0], color_size[1], rs.format.bgr8, fps)
    cfg.enable_stream(rs.stream.depth, depth_size[0], depth_size[1], rs.format.z16, fps)
    pipeline.start(cfg)
    align = rs.align(rs.stream.color)
    return pipeline, align


def class_name_from_ids(cls_id, names):
    """Resolve class name from Ultralytics result.names (dict or list)."""
    if isinstance(names, dict):
        return names.get(int(cls_id), str(cls_id))
    try:
        return names[int(cls_id)]
    except Exception:
        return str(cls_id)


def is_door_label(label: str) -> bool:
    """Identify detections that are doors (flexible substring match)."""
    l = label.lower()
    return "door" in l


def bbox_centroid(xyxy):
    x1, y1, x2, y2 = map(float, xyxy)
    return int((x1 + x2) / 2.0), int((y1 + y2) / 2.0)


def mask_centroid(mask: np.ndarray):
    if mask is None:
        return None
    if mask.dtype != np.uint8:
        mask = mask.astype(np.uint8)
    if mask.max() == 1:
        mask = mask * 255
    m = cv2.moments(mask)
    if m["m00"] == 0:
        return None
    return int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])


def choose_door_detection(result):
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
    masks = getattr(result, "masks", None)
    mask_data = None
    if masks is not None and getattr(masks, "data", None) is not None:
        try:
            mask_data = masks.data.detach().cpu().numpy()  # (N,H,W), float/bool
        except Exception:
            mask_data = None

    # gather door candidates
    candidates = []
    for i, bb in enumerate(xyxy):
        label = class_name_from_ids(cls_ids[i], names) if cls_ids is not None else "obj"
        if not is_door_label(label):
            continue
        x1, y1, x2, y2 = bb
        area = (x2 - x1) * (y2 - y1)
        # centroid preference: mask if available, else bbox
        c = None
        if mask_data is not None and i < mask_data.shape[0]:
            c = mask_centroid((mask_data[i] > 0.5).astype(np.uint8))
        if c is None:
            c = bbox_centroid(bb)
        candidates.append((area, c, bb, label))

    if not candidates:
        return None, None, None

    # pick the largest area door
    candidates.sort(key=lambda t: t[0], reverse=True)
    _, centroid, bb, label = candidates[0]
    return centroid, bb, label


def decide_action(cx: int, img_w: int, tol_px: int) -> str:
    """
    Decide action based on door centroid x (cx) relative to image center.
    If no centroid (cx is None) -> 'ROTATE_RIGHT' as a default search behavior.
    """
    if cx is None:
        return "ROTATE_RIGHT"  # default search spin
    center_x = img_w // 2
    dx = cx - center_x
    if abs(dx) <= tol_px:
        return "MOVE_STRAIGHT"
    return "ROTATE_LEFT" if dx < 0 else "ROTATE_RIGHT"


def overlay_status(img, state: str, cx: int | None, tol_px: int):
    """Draw current state, center line, tolerance band, and centroid marker."""
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
    # state text
    cv2.putText(img, f"STATE: {state}", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (50, 220, 50), 2, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser(description="Baseline door-centering state machine (display only).")
    ap.add_argument("--model", type=Path, required=True, help="Path to YOLO .pt")
    ap.add_argument("--imgsz", type=int, default=512, help="YOLO image size")
    ap.add_argument("--conf", type=float, default=0.25, help="YOLO confidence threshold")
    ap.add_argument("--iou", type=float, default=0.45, help="YOLO IoU threshold")
    ap.add_argument("--rotation", type=int, default=90, choices=[0, 90, 180, 270],
                    help="Rotate RGB before inference/display")
    ap.add_argument("--tol-px", type=int, default=40, help="Tolerance in pixels around image center")
    ap.add_argument("--show-depth", action="store_true", help="Show depth panel beside RGB")
    args = ap.parse_args()

    if not args.model.exists():
        raise FileNotFoundError(f"Model not found: {args.model}")

    device, half, use_gpu = _select_device_and_precision(400)
    print(f"[YOLO] Device: {'GPU' if use_gpu else 'CPU'} | half={half}")
    print(f"[YOLO] Loading model: {args.model}")

    from ultralytics import YOLO
    import torch

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

    pipeline, align = open_realsense()

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

            # Rotate BEFORE inference
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

            # Depth visualization (optional)
            if args.show_depth:
                depth_viz = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
                depth_rot = rotate_frame(depth_viz, rotation=args.rotation)

            # Decide action from largest door detection (centroid x)
            cx = None
            if result is not None:
                centroid, bb, label = choose_door_detection(result)
                if centroid is not None:
                    cx, cy = centroid
                    # draw explicit centroid + label
                    cv2.circle(vis, (cx, cy), 6, (0, 255, 255), -1)
                    cv2.putText(vis, f"{label}", (cx + 8, cy - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

            state = decide_action(cx, W, args.tol_px)
            overlay_status(vis, state, cx, args.tol_px)

            # Show window(s)
            if args.show_depth:
                view = np.hstack([vis, depth_rot])
            else:
                view = vis

            cv2.imshow("Baseline Door Alignment", view)

            # Also print the action in console (your integration hook)
            print(f"[ACTION] {state}")

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
