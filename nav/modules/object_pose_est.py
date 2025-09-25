# object_pose_est.py
# RealSense RGB + Depth viewer, with YOLOv11 *on RGB only* and 3D centroid (X,Y,Z in meters)
# for classes matching "door", "handle", or "knob".
# New: --no-depth / show_depth toggle and compact XYZ overlays.

from pathlib import Path
import numpy as np
import cv2
import pyrealsense2 as rs

# Reuse device/precision logic from your vision.py
try:
    from modules.vision import _select_device_and_precision
except Exception:
    from vision import _select_device_and_precision


# ---------------------- Image rotation helpers -------------------------------

def rotate_frame(frame, rotation: int = 90):
    """Rotate image by 90/180/270 deg clockwise; otherwise return unchanged."""
    if rotation == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    elif rotation == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    elif rotation == 270:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    else:
        return frame


def rotated_to_original_coords(xr: int, yr: int, w: int, h: int, rotation: int):
    """
    Map a pixel (xr, yr) from the rotated display back to the original (unrotated) image coordinates.
    rotation is clockwise: 0/90/180/270. w,h are the original (unrotated) image width/height.
    """
    if rotation == 0:
        return xr, yr
    elif rotation == 90:
        return yr, (h - 1 - xr)
    elif rotation == 180:
        return (w - 1 - xr), (h - 1 - yr)
    elif rotation == 270:
        return (w - 1 - yr), xr
    else:
        return xr, yr


# ---------------------- RealSense setup --------------------------------------

def _open_realsense(color_size=(640, 480), depth_size=(640, 480), fps: int = 30):
    """Start RealSense color+depth and return (pipeline, align)."""
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, color_size[0], color_size[1], rs.format.bgr8, fps)
    cfg.enable_stream(rs.stream.depth, depth_size[0], depth_size[1], rs.format.z16, fps)
    pipeline.start(cfg)
    align = rs.align(rs.stream.color)
    return pipeline, align


def run_realsense_view(
    color_size=(640, 480),
    depth_size=(640, 480),
    fps: int = 30,
    rotation: int = 90,
    window_name: str = "color | depth (rotated)",
    show_depth: bool = True,
):
    """Plain viewer: RGB + Depth (aligned) side-by-side (unless show_depth=False). Press 'q' to quit."""
    pipeline, align = _open_realsense(color_size=color_size, depth_size=depth_size, fps=fps)
    try:
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            depth = np.asanyarray(depth_frame.get_data())
            color = np.asanyarray(color_frame.get_data())

            color_rot = rotate_frame(color, rotation=rotation)

            if show_depth:
                depth_viz = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
                depth_rot = rotate_frame(depth_viz, rotation=rotation)
                view = np.hstack([color_rot, depth_rot])
            else:
                view = color_rot

            cv2.imshow(window_name, view)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            try:
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except Exception:
                pass
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


# ---------------------- Centroid & class utilities ---------------------------

def _bbox_centroid(xyxy):
    x1, y1, x2, y2 = map(float, xyxy)
    return int((x1 + x2) / 2.0), int((y1 + y2) / 2.0)


def _mask_centroid(mask: np.ndarray):
    if mask.dtype != np.uint8:
        mask = mask.astype(np.uint8)
    if mask.max() == 1:
        mask = mask * 255
    m = cv2.moments(mask)
    if m["m00"] == 0:
        return None
    return int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])


def _class_name(cls_id, names):
    if isinstance(names, dict):
        return names.get(int(cls_id), str(cls_id))
    try:
        return names[int(cls_id)]
    except Exception:
        return str(cls_id)


def _is_interesting_label(label: str):
    l = label.lower()
    return ("door" in l) or ("handle" in l) or ("knob" in l)


# ---------------------- Depth → 3D helpers -----------------------------------

def _depth_at(depth_frame: rs.depth_frame, u: int, v: int, search: int = 2) -> float:
    """
    Get depth (meters) at pixel (u, v). If 0, search a small (2*search+1) window for a non-zero median.
    """
    d = depth_frame.get_distance(u, v)
    if d and d > 0:
        return d

    w = depth_frame.get_width()
    h = depth_frame.get_height()
    u0 = max(0, u - search); u1 = min(w - 1, u + search)
    v0 = max(0, v - search); v1 = min(h - 1, v + search)
    vals = []
    for yy in range(v0, v1 + 1):
        for xx in range(u0, u1 + 1):
            dv = depth_frame.get_distance(xx, yy)
            if dv and dv > 0:
                vals.append(dv)
    if vals:
        return float(np.median(vals))
    return 0.0


def _pixel_to_3d(depth_frame: rs.depth_frame, u: int, v: int, depth_m: float):
    """
    Deproject (u,v,depth) to 3D camera coordinates (meters) using depth intrinsics.
    Returns (X, Y, Z).
    """
    intr = rs.video_stream_profile(depth_frame.profile).get_intrinsics()
    X, Y, Z = rs.rs2_deproject_pixel_to_point(intr, [float(u), float(v)], float(depth_m))
    return float(X), float(Y), float(Z)


# ---------------------- YOLO + RealSense main loop ---------------------------

def run_realsense_yolo(
    model_path: Path,
    *,
    color_size=(640, 480),
    depth_size=(640, 480),
    fps: int = 30,
    rotation: int = 90,            # fixed rotation to match viewer
    imgsz: int = 512,
    conf: float = 0.25,
    iou: float = 0.45,
    retina_masks: bool = False,
    max_det: int = 8,
    clear_interval: int = 50,
    window_name: str = "RGB+YOLO | Depth (rotated) + 3D",
    show_all_labels: bool = False,  # if True, compute XYZ for all classes
    show_depth: bool = True,        # <-- NEW: toggle depth panel
):
    """
    RealSense RGB + Depth with YOLOv11 on RGB only (no saving), and 3D centroid (X,Y,Z in meters).
    X,Y,Z are computed at the centroid pixel using aligned depth + deprojection.
    """
    if not Path(model_path).exists():
        raise FileNotFoundError(f"Model not found: {model_path}")

    device, half, use_gpu = _select_device_and_precision(400)
    print(f"[YOLO] Device: {'GPU' if use_gpu else 'CPU'} | half={half}")
    print(f"[YOLO] Loading model: {model_path}")

    from ultralytics import YOLO
    import torch

    model = YOLO(str(model_path))

    # Optional warm-up
    try:
        dummy = np.zeros((color_size[1], color_size[0], 3), dtype=np.uint8)
        _ = model.predict(
            source=dummy, imgsz=imgsz, conf=conf, iou=iou,
            device=device, half=half, save=False, show=False,
            retina_masks=retina_masks, max_det=max_det, stream=False, verbose=False
        )
        print("[YOLO] Warm-up complete.")
    except Exception as e:
        print(f"[YOLO] Warm-up skipped: {e}")

    pipeline, align = _open_realsense(color_size=color_size, depth_size=depth_size, fps=fps)

    try:
        processed = 0
        print("Running… Press 'q' or 'Esc' to quit.")
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Original (unrotated) arrays
            depth = np.asanyarray(depth_frame.get_data())
            color = np.asanyarray(color_frame.get_data())
            H, W = color.shape[:2]

            # Rotate BEFORE inference so YOLO runs on what we display
            color_for_infer = rotate_frame(color, rotation=rotation)

            # YOLO on RGB only
            results = model.predict(
                source=color_for_infer,
                imgsz=imgsz, conf=conf, iou=iou,
                device=device, half=half,
                save=False, show=False, retina_masks=retina_masks,
                max_det=max_det, stream=False, verbose=False
            )

            result = results[0] if results else None
            color_annot = result.plot() if result is not None else color_for_infer

            # Depth viz (rotate to match) if requested
            if show_depth:
                depth_viz = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
                depth_rot = rotate_frame(depth_viz, rotation=rotation)

            # --------------- 3D centroid overlays (BBox center only) --------------
            dets_total = 0
            if result is not None:
                names = getattr(result, "names", {})
                boxes = getattr(result, "boxes", None)
                masks = getattr(result, "masks", None)  # kept for compatibility, not used for centroid

                if boxes is not None:
                    try:
                        xyxy = boxes.xyxy.detach().cpu().numpy()
                        cls_ids = (boxes.cls.detach().cpu().numpy()
                                   if getattr(boxes, "cls", None) is not None else None)
                    except Exception:
                        xyxy, cls_ids = None, None

                    if xyxy is not None:
                        dets_total = len(xyxy)
                        for i, bb in enumerate(xyxy):
                            label = _class_name(cls_ids[i], names) if cls_ids is not None else "obj"
                            if not show_all_labels and not _is_interesting_label(label):
                                continue

                            # Centroid from BOUNDING BOX ONLY (no mask centroid)
                            centroid = _bbox_centroid(bb)
                            cx_r, cy_r = centroid

                            # Map centroid back to *original* pixel coords
                            u, v = rotated_to_original_coords(cx_r, cy_r, W, H, rotation)
                            u = int(np.clip(u, 0, W - 1))
                            v = int(np.clip(v, 0, H - 1))

                            # Depth (meters) with neighborhood fallback
                            z = _depth_at(depth_frame, u, v, search=2)

                            # Deproject to 3D and overlay compact text
                            if z > 0:
                                X, Y, Z = _pixel_to_3d(depth_frame, u, v, z)
                                # Draw centroid + compact XYZ (meters)
                                cv2.circle(color_annot, (cx_r, cy_r), 5, (0, 255, 255), -1)
                                text = f"{label}: X{X:.2f} Y{Y:.2f} Z{Z:.2f} m"
                                cv2.putText(color_annot, text, (cx_r + 8, cy_r - 8),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)
                            else:
                                cv2.circle(color_annot, (cx_r, cy_r), 5, (0, 165, 255), -1)
                                text = f"{label}: depth NA"
                                cv2.putText(color_annot, text, (cx_r + 8, cy_r - 8),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2, cv2.LINE_AA)
            # ----------------------------------------------------------------------

            # HUD (top-left)
            status = f"YOLO: {'GPU' if use_gpu else 'CPU'} | dets={dets_total} | conf≥{conf:.2f}"
            cv2.putText(color_annot, status, (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

            # Compose window
            if show_depth:
                view = np.hstack([color_annot, depth_rot])
            else:
                view = color_annot

            cv2.imshow(window_name, view)

            key = cv2.waitKey(1) & 0^0xFF  # keep low latency
            key &= 0xFF
            if key in (ord('q'), 27):
                break
            try:
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except Exception:
                pass

            processed += 1
            if use_gpu and (processed % clear_interval == 0):
                try:
                    torch.cuda.synchronize()
                    torch.cuda.empty_cache()
                except Exception:
                    pass

        print("\n=== RealSense + YOLO + 3D finished ===")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="RealSense viewer with YOLO on RGB and 3D centroid overlays.")
    parser.add_argument("--model", type=Path, required=True, help="Path to YOLO .pt")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--imgsz", type=int, default=512)
    parser.add_argument("--all", action="store_true", help="Show XYZ for all classes, not just door/handle.")
    parser.add_argument("--no-depth", action="store_true", help="Hide the depth panel.")
    args = parser.parse_args()

    run_realsense_yolo(
        model_path=args.model,
        conf=args.conf,
        iou=args.iou,
        imgsz=args.imgsz,
        show_all_labels=bool(args.all),
        show_depth=not args.no_depth,
    )
