# baseline_navigation_auto.py
# Same perception + focus logic as baseline_navigation.py, but instead of only drawing
# actions on the HUD, it sends (linear.x, angular.z) velocity commands to a server
# over TCP as big-endian floats, just like teleop_client.py.
#
# Behavior:
#   1) SEARCH (no target)  -> turn anticlockwise (angular.z > 0)
#   2) Door found left of tolerance band  -> TURN LEFT  (angular.z > 0)
#   3) Door found right of tolerance band -> TURN RIGHT (angular.z < 0)
#   4) Door within tolerance band         -> MOVE STRAIGHT (linear.x > 0)
#   5) If proximity to door > 1.5 m       -> focus DOOR
#   6) If proximity to door < 1.5 m       -> focus HANDLE
#   7) If <1.5 m and handle missing       -> fall back to DOOR; if handle reappears, refocus HANDLE
#   8) If proximity to target < 0.6 m     -> STOP
#
# The rest (camera, YOLO, rotation, overlays) mirrors baseline_navigation.py.

from pathlib import Path
import argparse
import socket
import struct
import time
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
        _depth_at,
        rotated_to_original_coords
    )
    _HAVE_MODULES_PREFIX = True
except Exception:
    from object_pose_est import rotate_frame, _bbox_centroid, _class_name  # type: ignore
    from object_pose_est import _depth_at, rotated_to_original_coords  # type: ignore
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

def _is_handle_label(label: str) -> bool:
    l = label.lower()
    return ("handle" in l) or ("knob" in l)


# ---------------------- Detection selection ---------------------------------

def _choose_door_detection(result):
    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return None, None, None
    try:
        xyxy = boxes.xyxy.detach().cpu().numpy()
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

def _choose_handle_detection(result):
    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return None, None, None
    try:
        xyxy = boxes.xyxy.detach().cpu().numpy()
        cls_ids = boxes.cls.detach().cpu().numpy() if getattr(boxes, "cls", None) is not None else None
    except Exception:
        return None, None, None

    names = getattr(result, "names", {})
    candidates = []
    for i, bb in enumerate(xyxy):
        label = _class_name(cls_ids[i], names) if cls_ids is not None else "obj"
        if not _is_handle_label(label):
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

def _decide_action(cx: int | None, img_w: int, tol_px: int, distance_m: float | None) -> str:
    """
    New stop threshold per spec: STOP if proximity < 0.6 m
    Otherwise, same directional band logic as baseline.
    """
    if distance_m is not None and distance_m > 0 and distance_m < 0.6:
        return "STOP"
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


def _overlay_status(img, action: str, cx: int | None, tol_px: int, distance_m: float | None, label: str | None):
    """Draw HUD: action, center band, centroid, distance, and focus label (door/handle)."""
    h, w = img.shape[:2]
    mid = w // 2
    cv2.line(img, (mid, 0), (mid, h), (255, 255, 255), 1)
    cv2.rectangle(img, (mid - tol_px, 0), (mid + tol_px, h), (200, 200, 200), 1)
    if cx is not None:
        cv2.circle(img, (int(cx), h // 2), 6, (0, 255, 255), -1)
        cv2.line(img, (int(cx), 0), (int(cx), h), (0, 255, 255), 1)
    cv2.putText(img, f"ACTION: {action}", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.85, (50, 220, 50), 2, cv2.LINE_AA)
    if distance_m is not None and distance_m > 0:
        cv2.putText(img, f"DIST: {distance_m:.2f} m", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2, cv2.LINE_AA)
    if label:
        cv2.putText(img, f"FOCUS: {label}", (10, 92),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)


# ---------------------- Networking ------------------------------------------

class VelocitySocket:
    """
    Minimal TCP client that sends (linear.x, angular.z) as big-endian floats (>ff).
    Reconnects on failure. Non-blocking-ish send with short timeouts.
    """
    def __init__(self, host: str, port: int, connect_timeout: float = 2.0, send_timeout: float = 0.2):
        self.host = host
        self.port = port
        self.connect_timeout = connect_timeout
        self.send_timeout = send_timeout
        self.sock: socket.socket | None = None
        self._connect()

    def _connect(self):
        self.close()
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(self.connect_timeout)
            s.connect((self.host, self.port))
            s.settimeout(self.send_timeout)
            self.sock = s
            print(f"[NET] Connected to {self.host}:{self.port}")
        except Exception as e:
            print(f"[NET] Connect failed: {e}")
            self.sock = None

    def send(self, linear_x: float, angular_z: float):
        payload = struct.pack('>ff', float(linear_x), float(angular_z))
        if self.sock is None:
            self._connect()
        try:
            if self.sock:
                self.sock.sendall(payload)
        except Exception as e:
            print(f"[NET] Send failed ({e}), will retry connect.")
            self._connect()

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            finally:
                self.sock = None


# ---------------------- Main -------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Auto baseline navigation: sends velocities over TCP while aligning to a door.")
    # Perception args (same as baseline)
    ap.add_argument("--model", type=Path, required=True, help="Path to YOLO .pt")
    ap.add_argument("--imgsz", type=int, default=512, help="YOLO image size")
    ap.add_argument("--conf", type=float, default=0.5, help="YOLO confidence threshold")
    ap.add_argument("--iou", type=float, default=0.45, help="YOLO IoU threshold")
    ap.add_argument("--rotation", type=int, default=90, choices=[0, 90, 180, 270],
                    help="Rotate RGB before inference/display")
    ap.add_argument("--tol-px", type=int, default=60, help="Tolerance in pixels around image center")
    ap.add_argument("--show-depth", action="store_true", help="Show depth panel beside RGB (informational)")
    # Networking + control rate
    ap.add_argument("--host", type=str, default="10.68.0.1", help="Server host")
    ap.add_argument("--port", type=int, default=65433, help="Server port")
    ap.add_argument("--rate-hz", type=float, default=20.0, help="Command send rate")
    # Speed presets (m/s and rad/s)
    ap.add_argument("--v-forward", type=float, default=0.25, help="Linear speed for MOVE STRAIGHT")
    ap.add_argument("--v-turn", type=float, default=0.8, help="Angular speed magnitude for TURN LEFT/RIGHT")
    ap.add_argument("--v-search", type=float, default=0.6, help="Angular speed for SEARCH (CCW)")
    args = ap.parse_args()

    if not args.model.exists():
        raise FileNotFoundError(f"Model not found: {args.model}")

    device, half, use_gpu = _select_device_and_precision(400)
    print(f"[YOLO] Device: {'GPU' if use_gpu else 'CPU'} | half={half}")
    print(f"[YOLO] Loading model: {args.model}")

    from ultralytics import YOLO
    import torch  # noqa: F401

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

    # Open network socket
    vel_sock = VelocitySocket(args.host, args.port)
    period = 1.0 / max(1e-6, args.rate_hz)
    last_send = 0.0

    def send_cmd(lx: float, az: float):
        nonlocal last_send
        t = time.time()
        if (t - last_send) >= period:
            vel_sock.send(lx, az)
            last_send = t

    try:
        print("Running… Press 'q' or 'Esc' to quit.")
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                # Keep issuing a gentle SEARCH spin if camera hiccups
                send_cmd(0.0, args.v_search)
                continue

            depth = np.asanyarray(depth_frame.get_data())
            color = np.asanyarray(color_frame.get_data())
            H0, W0 = color.shape[:2]

            color_rot = rotate_frame(color, rotation=args.rotation)
            H, W = color_rot.shape[:2]

            results = model.predict(
                source=color_rot,
                imgsz=args.imgsz, conf=args.conf, iou=args.iou,
                device=device, half=half, save=False, show=False,
                retina_masks=False, max_det=8, stream=False, verbose=False
            )
            result = results[0] if results else None
            vis = result.plot() if result is not None else color_rot.copy()

            # Optional depth viz panel
            if args.show_depth:
                depth_viz = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
                depth_rot = rotate_frame(depth_viz, rotation=args.rotation)

            cx = None
            distance_m = None
            label_for_hud = None

            if result is not None:
                # Choose DOOR first to measure proximity
                centroid, bb, focus_label = _choose_door_detection(result)
                if centroid is not None:
                    cx, cy = centroid
                    u, v = rotated_to_original_coords(cx, cy, W0, H0, args.rotation)
                    dw, dh = depth_frame.get_width(), depth_frame.get_height()
                    u = int(np.clip(u, 0, dw - 1))
                    v = int(np.clip(v, 0, dh - 1))
                    distance_m = _depth_at(depth_frame, u, v, search=2)

                    # If within 1.5 m, try HANDLE; fall back to DOOR if none
                    if distance_m is not None and distance_m > 0 and distance_m < 1.5:
                        h_centroid, _, h_label = _choose_handle_detection(result)
                        if h_centroid is not None:
                            cx, cy = h_centroid
                            focus_label = h_label
                            u, v = rotated_to_original_coords(cx, cy, W0, H0, args.rotation)
                            u = int(np.clip(u, 0, dw - 1))
                            v = int(np.clip(v, 0, dh - 1))
                            distance_m = _depth_at(depth_frame, u, v, search=2)
                    label_for_hud = focus_label

                    # Draw focus point label
                    cv2.circle(vis, (cx, cy), 6, (0, 255, 255), -1)
                    if focus_label:
                        cv2.putText(vis, f"{focus_label}", (cx + 8, cy - 8),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

            action = _decide_action(cx, W, args.tol_px, distance_m)
            _overlay_status(vis, action, cx, args.tol_px, distance_m, label_for_hud)

            # Map actions -> velocities (m/s, rad/s)
            lin_x, ang_z = 0.0, 0.0
            if action == "SEARCH":
                lin_x, ang_z = 0.0, abs(args.v_search)          # CCW
            elif action == "TURN LEFT":
                lin_x, ang_z = 0.0, abs(args.v_turn)            # CCW
            elif action == "TURN RIGHT":
                lin_x, ang_z = 0.0, -abs(args.v_turn)           # CW
            elif action == "MOVE STRAIGHT":
                lin_x, ang_z = abs(args.v_forward), 0.0
            elif action == "STOP":
                lin_x, ang_z = 0.0, 0.0

            # Send at configured rate
            send_cmd(lin_x, ang_z)

            # Compose window
            if args.show_depth:
                view = np.hstack([vis, depth_rot])
            else:
                view = vis

            cv2.imshow("Baseline Door Alignment (AUTO)", view)
            # Also log the command at a modest rate
            if int(time.time() * 5) % 5 == 0:  # ~5 Hz print without flooding
                print(f"[ACTION] {action} | vx={lin_x:.2f} m/s, wz={ang_z:.2f} rad/s")

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            try:
                if cv2.getWindowProperty("Baseline Door Alignment (AUTO)", cv2.WND_PROP_VISIBLE) < 1:
                    break
            except Exception:
                pass

    finally:
        try:
            vel_sock.send(0.0, 0.0)  # best-effort stop on exit
        except Exception:
            pass
        vel_sock.close()
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
