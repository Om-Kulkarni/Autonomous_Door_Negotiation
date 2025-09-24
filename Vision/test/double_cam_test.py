import os
# --- Make OpenCV HighGUI work under Wayland by using X11 (xcb) ---
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import sys
import time
import cv2
import numpy as np
import pyrealsense2 as rs

# ---------- Settings ----------
FRAME_WIDTH, FRAME_HEIGHT, FPS = 640, 480, 30
ROTATION_DEG = 90  # 0, 90, 180, 270
WINDOW_A = "Cam A (D435i): color | depth"
WINDOW_B = "Cam B (D457/D455): color | depth"

# Timeouts (ms)
WAIT_TIMEOUT_MS = 1200     # main loop: be generous for dual-cam USB scheduling
WARMUP_TIMEOUT_MS = 3000   # startup warmup timeout per grab
WARMUP_FRAMES = 20         # discard N frames to let auto-exposure/align stabilize

def rotate_frame(frame, rotation=90):
    if rotation == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    elif rotation == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    elif rotation == 270:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    else:
        return frame

def draw_fps_top_right(img, fps_text, margin=8, font=cv2.FONT_HERSHEY_SIMPLEX, scale=0.6, thickness=2):
    (tw, th), base = cv2.getTextSize(fps_text, font, scale, thickness)
    x2 = img.shape[1] - margin
    y1 = margin
    x1 = x2 - tw - 2*margin
    y2 = y1 + th + 2*margin
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 0), -1)
    cv2.putText(img, fps_text, (x1 + margin, y2 - margin - base), font, scale, (255, 255, 255), thickness, cv2.LINE_AA)

def colorize_depth(depth_frame):
    depth = np.asanyarray(depth_frame.get_data())
    return cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)

def start_pipeline_for_serial(serial, width=FRAME_WIDTH, height=FRAME_HEIGHT, fps=FPS):
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    profile = pipe.start(cfg)
    align = rs.align(rs.stream.color)
    return pipe, align, profile

def pick_devices():
    ctx = rs.context()
    devs = list(ctx.query_devices())
    if not devs:
        print("No RealSense devices found."); sys.exit(1)

    infos = []
    for d in devs:
        name = d.get_info(rs.camera_info.name)
        serial = d.get_info(rs.camera_info.serial_number)
        infos.append((name.upper(), serial, name, d))

    def find_first(token):
        for nu, s, n, obj in infos:
            if token in nu:
                return s, n, obj
        return None, None, None

    camA_serial, camA_name, camA_obj = find_first("D435I")
    if not camA_serial:
        camA_serial, camA_name, camA_obj = find_first("D435")  # fallback (no IMU)
    if not camA_serial:
        print("Could not find a D435/D435i for Cam A.\nConnected devices:\n  - " +
              "\n  - ".join([f"{n} [{s}]" for _, s, n, _ in infos])); sys.exit(1)

    camB_serial, camB_name, camB_obj = find_first("D457")
    if not camB_serial:
        camB_serial, camB_name, camB_obj = find_first("D455")
    if not camB_serial:
        print("Could not find a D457 or D455 for Cam B.\nConnected devices:\n  - " +
              "\n  - ".join([f"{n} [{s}]" for _, s, n, _ in infos])); sys.exit(1)

    if camA_serial == camB_serial:
        for nu, s, n, obj in infos:
            if s != camA_serial and ("D457" in nu or "D455" in nu):
                camB_serial, camB_name, camB_obj = s, n, obj
                break
        if camA_serial == camB_serial:
            print("Only one suitable device found; need two distinct cameras.")
            sys.exit(1)

    # Report USB link speeds (3.2 / 3.1 / 2.1 etc.)
    usbA = camA_obj.get_info(rs.camera_info.usb_type_descriptor) if camA_obj.supports(rs.camera_info.usb_type_descriptor) else "unknown"
    usbB = camB_obj.get_info(rs.camera_info.usb_type_descriptor) if camB_obj.supports(rs.camera_info.usb_type_descriptor) else "unknown"
    print(f"Cam A -> {camA_name} [{camA_serial}] (USB {usbA})")
    print(f"Cam B -> {camB_name} [{camB_serial}] (USB {usbB})")
    if "2." in usbA or "2." in usbB:
        print("WARNING: One camera is on USB2.x — reduce resolution/FPS or move it to a USB3 port/controller.")

    return camA_serial, camB_serial, camA_name, camB_name

def warmup(pipe, align, label="cam", frames=WARMUP_FRAMES, timeout=WARMUP_TIMEOUT_MS):
    ok = 0
    while ok < frames:
        try:
            f = pipe.wait_for_frames(timeout)
        except RuntimeError:
            # Just try again; device might still be settling
            continue
        f = align.process(f)
        if f.get_color_frame() and f.get_depth_frame():
            ok += 1

def main():
    camA_serial, camB_serial, camA_name, camB_name = pick_devices()

    # Start both pipelines first (helps with USB scheduling)
    pA, alignA, profA = start_pipeline_for_serial(camA_serial)
    pB, alignB, profB = start_pipeline_for_serial(camB_serial)

    # Warm up both cameras (discard first N frames so AE/AF/align stabilizes)
    print("Warming up cameras…")
    warmup(pA, alignA, "A")
    warmup(pB, alignB, "B")
    print("Warmup done.")

    lastA = time.time(); fpsA = 0.0
    lastB = time.time(); fpsB = 0.0

    cv2.namedWindow(WINDOW_A, cv2.WINDOW_NORMAL)
    cv2.namedWindow(WINDOW_B, cv2.WINDOW_NORMAL)

    try:
        while True:
            # Get frames with a generous timeout; don't crash on occasional misses
            try:
                framesA = pA.wait_for_frames(WAIT_TIMEOUT_MS)
                framesA = alignA.process(framesA)
            except RuntimeError:
                # Skip this iteration for Cam A
                framesA = None

            try:
                framesB = pB.wait_for_frames(WAIT_TIMEOUT_MS)
                framesB = alignB.process(framesB)
            except RuntimeError:
                framesB = None

            # If neither produced frames this tick, continue
            if framesA is None and framesB is None:
                continue

            # Build panels (keep last good frame if one side misses this tick)
            panelA = None
            panelB = None

            if framesA and framesA.get_color_frame() and framesA.get_depth_frame():
                colA = rotate_frame(np.asanyarray(framesA.get_color_frame().get_data()), ROTATION_DEG)
                depA = rotate_frame(colorize_depth(framesA.get_depth_frame()), ROTATION_DEG)
                panelA = np.hstack([colA, depA])
                now = time.time()
                dtA = now - lastA
                if dtA > 0:
                    fpsA = 0.9*fpsA + 0.1*(1.0/dtA) if fpsA > 0 else (1.0/dtA)
                lastA = now

            if framesB and framesB.get_color_frame() and framesB.get_depth_frame():
                colB = rotate_frame(np.asanyarray(framesB.get_color_frame().get_data()), ROTATION_DEG)
                depB = rotate_frame(colorize_depth(framesB.get_depth_frame()), ROTATION_DEG)
                panelB = np.hstack([colB, depB])
                now = time.time()
                dtB = now - lastB
                if dtB > 0:
                    fpsB = 0.9*fpsB + 0.1*(1.0/dtB) if fpsB > 0 else (1.0/dtB)
                lastB = now

            if panelA is not None:
                draw_fps_top_right(panelA, f"FPS: {fpsA:5.1f}")
                cv2.imshow(WINDOW_A, panelA)
            if panelB is not None:
                draw_fps_top_right(panelB, f"FPS: {fpsB:5.1f}")
                cv2.imshow(WINDOW_B, panelB)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        for pipe in (pA, pB):
            try: pipe.stop()
            except Exception: pass
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
