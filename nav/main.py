# main.py
# Edit ONLY the two paths below. Then:
# - Video mode:     python main.py
# - Realtime mode:  python main.py --realtime [--camera 0]
#
# Nothing is saved; masks are drawn on a cv2 window.

from pathlib import Path
import argparse
from modules.vision import infer_on_video

# === EDIT THESE PATHS ===
MODEL_PATH = Path("/home/siddharth/AutoAnnotatorPy/model_training/annotations/real/output/seg_20250924_134622/weights/best.pt")
VIDEO_PATH = Path("/home/siddharth/AutoAnnotatorPy/model_training/annotations/real/test/straight_test/video_rgb_1_pull.mp4")

# === TUNABLE DEFAULTS (kept close to your original) ===
IMG_SIZE = 512
CONF = 0.25
IOU = 0.45
MAX_DET = 8
CLEAR_INTERVAL = 50
FRAME_SKIP_VIDEO = 2        # like original
FRAME_SKIP_REALTIME = 1     # lower latency
RETINA_MASKS = False        # memory-friendly like original
DEFAULT_CAMERA = 0


def main():
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--realtime", action="store_true", help="Use webcam instead of video file.")
    ap.add_argument("--camera", type=int, default=DEFAULT_CAMERA, help="Camera index (realtime).")
    args = ap.parse_args()

    if args.realtime:
        # --- Realtime: RealSense RGB+Depth *and* YOLO on RGB (no saving) ---
        import pyrealsense2 as rs
        import numpy as np
        import cv2
        import torch
        from ultralytics import YOLO
        from modules.vision import enough_free_vram
        from modules.object_pose_est import rotate_frame

        use_gpu = torch.cuda.is_available() and enough_free_vram(400)
        device = 0 if use_gpu else "cpu"
        half = bool(use_gpu)

        print(f"Device: {'GPU' if use_gpu else 'CPU'} | half={half}")
        print(f"Loading model: {MODEL_PATH}")
        model = YOLO(str(MODEL_PATH))

        pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = pipeline.start(cfg)
        align = rs.align(rs.stream.color)

        rotation = 90  # fixed rotation
        window_name = "RGB+YOLO | Depth (rotated)"

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

                depth = np.asanyarray(depth_frame.get_data())
                color = np.asanyarray(color_frame.get_data())

                # Rotate before inference
                color_for_infer = rotate_frame(color, rotation=rotation)

                # ---- YOLO on rotated RGB only ----
                results = model.predict(
                    source=color_for_infer,
                    imgsz=IMG_SIZE,
                    conf=CONF,
                    iou=IOU,
                    device=device,
                    half=half,
                    save=False,
                    show=False,
                    retina_masks=RETINA_MASKS,
                    max_det=MAX_DET,
                    stream=False,
                    verbose=False,
                )
                result = results[0]
                color_annot = result.plot() if result is not None else color_for_infer

                # Depth visualization (rotate to match)
                depth_viz = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth, alpha=0.03),
                    cv2.COLORMAP_JET
                )
                depth_rot = rotate_frame(depth_viz, rotation=rotation)

                # Side-by-side view
                stacked = np.hstack([color_annot, depth_rot])
                cv2.imshow(window_name, stacked)

                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    break

                try:
                    if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                        break
                except Exception:
                    pass

                processed += 1
                if use_gpu and (processed % CLEAR_INTERVAL == 0):
                    try:
                        torch.cuda.synchronize()
                        torch.cuda.empty_cache()
                    except Exception:
                        pass
            print("\n=== Realtime finished ===")
        finally:
            pipeline.stop()
            cv2.destroyAllWindows()
        # -------------------------------------------------------------------------------
    else:
        infer_on_video(
            model_path=MODEL_PATH,
            video_path=VIDEO_PATH,
            imgsz=IMG_SIZE,
            frame_skip_stride=FRAME_SKIP_VIDEO,
            conf=CONF,
            iou=IOU,
            retina_masks=RETINA_MASKS,
            max_det=MAX_DET,
            clear_interval=CLEAR_INTERVAL,
            window_name="YOLOv11-Seg (Video)",
        )


if __name__ == "__main__":
    main()
