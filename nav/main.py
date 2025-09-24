# main.py
# Edit ONLY the two paths below. Then:
# - Video mode:     python main.py
# - Realtime mode:  python main.py --realtime [--camera 0]
#
# Nothing is saved; masks are drawn on a cv2 window.

from pathlib import Path
import argparse
from modules.vision import infer_on_video, infer_realtime

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
        infer_realtime(
            model_path=MODEL_PATH,
            camera_index=args.camera,
            imgsz=IMG_SIZE,
            frame_skip_stride=FRAME_SKIP_REALTIME,
            conf=CONF,
            iou=IOU,
            retina_masks=RETINA_MASKS,
            max_det=MAX_DET,
            clear_interval=CLEAR_INTERVAL,
            window_name="YOLOv11-Seg (Realtime)",
        )
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
