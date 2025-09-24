import os
from pathlib import Path
from datetime import datetime
import cv2

# --- paths ---
MODEL_PATH = Path("/home/siddharth/AutoAnnotatorPy/model_training/annotations/real/output/seg_20250924_134622/weights/best.pt")
VIDEO_PATH = Path("/home/siddharth/AutoAnnotatorPy/model_training/annotations/real/test/straight_test/video_rgb_1_pull.mp4")

# Be aggressive about fragmentation
os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True,max_split_size_mb:64")


def enough_free_vram(min_free_mb=400):
    try:
        import torch
        if not torch.cuda.is_available():
            return False
        free, total = torch.cuda.mem_get_info()
        return (free / (1024**2)) >= min_free_mb
    except Exception:
        return False


def main():
    if not MODEL_PATH.exists():
        raise FileNotFoundError(f"Model not found: {MODEL_PATH}")
    if not VIDEO_PATH.exists():
        raise FileNotFoundError(f"Video not found: {VIDEO_PATH}")

    from ultralytics import YOLO
    import torch

    # Choose device: prefer GPU, but fall back if almost full
    use_gpu = torch.cuda.is_available() and enough_free_vram(400)
    device = 0 if use_gpu else "cpu"
    half = use_gpu  # FP16 only on GPU

    print(f"Device: {'GPU' if use_gpu else 'CPU'} | half={half}")
    print(f"Loading model: {MODEL_PATH}")
    model = YOLO(str(MODEL_PATH))

    frame_skip_stride = 2         # change to 1 if you want every frame
    imgsz = 512
    clear_interval = 50

    # Run inference on video, streaming frame by frame
    gen = model.predict(
        source=str(VIDEO_PATH),
        imgsz=imgsz,
        conf=0.25,
        iou=0.45,
        device=device,
        half=half,
        vid_stride=frame_skip_stride,
        show=False,
        retina_masks=False,
        max_det=8,
        stream=True
    )

    processed = 0
    try:
        for result in gen:
            processed += 1

            # Get annotated frame directly from Ultralytics
            frame = result.plot()  # numpy BGR image with masks, boxes, labels

            cv2.imshow("YOLOv11 Segmentation", frame)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit
                break

            if use_gpu and (processed % clear_interval == 0):
                try:
                    torch.cuda.synchronize()
                    torch.cuda.empty_cache()
                except Exception:
                    pass
    finally:
        cv2.destroyAllWindows()
        if use_gpu:
            try:
                torch.cuda.synchronize()
                torch.cuda.empty_cache()
            except Exception:
                pass


if __name__ == "__main__":
    main()
