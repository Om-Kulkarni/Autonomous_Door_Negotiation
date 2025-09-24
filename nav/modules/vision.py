# vision.py
import os
from pathlib import Path

# Match your original memory tuning
os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True,max_split_size_mb:64")


def enough_free_vram(min_free_mb: int = 400) -> bool:
    try:
        import torch
        if not torch.cuda.is_available():
            return False
        free, _total = torch.cuda.mem_get_info()
        return (free / (1024 ** 2)) >= min_free_mb
    except Exception:
        return False


def _select_device_and_precision(min_free_mb: int = 400):
    import torch
    use_gpu = torch.cuda.is_available() and enough_free_vram(min_free_mb)
    device = 0 if use_gpu else "cpu"
    half = bool(use_gpu)
    return device, half, use_gpu


def _show_frame(win_name: str, frame, exit_keys=(ord('q'), 27)) -> bool:
    import cv2
    cv2.imshow(win_name, frame)
    key = cv2.waitKey(1) & 0xFF
    if key in exit_keys:
        return False
    try:
        if cv2.getWindowProperty(win_name, cv2.WND_PROP_VISIBLE) < 1:
            return False
    except Exception:
        pass
    return True


def _infer_stream(
    model_path: Path,
    source,
    *,
    win_name: str,
    imgsz: int = 512,
    frame_skip_stride: int = 2,
    conf: float = 0.25,
    iou: float = 0.45,
    retina_masks: bool = False,
    max_det: int = 8,
    clear_interval: int = 50,
):
    if not Path(model_path).exists():
        raise FileNotFoundError(f"Model not found: {model_path}")

    if isinstance(source, (str, Path)) and not str(source).isdigit():
        if not Path(source).exists():
            raise FileNotFoundError(f"Video not found: {source}")

    from ultralytics import YOLO
    import torch
    import cv2

    device, half, use_gpu = _select_device_and_precision(400)
    print(f"Device: {'GPU' if use_gpu else 'CPU'} | half={half}")
    print(f"Loading model: {model_path}")

    model = YOLO(str(model_path))

    gen = model.predict(
        source=str(source),
        imgsz=imgsz,
        conf=conf,
        iou=iou,
        device=device,
        half=half,
        save=False,           # <-- never save
        project=None,
        name=None,
        exist_ok=True,
        vid_stride=frame_skip_stride,
        show=False,           # we'll draw + show ourselves
        retina_masks=retina_masks,
        max_det=max_det,
        stream=True,
        verbose=False,
    )

    processed = 0
    print("Running… Press 'q' or 'Esc' to quit.")
    try:
        for result in gen:
            processed += 1
            try:
                frame = result.plot()   # draw masks/boxes/labels
            except Exception:
                frame = result.orig_img

            if not _show_frame(win_name, frame):
                print("Exit requested — closing.")
                break

            if use_gpu and (processed % clear_interval == 0):
                try:
                    torch.cuda.synchronize()
                    torch.cuda.empty_cache()
                except Exception:
                    pass
    finally:
        if use_gpu:
            try:
                torch.cuda.synchronize()
                torch.cuda.empty_cache()
            except Exception:
                pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    print("\n=== Inference finished ===")


def infer_on_video(
    model_path: Path,
    video_path: Path,
    *,
    imgsz: int = 512,
    frame_skip_stride: int = 2,
    conf: float = 0.25,
    iou: float = 0.45,
    retina_masks: bool = False,
    max_det: int = 8,
    clear_interval: int = 50,
    window_name: str = "YOLOv11-Seg (Video)",
):
    _infer_stream(
        model_path=model_path,
        source=str(video_path),
        win_name=window_name,
        imgsz=imgsz,
        frame_skip_stride=frame_skip_stride,
        conf=conf,
        iou=iou,
        retina_masks=retina_masks,
        max_det=max_det,
        clear_interval=clear_interval,
    )


def infer_realtime(
    model_path: Path,
    camera_index: int = 0,
    *,
    imgsz: int = 512,
    frame_skip_stride: int = 1,
    conf: float = 0.25,
    iou: float = 0.45,
    retina_masks: bool = False,
    max_det: int = 8,
    clear_interval: int = 50,
    window_name: str = "YOLOv11-Seg (Realtime)",
):
    _infer_stream(
        model_path=model_path,
        source=str(camera_index),
        win_name=window_name,
        imgsz=imgsz,
        frame_skip_stride=frame_skip_stride,
        conf=conf,
        iou=iou,
        retina_masks=retina_masks,
        max_det=max_det,
        clear_interval=clear_interval,
    )
