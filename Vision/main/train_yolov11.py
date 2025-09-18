#!/usr/bin/env python3
"""
End-to-end YOLOv11 training script (uv/venv friendly) with:
- Memory handling (CUDA allocator tweak)
- Auto-retry on CUDA OOM (reduces batch/imgsz/workers)
- Auto-resume from last best checkpoint

Usage examples:
  uv run python train_yolov11.py
  uv run python train_yolov11.py --model yolo11n.pt --epochs 200 --batch 4 --imgsz 512 --device 0
  uv run python train_yolov11.py --skip-download --resume  # force resume
"""

import argparse
import json
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional, Tuple, List

# ---- Memory handling: mitigate CUDA fragmentation ----
os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")

# ---------- Roboflow dataset download (YOUR VALUES) ----------
from roboflow import Roboflow

ROBOFLOW_API_KEY = "NJsPl8PO4pm1ic91oJo0"
WORKSPACE = "projects-mpqcn"
PROJECT = "door-detection-sblml-gmhne"
VERSION = 2
EXPORT_FORMAT = "yolov11"  # or "yolov8"


# ---------------- Helpers ----------------
def run_cmd(cmd: List[str]) -> None:
    print("Running:\n", " ".join(shlex.quote(x) for x in cmd))
    subprocess.run(cmd, check=True)


def download_dataset() -> str:
    print("loading Roboflow workspace...")
    rf = Roboflow(api_key=ROBOFLOW_API_KEY)
    print("loading Roboflow project...")
    project = rf.workspace(WORKSPACE).project(PROJECT)
    version = project.version(VERSION)
    dataset = version.download(EXPORT_FORMAT)  # has .location
    dataset_dir = dataset.location
    data_yaml = os.path.join(dataset_dir, "data.yaml")
    print("DATASET DIR:", dataset_dir)
    print("DATA.YAML  :", data_yaml)

    os.makedirs("artifacts", exist_ok=True)
    with open("artifacts/dataset_meta.json", "w") as f:
        json.dump({"dataset_dir": dataset_dir, "data_yaml": data_yaml}, f)
    return data_yaml


def load_data_yaml() -> str:
    with open("artifacts/dataset_meta.json", "r") as f:
        meta = json.load(f)
    return meta["data_yaml"]


def find_yolo_executable() -> List[str]:
    """
    Use the 'yolo' CLI that lives next to the current Python (venv/uv),
    else fall back to `python -m ultralytics`.
    """
    py_dir = Path(sys.executable).parent
    cand = py_dir / "yolo"
    if cand.exists() and os.access(cand, os.X_OK):
        return [str(cand)]
    return [sys.executable, "-m", "ultralytics"]


def locate_latest_best(project: str, name: str) -> Optional[Path]:
    """
    Find the most recent run dir under project/ matching `name` or `name*`
    that contains weights/best.pt, return that path.
    """
    proj = Path(project)
    if not proj.exists():
        return None

    candidates: List[Tuple[float, Path]] = []
    for d in proj.glob(f"{name}*"):
        best = d / "weights" / "best.pt"
        if best.exists():
            # use mtime of best.pt (or dir) to pick newest
            mtime = best.stat().st_mtime
            candidates.append((mtime, best))

    if not candidates:
        return None

    candidates.sort(key=lambda x: x[0], reverse=True)
    return candidates[0][1]  # latest best.pt


def build_train_cmd(
    data_yaml: str,
    args: argparse.Namespace,
    *,
    override_batch: Optional[int] = None,
    override_imgsz: Optional[int] = None,
    override_workers: Optional[int] = None,
    override_model: Optional[str] = None,
    force_resume: Optional[bool] = None,
) -> List[str]:
    """
    Build Ultralytics CLI command with optional overrides used for auto-retry.
    """
    model = override_model if override_model is not None else args.model
    imgsz = override_imgsz if override_imgsz is not None else args.imgsz
    batch = override_batch if override_batch is not None else args.batch
    workers = override_workers if override_workers is not None else args.workers
    resume = args.resume if force_resume is None else force_resume

    cli_args = []
    if model:        cli_args += [f"model={model}"]
    if imgsz:        cli_args += [f"imgsz={imgsz}"]
    if args.epochs is not None: cli_args += [f"epochs={args.epochs}"]
    if batch is not None:       cli_args += [f"batch={batch}"]
    if args.device is not None: cli_args += [f"device={args.device}"]
    if workers is not None:     cli_args += [f"workers={workers}"]
    if args.amp is not None:    cli_args += [f"amp={str(args.amp).lower()}"]
    if args.project:            cli_args += [f"project={args.project}"]
    if args.name:               cli_args += [f"name={args.name}"]
    if resume:                  cli_args += [f"resume={str(resume).lower()}"]

    yolo = find_yolo_executable()
    cmd = yolo + ["detect", "train", f"data={data_yaml}"] + cli_args
    return cmd


def is_oom_error(exc: subprocess.CalledProcessError) -> bool:
    s = ""
    try:
        s = exc.stderr.decode() if isinstance(exc.stderr, (bytes, bytearray)) else str(exc)
    except Exception:
        s = str(exc)
    # Heuristics for OOM markers from PyTorch/Ultralytics
    markers = [
        "CUDA out of memory",
        "CUBLAS_STATUS_ALLOC_FAILED",
        "RuntimeError: CUDA error",  # generic
    ]
    return any(m in s for m in markers)


# ---------------- Main ----------------
def main():
    parser = argparse.ArgumentParser(description="Train YOLOv11 on Roboflow dataset (with memory auto-retry and auto-resume)")
    parser.add_argument("--model", default="yolo11s.pt")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch", type=int, default=8)          # use -1 for Ultralytics auto-batch
    parser.add_argument("--device", default="0", help="GPU id or 'cpu'")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--amp", type=lambda s: s.lower() in ("1","true","yes"), default=True)
    parser.add_argument("--project", default="runs")
    parser.add_argument("--name", default="exp_yolo11s")
    parser.add_argument("--resume", action="store_true", help="Force Ultralytics resume=True")
    parser.add_argument("--auto-resume", action="store_true", default=True, help="If best checkpoint exists, resume from it")
    parser.add_argument("--skip-download", action="store_true", help="Skip dataset download if metadata exists")
    parser.add_argument("--retries", type=int, default=2, help="Additional retry attempts after first OOM")
    parser.add_argument("--downscale-imgsz", type=int, nargs="*", default=[512, 448], help="imgsz fallbacks per retry")
    parser.add_argument("--min-batch", type=int, default=1, help="lowest batch size to try on retry")
    parser.add_argument("--min-workers", type=int, default=0, help="lowest dataloader workers to try on retry")
    args = parser.parse_args()

    # Dataset path
    if args.skip_download and os.path.exists("artifacts/dataset_meta.json"):
        data_yaml = load_data_yaml()
    else:
        data_yaml = download_dataset()

    # Auto-resume from latest best.pt unless user explicitly passed --resume already
    resume_flag = args.resume
    model_override: Optional[str] = None
    if args.auto_resume and not resume_flag:
        best = locate_latest_best(args.project, args.name)
        if best:
            print(f"[auto-resume] Found best checkpoint: {best}")
            model_override = str(best)
            resume_flag = True

    # First attempt
    cmd = build_train_cmd(
        data_yaml,
        args,
        override_model=model_override,
        force_resume=resume_flag
    )

    try:
        run_cmd(cmd)
        return
    except subprocess.CalledProcessError as e:
        if not is_oom_error(e) or args.retries <= 0:
            raise

        print("\n[WARN] CUDA OOM detected. Starting auto-retry sequence...\n")

    # Retry loop on OOM: reduce batch -> reduce imgsz -> reduce workers
    # Start from current values (or sensible defaults)
    batch = args.batch
    imgsz = args.imgsz
    workers = args.workers

    # If user used auto-batch (-1) and still OOM'd, start with a small fixed batch
    if batch is None or batch == -1:
        batch = 4

    # build a sequence of (batch, imgsz, workers) to try
    retry_plan = []

    # 1) Halve batch a couple of times
    b = max(args.min_batch, batch // 2)
    if b < batch:
        retry_plan.append((b, imgsz, workers))

    b2 = max(args.min_batch, b // 2)
    if b2 < b:
        retry_plan.append((b2, imgsz, workers))

    # 2) Keep batch lower, reduce imgsz by the provided scales
    for new_img in args.downscale_imgsz:
        retry_plan.append((max(args.min_batch, b2), new_img, workers))

    # 3) As last resort, also cut workers
    if workers > args.min_workers:
        retry_plan.append((max(args.min_batch, b2), args.downscale_imgsz[-1] if args.downscale_imgsz else imgsz, args.min_workers))

    # Cap number of tries
    retry_plan = retry_plan[:args.retries]

    for i, (rbatch, rimg, rworkers) in enumerate(retry_plan, start=1):
        print(f"[retry {i}/{len(retry_plan)}] Trying batch={rbatch}, imgsz={rimg}, workers={rworkers}")
        cmd = build_train_cmd(
            data_yaml,
            args,
            override_model=model_override,
            force_resume=resume_flag,
            override_batch=rbatch,
            override_imgsz=rimg,
            override_workers=rworkers,
        )
        try:
            run_cmd(cmd)
            print("[retry] Succeeded.")
            return
        except subprocess.CalledProcessError as e:
            if is_oom_error(e) and i < len(retry_plan):
                print("[retry] OOM again, moving to next fallback...\n")
                time.sleep(1.0)
                continue
            raise


if __name__ == "__main__":
    main()
