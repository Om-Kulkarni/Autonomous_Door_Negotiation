# sanity_class_map.py
from ultralytics import YOLO
import cv2

MODEL = "best.pt"
FRAME = "frame_000478.jpg"   # grab a frame from the video for reproducibility

m = YOLO(MODEL)
img = cv2.cvtColor(cv2.imread(FRAME), cv2.COLOR_BGR2RGB)
r = m.predict(img, imgsz=640, conf=0.05, iou=0.5, verbose=False)[0]  # low conf to see everything

print("Model class table:", r.names)  # <-- truth from the weights
if r.boxes is not None:
    for i in range(len(r.boxes)):
        cls_id = int(r.boxes.cls[i].item())
        conf   = float(r.boxes.conf[i].item())
        print(f"det#{i}: cls_id={cls_id} name={r.names[cls_id]} conf={conf:.3f}")
