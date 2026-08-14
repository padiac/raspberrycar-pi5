"""YOLO26n object detection, wrapped for use on the Pi's CPU.

Uses the NCNN export rather than the .pt weights: same model, but NCNN is
built for ARM CPUs and measured ~4x faster here (58 fps vs 14 fps at 320px).
"""

import os
import threading

import cv2

MODEL_DIR = os.path.join(os.path.dirname(__file__), "..", "models")
NCNN_MODEL = os.path.join(MODEL_DIR, "yolo26n_ncnn_model")
PT_MODEL = os.path.join(MODEL_DIR, "yolo26n.pt")
IMG_SIZE = 320


def _colour(class_id):
    """Stable, well-separated colour per class id."""
    hue = int((class_id * 47) % 180)
    bgr = cv2.cvtColor(
        __import__("numpy").uint8([[[hue, 200, 255]]]), cv2.COLOR_HSV2BGR
    )[0][0]
    return int(bgr[0]), int(bgr[1]), int(bgr[2])


class Detector:
    def __init__(self, conf=0.4):
        from ultralytics import YOLO

        weights = NCNN_MODEL if os.path.isdir(NCNN_MODEL) else PT_MODEL
        self.model = YOLO(weights)
        self.backend = "ncnn" if weights == NCNN_MODEL else "pytorch"
        self.conf = conf
        self.classes = set()  # empty == show everything
        self.lock = threading.Lock()
        self.names = {}

    @property
    def class_names(self):
        return sorted(set(self.names.values())) if self.names else []

    def configure(self, conf=None, classes=None):
        with self.lock:
            if conf is not None:
                self.conf = max(0.05, min(0.95, float(conf)))
            if classes is not None:
                self.classes = {c.strip().lower() for c in classes if c.strip()}

    def detect(self, frame):
        """Return [(label, confidence, (x1, y1, x2, y2), class_id), ...]."""
        with self.lock:
            conf, wanted = self.conf, set(self.classes)

        result = self.model.predict(
            frame, imgsz=IMG_SIZE, conf=conf, verbose=False, device="cpu"
        )[0]
        self.names = result.names

        found = []
        for box, cls, score in zip(
            result.boxes.xyxy, result.boxes.cls, result.boxes.conf
        ):
            class_id = int(cls)
            label = result.names[class_id]
            if wanted and label.lower() not in wanted:
                continue
            x1, y1, x2, y2 = (int(v) for v in box)
            found.append((label, float(score), (x1, y1, x2, y2), class_id))
        return found


def draw(frame, detections):
    """Draw boxes and labels onto frame in place."""
    for label, score, (x1, y1, x2, y2), class_id in detections:
        colour = _colour(class_id)
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)

        text = f"{label} {score:.2f}"
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        top = max(y1 - th - 6, 0)
        cv2.rectangle(frame, (x1, top), (x1 + tw + 6, top + th + 6), colour, -1)
        cv2.putText(
            frame, text, (x1 + 3, top + th + 1),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA,
        )
    return frame
