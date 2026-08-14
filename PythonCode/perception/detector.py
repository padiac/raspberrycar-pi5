"""Object detection with YOLO26n.

Loads the NCNN export rather than the .pt weights: same model, but NCNN is
built for ARM CPUs and measured about four times faster on this Pi 5 -- 58 fps
against 14 fps at 320 px.
"""

import os
import threading

import cv2
import numpy as np

from .types import Detection

MODEL_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "models")
NCNN_MODEL = os.path.join(MODEL_DIR, "yolo26n_ncnn_model")
PT_MODEL = os.path.join(MODEL_DIR, "yolo26n.pt")
IMG_SIZE = 320


def colour_for(class_id):
    """Stable, well-separated colour per class."""
    hue = int((class_id * 47) % 180)
    bgr = cv2.cvtColor(np.uint8([[[hue, 200, 255]]]), cv2.COLOR_HSV2BGR)[0][0]
    return int(bgr[0]), int(bgr[1]), int(bgr[2])


class Detector:
    def __init__(self, confidence=0.4):
        from ultralytics import YOLO

        weights = NCNN_MODEL if os.path.isdir(NCNN_MODEL) else PT_MODEL
        self.model = YOLO(weights)
        self.backend = "ncnn" if weights == NCNN_MODEL else "pytorch"
        self.confidence = confidence
        self.classes = set()        # empty means "report everything"
        self.names = {}
        self.lock = threading.Lock()

    @property
    def class_names(self):
        return sorted(set(self.names.values())) if self.names else []

    def configure(self, confidence=None, classes=None):
        with self.lock:
            if confidence is not None:
                self.confidence = max(0.05, min(0.95, float(confidence)))
            if classes is not None:
                self.classes = {c.strip().lower() for c in classes if c.strip()}
        return self.status()

    def status(self):
        return {
            "backend": self.backend,
            "confidence": round(self.confidence, 2),
            "classes": sorted(self.classes),
        }

    def detect(self, frame):
        with self.lock:
            confidence, wanted = self.confidence, set(self.classes)

        result = self.model.predict(
            frame, imgsz=IMG_SIZE, conf=confidence, verbose=False, device="cpu"
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
            found.append(
                Detection(
                    label=label, confidence=float(score),
                    box=(x1, y1, x2, y2), class_id=class_id,
                )
            )
        return found


def draw(frame, detections):
    """Draw boxes and labels onto `frame`, in place."""
    for det in detections:
        x1, y1, x2, y2 = det.box
        colour = colour_for(det.class_id)
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)

        text = f"{det.label} {det.confidence:.2f}"
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        top = max(y1 - th - 6, 0)
        cv2.rectangle(frame, (x1, top), (x1 + tw + 6, top + th + 6), colour, -1)
        cv2.putText(
            frame, text, (x1 + 3, top + th + 1),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA,
        )
    return frame
