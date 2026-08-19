"""Benchmark general object detectors on this Pi, on a real photo."""
import time
import warnings

import cv2
import numpy as np
import torch

warnings.filterwarnings("ignore")
IMG = "/tmp/face_test.jpg"
RUNS = 8


def timeit(fn, runs=RUNS):
    fn()  # warm-up
    t0 = time.perf_counter()
    for _ in range(runs):
        out = fn()
    return (time.perf_counter() - t0) / runs, out


def bench_ssdlite(frame):
    from torchvision.models.detection import (
        SSDLite320_MobileNet_V3_Large_Weights,
        ssdlite320_mobilenet_v3_large,
    )

    w = SSDLite320_MobileNet_V3_Large_Weights.COCO_V1
    model = ssdlite320_mobilenet_v3_large(weights=w).eval()
    tensor = w.transforms()(torch.from_numpy(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)).permute(2, 0, 1))

    def run():
        with torch.no_grad():
            return model([tensor])[0]

    dt, out = timeit(run)
    names = w.meta["categories"]
    labels = [f"{names[l]} {s:.2f}" for l, s in zip(out["labels"], out["scores"]) if s > 0.4]
    return dt, labels


def bench_yolo(frame, weights, tag):
    from ultralytics import YOLO

    model = YOLO(weights)

    def run():
        return model.predict(frame, imgsz=320, verbose=False, device="cpu")[0]

    dt, out = timeit(run)
    labels = [
        f"{out.names[int(c)]} {float(s):.2f}"
        for c, s in zip(out.boxes.cls, out.boxes.conf)
        if float(s) > 0.4
    ]
    return dt, labels


if __name__ == "__main__":
    frame = cv2.resize(cv2.imread(IMG), (640, 480))
    print(f"test frame: {frame.shape}\n")
    results = []

    for name, fn in [
        ("SSDLite320 MobileNetV3 (PyTorch)", lambda: bench_ssdlite(frame)),
        ("YOLO26n (PyTorch)", lambda: bench_yolo(frame, "yolo26n.pt", "pt")),
        ("YOLO26n (NCNN, ARM-optimised)", lambda: bench_yolo(frame, "yolo26n_ncnn_model", "ncnn")),
    ]:
        try:
            dt, labels = fn()
            results.append((name, dt, labels))
            print(f"{name}\n   {dt*1000:7.0f} ms   {1/dt:5.2f} fps   -> {', '.join(labels) or '(nothing above 0.4)'}\n")
        except Exception as exc:
            print(f"{name}\n   FAILED: {exc}\n")

    if results:
        print("=" * 62)
        for name, dt, _ in sorted(results, key=lambda r: r[1]):
            print(f"{1/dt:6.2f} fps   {name}")
