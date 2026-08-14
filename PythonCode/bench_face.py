"""Benchmark face detection on this Pi, on real camera frames."""
import time

import cv2
import numpy as np

MODEL = "../models/yunet.onnx"


def bench(frame, size, runs=40):
    det = cv2.FaceDetectorYN.create(MODEL, "", size, 0.7, 0.3, 5000)
    small = cv2.resize(frame, size)
    det.detect(small)  # warm-up, keeps first-call setup out of the timing
    t0 = time.perf_counter()
    faces = None
    for _ in range(runs):
        _, faces = det.detect(small)
    dt = (time.perf_counter() - t0) / runs
    n = 0 if faces is None else len(faces)
    print(f"  {size[0]}x{size[1]:<4} {dt*1000:6.1f} ms/frame   {1/dt:5.1f} fps   faces found: {n}")


if __name__ == "__main__":
    from picamera2 import Picamera2

    cam = Picamera2()
    cam.configure(cam.create_still_configuration(main={"size": (640, 480), "format": "RGB888"}))
    cam.start()
    time.sleep(2)
    frame = cam.capture_array()
    cam.close()
    print("captured frame:", frame.shape)

    print("\nYuNet face detection (single core, CPU):")
    for size in [(160, 120), (320, 240), (640, 480)]:
        bench(frame, size)

    cv2.setNumThreads(4)
    print("\nsame, with all 4 cores:")
    for size in [(320, 240), (640, 480)]:
        bench(frame, size)
