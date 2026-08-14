"""Is the picture soft because of focus, or just because of resolution?

Laplacian variance is the usual focus metric: high = crisp edges, low = blur.
Comparing full-res against the downscaled stream size separates "the lens is
out of focus" from "we're only streaming 640x480".
"""
import time

import cv2
import numpy as np
from picamera2 import Picamera2


def sharpness(img):
    grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return cv2.Laplacian(grey, cv2.CV_64F).var()


def grab(cam, size):
    cam.stop()
    cam.configure(cam.create_still_configuration(main={"size": size, "format": "RGB888"}))
    cam.start()
    time.sleep(1.5)
    return cam.capture_array()


if __name__ == "__main__":
    cam = Picamera2()

    full = grab(cam, (3280, 2464))
    h, w = full.shape[:2]
    centre = full[h // 3: 2 * h // 3, w // 3: 2 * w // 3]

    print(f"full res {w}x{h}")
    print(f"  sharpness whole frame : {sharpness(full):8.1f}")
    print(f"  sharpness centre third: {sharpness(centre):8.1f}")
    cv2.imwrite("/tmp/focus_full.jpg", full, [cv2.IMWRITE_JPEG_QUALITY, 95])
    cv2.imwrite("/tmp/focus_centre.jpg", centre, [cv2.IMWRITE_JPEG_QUALITY, 95])

    print("\nsharpness of the same scene captured at each size:")
    for size in [(640, 480), (1280, 960), (1640, 1232)]:
        img = grab(cam, size)
        print(f"  {size[0]:>4}x{size[1]:<5} {sharpness(img):8.1f}")

    print("\nfull frame downscaled in software (not re-captured):")
    for size in [(640, 480), (1280, 960)]:
        print(f"  {size[0]:>4}x{size[1]:<5} {sharpness(cv2.resize(full, size)):8.1f}")

    cam.close()
    print("\nrule of thumb: <100 is visibly soft, 100-500 acceptable, >500 crisp")
