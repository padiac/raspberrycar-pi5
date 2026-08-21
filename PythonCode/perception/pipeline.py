"""Keeps frames flowing and detections fresh, on their own threads.

Capture and detection are deliberately separate. If they shared a thread the
video would only run as fast as the model, so switching detection on would
visibly stutter the picture; instead the capture loop draws whatever boxes the
detector produced most recently and never waits for it.

The same loop also encodes the JPEG the web stream serves, so annotation
happens once for both the browser and any behaviour overlay.
"""

import os
import threading
import time

import cv2
import numpy as np

from . import detector as detector_module

JPEG_QUALITY = int(os.environ.get("CAR_JPEG_QUALITY", "90"))

#: The browser gets its own, smaller copy.
#:
#: The full frames exist for the recorder and are worth their size there. Sent
#: to a phone they are a disaster: 1280x720 at quality 90, thirty times a
#: second, measured at 152KB a frame and 37 Mbit/s. No house wifi delivers that
#: to a handset, so the link saturates, TCP queues what it cannot send, and the
#: picture on the phone falls further and further behind -- which looks like
#: stuttering and is actually a growing backlog of frames that were true
#: several seconds ago.
#:
#: Motion Jpeg has no compression between frames, so a still room costs exactly
#: as much to send as a moving one. There is no cleverness to apply; the only
#: lever is to send less.
PREVIEW_WIDTH = int(os.environ.get("CAR_PREVIEW_WIDTH", "640"))
PREVIEW_QUALITY = int(os.environ.get("CAR_PREVIEW_QUALITY", "70"))
#: Driving does not need thirty. Half of them, at a fifth of the size, is a
#: tenth of the bytes and looks the same on a phone screen.
PREVIEW_FPS = float(os.environ.get("CAR_PREVIEW_FPS", "15"))

# Unthrottled the detector runs ~50 fps and eats about 3.4 of the Pi's 4 cores,
# starving the control loop. A robot reacts fine on 10 fps of detections.
DEFAULT_DETECT_FPS = 10.0


class FrameBuffer:
    """Holds the latest encoded frame and wakes up waiting stream clients."""

    def __init__(self):
        self.jpeg = None
        self.condition = threading.Condition()

    def publish(self, jpeg):
        with self.condition:
            self.jpeg = jpeg
            self.condition.notify_all()

    def wait_for_next(self, timeout=2.0):
        """Block for a new frame. Returns (got_new, jpeg)."""
        with self.condition:
            got = self.condition.wait(timeout=timeout)
            return got, self.jpeg


class VisionPipeline:
    def __init__(self, camera, detector=None):
        self.camera = camera
        self.detector = detector
        self.frames = FrameBuffer()
        #: What the browser watches: same pictures, much smaller.
        self.preview = FrameBuffer()
        self.preview_width = PREVIEW_WIDTH
        self.preview_quality = PREVIEW_QUALITY
        self.preview_fps = PREVIEW_FPS
        self._preview_at = 0.0

        self.detect_enabled = False
        self.detect_target_fps = DEFAULT_DETECT_FPS

        self._frame = None
        self._frame_size = camera.size
        self._detections = []

        self._frame_lock = threading.Lock()
        self._det_lock = threading.Lock()

        self.stream_fps = 0.0
        self.detect_fps = 0.0

        #: Set by the robot to let the active behaviour draw on the stream.
        self.overlay_hook = None

        self.running = True
        threading.Thread(target=self._capture_loop, daemon=True).start()
        threading.Thread(target=self._detect_loop, daemon=True).start()

    # -- what the control loop reads ---------------------------------------

    def snapshot(self):
        with self._frame_lock:
            frame = None if self._frame is None else self._frame.copy()
            size = self._frame_size
        with self._det_lock:
            detections = list(self._detections)
        return frame, size, detections

    @property
    def detections(self):
        with self._det_lock:
            return list(self._detections)

    # -- toggles -----------------------------------------------------------

    def set_detection(self, enabled):
        self.detect_enabled = bool(enabled) and self.detector is not None
        if not self.detect_enabled:
            with self._det_lock:
                self._detections = []
            self.detect_fps = 0.0
        return self.detect_enabled

    def set_camera(self, enabled):
        result = self.camera.set_enabled(enabled)
        if not self.camera.enabled:
            self.stream_fps = 0.0
            with self._frame_lock:
                self._frame = None
            with self._det_lock:
                self._detections = []
            self._publish_placeholder()
        return result

    # -- internals ---------------------------------------------------------

    def _publish_placeholder(self):
        """A visible 'camera off' frame, so the page does not just freeze."""
        width, height = self.camera.size
        img = np.full((height, width, 3), 22, dtype=np.uint8)
        text = "CAMERA OFF"
        scale = width / 580.0
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 2)
        cv2.putText(
            img, text, ((width - tw) // 2, (height + th) // 2),
            cv2.FONT_HERSHEY_SIMPLEX, scale, (90, 90, 90), 2, cv2.LINE_AA,
        )
        ok, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            self.frames.publish(jpeg.tobytes())
            self._publish_preview(img, force=True)

    def _publish_preview(self, image, force=False):
        """Encode a small copy for the browser, no faster than it needs.

        Rate-limited here rather than in the streaming endpoint because the
        saving that matters is the encode as well as the bytes: skipping a
        frame this early costs nothing at all.
        """
        now = time.perf_counter()
        interval = 1.0 / max(1.0, self.preview_fps)
        if not force and now - self._preview_at < interval:
            return
        self._preview_at = now

        height, width = image.shape[:2]
        if width > self.preview_width:
            scale = self.preview_width / float(width)
            image = cv2.resize(
                image, (self.preview_width, max(2, int(round(height * scale)))),
                interpolation=cv2.INTER_AREA)
        ok, jpeg = cv2.imencode(
            ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.preview_quality])
        if ok:
            self.preview.publish(jpeg.tobytes())

    def _capture_loop(self):
        smoothed = None
        start = time.perf_counter()
        while self.running:
            frame = self.camera.capture()
            if frame is None:
                smoothed = None
                time.sleep(0.15)
                start = time.perf_counter()
                continue

            with self._frame_lock:
                self._frame = frame
                self._frame_size = (frame.shape[1], frame.shape[0])

            annotated = frame
            if self.detect_enabled:
                current = self.detections
                if current:
                    annotated = detector_module.draw(frame.copy(), current)

            hook = self.overlay_hook
            if hook is not None:
                try:
                    annotated = hook(
                        annotated if annotated is not frame else frame.copy()
                    )
                except Exception:  # noqa: BLE001 - a bad overlay must not stop video
                    pass

            ok, jpeg = cv2.imencode(
                ".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
            )
            if ok:
                self.frames.publish(jpeg.tobytes())
            self._publish_preview(annotated)

            # Whole-loop time, capture wait included: this is the rate frames
            # actually reach the browser, not just how fast we can encode one.
            now = time.perf_counter()
            dt, start = now - start, now
            smoothed = dt if smoothed is None else smoothed * 0.9 + dt * 0.1
            self.stream_fps = 1.0 / smoothed if smoothed > 0 else 0.0

    def _detect_loop(self):
        smoothed = None
        while self.running:
            if not self.detect_enabled or self.detector is None or not self.camera.enabled:
                # Reset here rather than in set_detection(): a detect() call
                # already in flight when detection is switched off would
                # otherwise write its timing back afterwards.
                smoothed = None
                self.detect_fps = 0.0
                time.sleep(0.1)
                continue

            with self._frame_lock:
                frame = None if self._frame is None else self._frame.copy()
            if frame is None:
                time.sleep(0.02)
                continue

            start = time.perf_counter()
            try:
                found = self.detector.detect(frame)
            except Exception as exc:  # noqa: BLE001 - keep video going if a frame fails
                print(f"[detect] {exc}", flush=True)
                time.sleep(0.5)
                continue

            with self._det_lock:
                self._detections = found

            elapsed = time.perf_counter() - start
            if self.detect_target_fps > 0:
                time.sleep(max(0.0, (1.0 / self.detect_target_fps) - elapsed))

            dt = time.perf_counter() - start
            smoothed = dt if smoothed is None else smoothed * 0.8 + dt * 0.2
            self.detect_fps = 1.0 / smoothed if smoothed > 0 else 0.0

    def close(self):
        self.running = False
