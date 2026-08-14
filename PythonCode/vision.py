"""Camera capture, object detection and MJPEG streaming.

One capture thread and one detection thread. Keeping them apart means a slow
model can't drag the video's frame rate down: the capture loop just overlays
whatever boxes the detector produced most recently.
"""

import os
import threading
import time

import cv2
import numpy as np

STREAM_SIZE = tuple(
    int(v) for v in os.environ.get("CAR_STREAM_SIZE", "1280x720").split("x")
)
JPEG_QUALITY = int(os.environ.get("CAR_JPEG_QUALITY", "90"))

# Left unthrottled the detector runs ~50 fps and eats ~3.4 of the Pi's 4
# cores, starving the driving loop. A robot reacts fine on 10 fps.
DEFAULT_DETECT_FPS = 10.0


class StreamingOutput:
    """Holds the most recent JPEG frame and wakes up waiting stream clients."""

    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()

    def publish(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


class CameraPipeline:
    def __init__(self, picam, detector):
        self.picam = picam
        self.detector = detector
        self.output = StreamingOutput()

        self.camera_enabled = True
        self.detect_enabled = False
        self.detect_target_fps = DEFAULT_DETECT_FPS

        self.detections = []
        self.latest_frame = None
        self.frame_size = STREAM_SIZE

        self.frame_lock = threading.Lock()
        self.det_lock = threading.Lock()
        self.camera_lock = threading.Lock()

        self.stream_fps = 0.0
        self.detect_fps = 0.0
        self.running = True

        # Modes can set this to draw their own debug graphics on the stream
        # (lane points, target lock-on, and so on).
        self.overlay_hook = None

        threading.Thread(target=self._capture_loop, daemon=True).start()
        threading.Thread(target=self._detect_loop, daemon=True).start()

    # -- snapshots for the control loop ----------------------------------

    def snapshot_frame(self):
        with self.frame_lock:
            return None if self.latest_frame is None else self.latest_frame.copy()

    def snapshot_detections(self):
        with self.det_lock:
            return list(self.detections)

    # -- toggles ----------------------------------------------------------

    def set_detection(self, enabled):
        self.detect_enabled = bool(enabled) and self.detector is not None
        if not self.detect_enabled:
            with self.det_lock:
                self.detections = []
            self.detect_fps = 0.0
        return self.detect_enabled

    def set_camera(self, enabled):
        """Really stop the sensor when off, rather than just dropping frames."""
        enabled = bool(enabled)
        with self.camera_lock:
            if enabled == self.camera_enabled:
                return self.camera_enabled
            try:
                if enabled:
                    self.picam.start()
                    time.sleep(0.5)  # let auto-exposure settle before streaming
                else:
                    self.picam.stop()
            except Exception as exc:  # noqa: BLE001 - keep serving even if this fails
                print(f"[camera] could not {'start' if enabled else 'stop'}: {exc}", flush=True)
                return self.camera_enabled
            self.camera_enabled = enabled

        if not enabled:
            self.stream_fps = 0.0
            with self.frame_lock:
                self.latest_frame = None
            with self.det_lock:
                self.detections = []
            self._publish_placeholder()
        return self.camera_enabled

    # -- internals --------------------------------------------------------

    def _publish_placeholder(self):
        """A visible 'camera off' frame, so the page doesn't just freeze."""
        w, h = STREAM_SIZE
        img = np.full((h, w, 3), 22, dtype=np.uint8)
        text = "CAMERA OFF"
        scale = w / 580.0
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 2)
        cv2.putText(
            img, text, ((w - tw) // 2, (h + th) // 2),
            cv2.FONT_HERSHEY_SIMPLEX, scale, (90, 90, 90), 2, cv2.LINE_AA,
        )
        ok, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            self.output.publish(jpeg.tobytes())

    def _capture_loop(self):
        smoothed = None
        start = time.perf_counter()
        while self.running:
            if not self.camera_enabled:
                smoothed = None
                time.sleep(0.15)
                start = time.perf_counter()
                continue

            try:
                frame = self.picam.capture_array()
            except Exception:  # noqa: BLE001 - e.g. stopped mid-capture by the toggle
                time.sleep(0.1)
                continue

            with self.frame_lock:
                self.latest_frame = frame
                self.frame_size = (frame.shape[1], frame.shape[0])

            annotated = frame
            overlay = self.overlay_hook
            if self.detect_enabled:
                current = self.snapshot_detections()
                if current:
                    from detector import draw

                    annotated = draw(frame.copy(), current)
            if overlay is not None:
                annotated = overlay(annotated if annotated is not frame else frame.copy())

            ok, jpeg = cv2.imencode(
                ".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
            )
            if ok:
                self.output.publish(jpeg.tobytes())

            # Whole-loop time, capture wait included -- this is the rate frames
            # actually reach the browser, not just how fast we can encode one.
            now = time.perf_counter()
            dt, start = now - start, now
            smoothed = dt if smoothed is None else smoothed * 0.9 + dt * 0.1
            self.stream_fps = 1.0 / smoothed if smoothed > 0 else 0.0

    def _detect_loop(self):
        smoothed = None
        while self.running:
            if not self.detect_enabled or self.detector is None or not self.camera_enabled:
                # Reset here rather than in set_detection(): a detect() call
                # already in flight when detection is switched off would
                # otherwise write its timing back afterwards.
                smoothed = None
                self.detect_fps = 0.0
                time.sleep(0.1)
                continue

            frame = self.snapshot_frame()
            if frame is None:
                time.sleep(0.02)
                continue

            start = time.perf_counter()
            try:
                found = self.detector.detect(frame)
            except Exception as exc:  # noqa: BLE001 - keep streaming if a frame fails
                print(f"[detect] {exc}", flush=True)
                time.sleep(0.5)
                continue

            with self.det_lock:
                self.detections = found

            elapsed = time.perf_counter() - start
            target = self.detect_target_fps
            if target > 0:
                time.sleep(max(0.0, (1.0 / target) - elapsed))

            dt = time.perf_counter() - start
            smoothed = dt if smoothed is None else smoothed * 0.8 + dt * 0.2
            self.detect_fps = 1.0 / smoothed if smoothed > 0 else 0.0

    def close(self):
        self.running = False
