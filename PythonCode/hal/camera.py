"""The camera, as a source of BGR frames.

Wraps picamera2 so nothing above has to know about libcamera configuration, and
so "off" really means the sensor is stopped rather than frames being thrown
away -- on this Pi that is the difference between 25% and 2% CPU.
"""

import os
import threading
import time

DEFAULT_SIZE = "1280x720"


def _parse_size(text):
    width, _, height = text.partition("x")
    return int(width), int(height)


from .base import FrameSource, driver


class Camera(FrameSource):
    label = "IMX219 on the CSI ribbon"
    def __init__(self, size=None):
        from picamera2 import Picamera2

        self.size = _parse_size(size or os.environ.get("CAR_STREAM_SIZE", DEFAULT_SIZE))
        self.picam = Picamera2()
        # picamera2 calls this format RGB888, but it hands over the bytes in
        # BGR order, which is exactly what OpenCV expects.
        self.picam.configure(
            self.picam.create_video_configuration(
                main={"size": self.size, "format": "RGB888"}
            )
        )
        self.lock = threading.Lock()
        self.enabled = False
        self.start()

    def start(self):
        with self.lock:
            if self.enabled:
                return True
            self.picam.start()
            time.sleep(1.5)  # let auto-exposure and white balance settle
            self.enabled = True
            return True

    def stop(self):
        with self.lock:
            if not self.enabled:
                return True
            self.picam.stop()
            self.enabled = False
            return True

    def set_enabled(self, enabled):
        try:
            return self.start() if enabled else self.stop()
        except Exception as exc:  # noqa: BLE001 - a wedged sensor must not take
            # the whole robot down with it
            print(f"[camera] could not {'start' if enabled else 'stop'}: {exc}", flush=True)
            return self.enabled

    def capture(self):
        """One BGR frame, or None if the camera is off or hiccuped."""
        if not self.enabled:
            return None
        try:
            return self.picam.capture_array()
        except Exception:  # noqa: BLE001 - e.g. stopped mid-capture by a toggle
            return None

    def close(self):
        try:
            self.picam.close()
        except Exception:  # noqa: BLE001
            pass


@driver("camera", "picamera2")
def _camera(context):
    return Camera()
