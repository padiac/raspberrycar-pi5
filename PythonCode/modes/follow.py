"""Follow any object the detector can see.

The original main_tennis_tracking.py could only chase one thing: a yellow-ish
circle, found with a Hough transform plus an HSV colour test. This does the
same job against any of the detector's 80 classes -- steer from how far the
target sits off centre, throttle from how big it looks.
"""

import cv2

from PID_controller import PID

from .base import DriveMode

# Fraction of the frame height the target should occupy. Bigger than this and
# we're too close and back off; smaller and we chase.
TARGET_HEIGHT = 0.45
HEIGHT_DEADBAND = 0.08
STEER_DEADBAND = 0.06

MAX_THROTTLE = 0.6
MAX_STEER = 0.8

# How many consecutive frames without the target before giving up and
# stopping, rather than driving on blind.
LOST_PATIENCE = 8


class FollowMode(DriveMode):
    name = "follow"
    label = "Follow object"
    description = "Locks onto a chosen object and keeps it centred and at distance."
    requires = ("motors", "camera", "detector")
    wants_detection = True

    def __init__(self, target="person"):
        self.target = target
        # Steering: drive the horizontal offset to zero.
        self.steer_pid = PID(kp=1.6, ki=0.0, kd=0.35)
        self.steer_pid.SetExpectedOutput(0.0)
        self.lost_frames = 0
        self.last_box = None
        self.state = "waiting"

    def configure(self, **kwargs):
        target = kwargs.get("target")
        if target:
            self.target = str(target).strip().lower()
            self.reset()
        return self.status()

    def reset(self):
        self.lost_frames = 0
        self.last_box = None
        self.steer_pid.last_error = 0
        self.steer_pid.accumulate_error = 0

    def start(self, robot):
        self.reset()
        self.state = "waiting"

    def stop(self, robot):
        self.last_box = None

    def _pick(self, detections):
        """Largest matching box -- the nearest instance of what we're after."""
        best, best_area = None, 0
        for label, conf, box, _cls in detections:
            if label.lower() != self.target:
                continue
            x1, y1, x2, y2 = box
            area = (x2 - x1) * (y2 - y1)
            if area > best_area:
                best, best_area = (label, conf, box), area
        return best

    def update(self, robot, view):
        width, height = view.frame_size
        if not width or not height:
            return 0.0, 0.0

        match = self._pick(view.detections)
        if match is None:
            self.lost_frames += 1
            if self.lost_frames > LOST_PATIENCE:
                self.last_box = None
                self.state = f"no {self.target} in view"
                return 0.0, 0.0
            # Briefly missing: coast rather than stamping on the brakes, the
            # detector drops a frame now and then.
            self.state = f"{self.target} lost ({self.lost_frames})"
            return 0.0, 0.0

        self.lost_frames = 0
        _label, conf, (x1, y1, x2, y2) = match
        self.last_box = (x1, y1, x2, y2)

        # -- steering: how far off centre, as -1 (left) .. +1 (right) --------
        centre_x = (x1 + x2) / 2
        offset = (centre_x - width / 2) / (width / 2)
        steer = 0.0
        if abs(offset) > STEER_DEADBAND:
            # PID error is (expected - current) = -offset, so a target to the
            # right gives a negative output; flip it back into "turn right".
            steer = -self.steer_pid.UpdateOutput(offset)
            steer = max(-MAX_STEER, min(MAX_STEER, steer))

        # -- throttle: from how much of the frame the target fills -----------
        fill = (y2 - y1) / height
        error = TARGET_HEIGHT - fill
        throttle = 0.0
        if abs(error) > HEIGHT_DEADBAND:
            throttle = max(-MAX_THROTTLE, min(MAX_THROTTLE, error * 2.0))

        self.state = (
            f"{self.target} {conf:.2f} | offset {offset:+.2f} | fill {fill:.2f}"
        )
        return throttle, steer

    def overlay(self, frame):
        if self.last_box is None:
            return frame
        x1, y1, x2, y2 = self.last_box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
        cx = (x1 + x2) // 2
        h = frame.shape[0]
        # Vertical line through the target and through the frame centre, so
        # the steering error is visible at a glance.
        cv2.line(frame, (cx, 0), (cx, h), (0, 0, 255), 1)
        cv2.line(frame, (frame.shape[1] // 2, 0), (frame.shape[1] // 2, h), (255, 255, 0), 1)
        return frame

    def status(self):
        return {"target": self.target, "state": self.state}
