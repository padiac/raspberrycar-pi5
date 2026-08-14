"""Follow whatever object you name.

The original `main_tennis_tracking.py` could chase exactly one thing: a
yellow-ish circle, found with a Hough transform and an HSV colour test. Because
detection is now a separate concern, the same control logic works on any of the
detector's 80 classes -- steer from how far off centre the target sits, drive
from how big it looks.

On a mecanum chassis this also gains an option the original could not express:
`orbit` strafes sideways while keeping the target centred, circling it instead
of only approaching head-on.
"""

import cv2

from control import PID
from hal import Velocity

from .base import Behavior

#: Fraction of frame height the target should fill. Larger means "come closer".
TARGET_FILL = 0.45
FILL_DEADBAND = 0.06
CENTRE_DEADBAND = 0.05

MAX_FORWARD = 0.6
MAX_TURN = 0.8
MAX_STRAFE = 0.5

#: Frames without the target before giving up, rather than driving on blind.
LOST_PATIENCE = 8


class FollowBehavior(Behavior):
    name = "follow"
    label = "Follow object"
    description = "Locks onto a chosen object, keeps it centred and at distance."
    requires = ("motors", "camera", "detector")
    wants_detection = True
    accepts_target = True

    def __init__(self, target="person"):
        self.target = target
        self.orbit = 0.0            # -1 .. 1, sideways circling (mecanum only)
        self.turn_pid = PID(kp=1.6, kd=0.3, setpoint=0.0)
        self.forward_pid = PID(kp=2.0, kd=0.2, setpoint=TARGET_FILL)
        self.lost_frames = 0
        self.box = None
        self.state = "waiting"

    def configure(self, **kwargs):
        if kwargs.get("target"):
            self.target = str(kwargs["target"]).strip().lower()
            self._reset()
        if "orbit" in kwargs:
            try:
                self.orbit = max(-1.0, min(1.0, float(kwargs["orbit"])))
            except (TypeError, ValueError):
                pass
        return self.status()

    def _reset(self):
        self.lost_frames = 0
        self.box = None
        self.turn_pid.reset()
        self.forward_pid.reset()

    def start(self, robot):
        self._reset()
        self.state = "waiting"

    def stop(self, robot):
        self.box = None

    def update(self, robot, world):
        width, height = world.frame_size
        if not width or not height:
            self.state = "no camera"
            return Velocity.STOP

        target = world.largest(self.target)
        if target is None:
            self.lost_frames += 1
            if self.lost_frames > LOST_PATIENCE:
                self.box = None
                self.state = f"no {self.target} in view"
            else:
                # Briefly missing: the detector drops a frame now and then, so
                # coast rather than stamping on the brakes.
                self.state = f"{self.target} lost ({self.lost_frames})"
            return Velocity.STOP

        self.lost_frames = 0
        self.box = target.box

        # -- turn: drive the horizontal offset to zero ----------------------
        offset = (target.centre_x - width / 2) / (width / 2)
        omega = 0.0
        if abs(offset) > CENTRE_DEADBAND:
            # PID error is (setpoint - measured) = -offset, so a target to the
            # right gives a negative output; flip it back to "turn right".
            omega = -self.turn_pid.update(offset, world.dt)
            omega = max(-MAX_TURN, min(MAX_TURN, omega))

        # -- drive: hold the target at a constant apparent size -------------
        fill = target.height / height
        vx = 0.0
        if abs(TARGET_FILL - fill) > FILL_DEADBAND:
            vx = self.forward_pid.update(fill, world.dt)
            vx = max(-MAX_FORWARD, min(MAX_FORWARD, vx))

        vy = self.orbit * MAX_STRAFE

        self.state = (
            f"{self.target} {target.confidence:.2f} | off {offset:+.2f} | fill {fill:.2f}"
        )
        return Velocity(vx=vx, vy=vy, omega=omega)

    def overlay(self, frame):
        if self.box is None:
            return frame
        x1, y1, x2, y2 = self.box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
        height, width = frame.shape[:2]
        centre_x = (x1 + x2) // 2
        # The target line against the frame centre line makes the steering
        # error visible at a glance.
        cv2.line(frame, (centre_x, 0), (centre_x, height), (0, 0, 255), 1)
        cv2.line(frame, (width // 2, 0), (width // 2, height), (255, 255, 0), 1)
        return frame

    def status(self):
        return {"target": self.target, "orbit": self.orbit, "state": self.state}
