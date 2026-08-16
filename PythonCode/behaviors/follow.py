"""Follow whatever object you name.

The original `main_tennis_tracking.py` could chase exactly one thing: a
yellow-ish circle, found with a Hough transform and an HSV colour test. Because
detection is now a separate concern, the same control logic works on any of the
detector's 80 classes -- steer from how far off centre the target sits, drive
from how big it looks.

On a mecanum chassis this also gains an option the original could not express:
`orbit` strafes sideways while keeping the target centred, circling it instead
of only approaching head-on.

Two sensors, two questions. The camera says *which way* the target is, which
is what a camera is good at. The ultrasonic says *how far*, which is what a
rangefinder is good at and what a camera is worst at. Judging distance from
how big something looks works only while the whole of it is inside the frame,
and this camera sits 25cm off the floor -- follow a person and it sees legs,
which stop growing once they reach the top and bottom edges. The box then
says "still far away" at exactly the moment the car is about to arrive.

The apparent-size estimate is kept as a fallback rather than deleted, because
an ultrasonic does not hear everything. A small soft ball can return no echo
at all, and a follow behaviour that stops working when one sensor goes quiet
is worse than one that gets vaguer.
"""

import cv2

from control import PID
from hal import Velocity

from .base import Behavior

#: How far to sit from whatever is being followed, in centimetres. A real
#: measurement in a unit you can hold a ruler up to, rather than a fraction of
#: a frame that means something different for every target and every lens.
#: Kept well clear of the safety layer's stop distance: closer than that and
#: forward motion is refused, and the car would sit shuffling at a target it
#: has been told to approach and is not allowed to reach.
STANDOFF_CM = 45.0
DISTANCE_DEADBAND_CM = 8.0

#: Fraction of frame height the target should fill. Only used when there is no
#: distance reading to be had.
TARGET_FILL = 0.45
FILL_DEADBAND = 0.06
CENTRE_DEADBAND = 0.05

#: The rangefinder looks where the car points, not where the camera is looking,
#: so its reading is only about the target while the target is roughly ahead.
#: Off to one side, turn to face it first and do not pretend to know how far
#: away it is -- which also keeps the head forward, where the safety layer
#: needs it to allow any forward motion at all.
AHEAD_DEADBAND = 0.25

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
        self.standoff_cm = STANDOFF_CM
        self.turn_pid = PID(kp=1.6, kd=0.3, setpoint=0.0)
        #: Gain is per centimetre: 35cm too far asks for 0.7, which the clamp
        #: below brings to MAX_FORWARD.
        self.distance_pid = PID(kp=0.02, kd=0.005, setpoint=STANDOFF_CM)
        self.forward_pid = PID(kp=2.0, kd=0.2, setpoint=TARGET_FILL)
        self.lost_frames = 0
        self.box = None
        self.state = "waiting"

    def configure(self, **kwargs):
        if kwargs.get("target"):
            self.target = str(kwargs["target"]).strip().lower()
            self._reset()
        if "standoff_cm" in kwargs:
            try:
                self.standoff_cm = max(25.0, min(200.0, float(kwargs["standoff_cm"])))
                self.distance_pid.setpoint = self.standoff_cm
            except (TypeError, ValueError):
                pass
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
        self.distance_pid.reset()
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

        # -- drive: hold a real distance, and fall back to apparent size ----
        fill = target.height / height
        ahead = abs(offset) < AHEAD_DEADBAND
        measured = (
            world.range_cm
            if (world.range_cm is not None and world.range_cm > 0
                and world.range_forward and ahead)
            else None
        )

        vx = 0.0
        if measured is not None:
            # PID error is (setpoint - measured), which goes negative when the
            # target is further away than wanted -- and further away is exactly
            # when to drive forward. Flip it, as the turn above does.
            if abs(measured - self.standoff_cm) > DISTANCE_DEADBAND_CM:
                vx = -self.distance_pid.update(measured, world.dt)
            how = f"{measured:.0f}cm"
        elif not ahead:
            # Turning to face it. Refusing to guess at the distance here is not
            # caution for its own sake: the rangefinder is pointed down the
            # middle of the car, so while the target is off to one side, any
            # number it gives is about something else entirely.
            how = "turning to face"
        else:
            # No echo. The camera's estimate is worse, and worse is not nothing.
            if abs(TARGET_FILL - fill) > FILL_DEADBAND:
                vx = self.forward_pid.update(fill, world.dt)
            how = f"fill {fill:.2f} (no echo)"

        vx = max(-MAX_FORWARD, min(MAX_FORWARD, vx))
        vy = self.orbit * MAX_STRAFE

        self.state = (
            f"{self.target} {target.confidence:.2f} | off {offset:+.2f} | {how}"
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
        return {
            "target": self.target,
            "orbit": self.orbit,
            "standoff_cm": self.standoff_cm,
            "state": self.state,
        }
