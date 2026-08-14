"""What a driving mode is.

A mode is a policy, not a program: given what the robot currently senses it
returns how hard to drive and how hard to turn. The robot owns the hardware
and the control loop, so modes can be swapped at runtime without anything
being opened or closed.
"""

from dataclasses import dataclass, field


@dataclass
class Perception:
    """Everything the robot knows right now, passed to the active mode."""

    frame: object = None            # BGR numpy array, or None if camera is off
    detections: list = field(default_factory=list)  # [(label, conf, (x1,y1,x2,y2), cls_id)]
    distance_cm: float = None       # ultrasonic, None if unavailable
    obstacle: list = None           # [left, right] IR, 0 = something there
    tracking: list = None           # [left, right] floor IR, 1 = dark
    frame_size: tuple = (0, 0)
    dt: float = 0.05                # seconds since the previous update


class DriveMode:
    """Base class. Subclasses override update(), and optionally start/stop."""

    name = "base"
    label = "Base"
    description = ""
    # Hardware this mode can't work without; the robot refuses to select a
    # mode whose requirements aren't met, rather than failing silently.
    requires = ()
    # Set True to have the robot switch detection on while this mode runs.
    wants_detection = False

    def start(self, robot):
        """Called once when this mode becomes active."""

    def stop(self, robot):
        """Called once when this mode is replaced."""

    def update(self, robot, view: Perception):
        """Return (throttle, steer), each -1.0 ~ 1.0."""
        return 0.0, 0.0

    def status(self):
        """Extra per-mode fields for the telemetry endpoint."""
        return {}

    def configure(self, **kwargs):
        """Mode-specific settings from the web UI. Return a status dict."""
        return self.status()

    def overlay(self, frame):
        """Optional: draw debug graphics on the streamed frame."""
        return frame
