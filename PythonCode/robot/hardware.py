"""Assembling the car: which devices exist, and in what order.

One module that knows how this machine is put together, so that the rest
of the robot can be about what the machine does. It builds nothing itself
-- the drivers in hal do that, and which driver wins is their business --
it decides what roles to ask for, in what order, and what to do when a
role cannot be filled.

Order matters in two places and nowhere else, which is the main reason
this is a sequence rather than a dict comprehension. The calibration has
to be read before the pan mount is built, because the mount aims itself at
its remembered angle the moment it is constructed. And the vision pipeline
needs the camera and the detector to exist first.

Missing hardware is ordinary. No camera ribbon, no board plugged in, a
library nobody installed: each returns None, every consumer already
handles None, and the only hard requirement is that it be said out loud.
A car that comes up without a rangefinder and mentions nothing looks
exactly like a car whose rangefinder is lying.
"""

import os

import hal
from hal.camera import Camera                   # noqa: F401 - registers it
from hal.drivetrain import build_drivetrain
from perception.clip import ClipRecorder
from perception.pipeline import VisionPipeline

#: Where recordings go: beside the code rather than in a home directory, so
#: what is filling the card is obvious to anyone who goes looking.
CLIPS = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "clips",
)


class Hardware:
    """Every physical thing the robot has, built once."""

    def __init__(self, calibration, settings, announce=print):
        self.announce = announce
        self.drivetrain = self._try("drivetrain", build_drivetrain)

        #: The serial link, if the chassis came with one. Sensors wired to
        #: that board report over it rather than off this one's pins, which
        #: is a fact about this car and not a branch any caller should see.
        self.link = getattr(self.drivetrain, "link", None)
        self.context = hal.Context(link=self.link, calibration=calibration,
                                   preferences=settings)

        self.rangefinder = self._device("range")
        self.pan = self._device("pan")
        self.proximity = self._device("proximity")
        self.floor = self._device("floor")

        self.detector = self._try("detector", _load_detector)
        self.camera = self._device("camera")
        self.vision = VisionPipeline(self.camera, self.detector) if self.camera else None
        self.clips = ClipRecorder(
            self.vision.frames, CLIPS,
            camera_off=lambda: not (self.camera and self.camera.enabled),
        ) if self.vision else None

        #: Costs no wiring: the Pi already measures the rail it runs on.
        self.supply = self._device("power")

    def _device(self, kind):
        """Fill a role from the registry, saying which driver filled it."""
        def grumble(name, exc):
            self.announce(f"[hw] {kind} driver {name}: {exc}")

        device = hal.build(kind, self.context, on_error=grumble)
        if device is None:
            self.announce(f"[hw] {kind}: none available")
        else:
            self.announce(f"[hw] {kind}: {device.label}")
        return device

    def _try(self, name, factory):
        """For the two things that are not devices in the registry sense."""
        try:
            made = factory()
        except Exception as exc:        # noqa: BLE001 - absent is normal
            self.announce(f"[hw] {name}: unavailable ({exc})")
            return None
        self.announce(f"[hw] {name}: ok")
        return made

    def status(self):
        """What filled each role, for the page and for a bug report."""
        roles = ("drivetrain", "rangefinder", "pan", "proximity", "floor",
                 "camera", "supply")
        return {role: getattr(getattr(self, role), "label", None)
                for role in roles}

    def close(self):
        for part in (self.clips, self.vision, self.camera, self.drivetrain):
            if part is None:
                continue
            try:
                part.close()
            except Exception as exc:    # noqa: BLE001 - shutting down
                self.announce(f"[hw] closing {part}: {exc}")


def _load_detector():
    """Imported late: it pulls in the whole inference stack, which is slow
    and which a car with no camera has no use for."""
    from perception.detector import Detector
    return Detector()
