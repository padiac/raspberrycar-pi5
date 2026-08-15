"""The robot: wires the layers together and turns the crank.

Every tick, in order:

    perception  ->  behaviour  ->  safety  ->  drivetrain
    what is out    what I want   what I am    wheel speeds
    there          to do         allowed

Each arrow is the only way information crosses that boundary, which is what
makes the pieces swappable: a behaviour cannot reach a motor, the safety layer
cannot see the camera, and the drivetrain has no idea why it was asked to move.

Missing hardware is not fatal. Anything that fails to initialise is simply
absent, behaviours that need it cannot be selected, and the rest still runs --
which matters while the motor board is still in the post.
"""

import threading
import time

from behaviors import build_behaviors
from control import PathRecorder, SafetyLayer
from hal import Velocity, build_drivetrain
from hal.drivetrain import DRIVETRAINS
from hal.camera import Camera
from hal.sensors import FloorArray, ProximityArray, RangeFinder, SerialRangeFinder
from perception.pipeline import VisionPipeline
from perception.types import WorldState

CONTROL_HZ = 20.0


def _optional(name, factory):
    """Build a piece of hardware; report and return None if it is not there."""
    try:
        thing = factory()
        print(f"[hw] {name}: ok", flush=True)
        return thing
    except Exception as exc:  # noqa: BLE001 - absent hardware must not be fatal
        print(f"[hw] {name}: unavailable ({exc})", flush=True)
        return None


class Robot:
    def __init__(self):
        self.drivetrain = _optional("drivetrain", build_drivetrain)

        # When an ESP32 is driving, the sensors wired to it come back over the
        # same link rather than off this board's pins. Everything above still
        # just sees a WorldState and cannot tell the difference.
        self.link = getattr(self.drivetrain, "link", None)
        if self.link is not None:
            self.rangefinder = _optional(
                "rangefinder (via ESP32)", lambda: SerialRangeFinder(self.link)
            )
            self.proximity = None
            self.floor = None
        else:
            self.rangefinder = _optional("rangefinder", RangeFinder)
            self.proximity = _optional("proximity", ProximityArray)
            self.floor = _optional("floor sensors", FloorArray)
        self.detector = _optional(
            "detector", lambda: __import__("perception.detector", fromlist=["Detector"]).Detector()
        )
        self.camera = _optional("camera", Camera)

        self.vision = (
            VisionPipeline(self.camera, self.detector) if self.camera else None
        )
        self.safety = SafetyLayer()
        self.recorder = PathRecorder()

        self.behaviors = build_behaviors(self.recorder)
        self.behavior = self.behaviors["idle"]
        self._attach_overlay()

        self.requested = Velocity.STOP
        self.allowed = Velocity.STOP
        self.behavior_error = None
        self.world = WorldState()

        self.lock = threading.RLock()
        self.running = True
        threading.Thread(target=self._control_loop, daemon=True).start()

    # -- capabilities -------------------------------------------------------

    @property
    def available(self):
        return {
            "motors": self.drivetrain is not None,
            "camera": self.camera is not None and self.camera.enabled,
            "detector": self.detector is not None,
            "rangefinder": self.rangefinder is not None,
            "proximity": self.proximity is not None,
            "floor": self.floor is not None,
        }

    def missing_for(self, behavior):
        have = self.available
        return [need for need in behavior.requires if not have.get(need, False)]

    def behavior_list(self):
        return [
            {
                "name": b.name,
                "label": b.label,
                "description": b.description,
                "category": getattr(b, "category", "skill"),
                "accepts_target": getattr(b, "accepts_target", False),
                "requires": list(b.requires),
                "missing": self.missing_for(b),
                "active": b is self.behavior,
            }
            for b in self.behaviors.values()
        ]

    # -- switching ----------------------------------------------------------

    def _attach_overlay(self):
        if self.vision is not None:
            self.vision.overlay_hook = self.behavior.overlay

    def select(self, name):
        """Switch behaviour. Returns (ok, message)."""
        behavior = self.behaviors.get(name)
        if behavior is None:
            return False, f"unknown behaviour '{name}'"

        missing = self.missing_for(behavior)
        if missing:
            return False, f"{behavior.label} needs: {', '.join(missing)}"

        with self.lock:
            if behavior is self.behavior:
                return True, f"already in {behavior.label}"

            self.behavior.stop(self)
            self._drive(Velocity.STOP)      # never carry speed across a switch

            self.behavior = behavior
            self.behavior_error = None
            self._attach_overlay()
            if behavior.wants_detection and self.vision is not None:
                self.vision.set_detection(True)
            behavior.start(self)

        return True, f"switched to {behavior.label}"

    def emergency_stop(self):
        self.select("idle")
        with self.lock:
            self._drive(Velocity.STOP)

    def set_drivetrain(self, kind):
        """Swap the chassis model at runtime. Returns (ok, message).

        Useful before the four-motor board arrives: the mecanum kinematics can
        be exercised, and the wheel speeds watched, with only two motors
        actually wired. The pins have to be handed back before the new chassis
        can claim them, so a failure leaves nothing driving -- hence the
        attempt to put the old one back.
        """
        with self.lock:
            if self.drivetrain is not None:
                if kind == getattr(self.drivetrain, "kind", None):
                    return True, f"already {kind}"
                self._drive(Velocity.STOP)
                self.drivetrain.close()
                self.drivetrain = None

            try:
                self.drivetrain = build_drivetrain(kind)
                self.drivetrain.kind = kind
                return True, f"drivetrain: {self.drivetrain.label}"
            except Exception as exc:  # noqa: BLE001
                message = f"could not build '{kind}': {exc}"
                print(f"[hw] {message}", flush=True)
                try:
                    self.drivetrain = build_drivetrain()   # back to the default
                    self.drivetrain.kind = None
                except Exception:  # noqa: BLE001 - now there is simply no chassis
                    self.drivetrain = None
                return False, message

    def set_target(self, label):
        """Point whatever is running at `label`, or start following it.

        Called when someone taps an object in the video. If the current
        behaviour already chases a named thing -- follow, or a mission with a
        follow leaf -- just retarget it and carry on; otherwise following is
        the obvious thing they meant.
        """
        label = str(label).strip().lower()
        if not label:
            return False, "no label"

        with self.lock:
            if getattr(self.behavior, "accepts_target", False):
                self.behavior.configure(target=label)
                return True, f"{self.behavior.label} now tracking {label}"

        ok, message = self.select("follow")
        if not ok:
            return False, message
        self.behaviors["follow"].configure(target=label)
        return True, f"following {label}"

    def manual(self, velocity):
        """Feed the manual behaviour, selecting it so the controls just work."""
        if self.behavior.name != "manual":
            ok, message = self.select("manual")
            if not ok:
                return False, message
        self.behaviors["manual"].command(velocity)
        return True, "ok"

    # -- the loop -----------------------------------------------------------

    def _perceive(self, dt):
        frame, size, detections = (
            self.vision.snapshot() if self.vision else (None, (0, 0), [])
        )

        range_cm = None
        if self.rangefinder is not None:
            try:
                range_cm = self.rangefinder.read_smoothed_cm()
            except Exception:  # noqa: BLE001 - a flaky sensor must not stop the loop
                range_cm = None

        proximity = floor = None
        if self.proximity is not None:
            try:
                proximity = self.proximity.read()
            except Exception:  # noqa: BLE001
                pass
        if self.floor is not None:
            try:
                floor = self.floor.read()
            except Exception:  # noqa: BLE001
                pass

        return WorldState(
            dt=dt, frame=frame, frame_size=size, detections=detections,
            range_cm=range_cm, proximity=proximity, floor=floor,
        )

    def _drive(self, velocity):
        self.allowed = velocity
        if self.drivetrain is not None:
            self.drivetrain.apply(velocity)

    def _control_loop(self):
        period = 1.0 / CONTROL_HZ
        previous = time.monotonic()
        while self.running:
            time.sleep(period)
            now = time.monotonic()
            dt, previous = now - previous, now

            try:
                world = self._perceive(dt)
                with self.lock:
                    self.world = world
                    self.requested = self.behavior.update(self, world)
                    self._drive(self.safety.filter(self.requested, world))
                    self.behavior_error = None
                    # Record what reached the wheels rather than what was asked
                    # for, so a replay reproduces the drive that happened and
                    # not one the safety layer refused. Replaying is excluded,
                    # or a playback would record itself.
                    if self.behavior.name != "replay":
                        self.recorder.observe(self.allowed, dt)
            except Exception as exc:  # noqa: BLE001 - a bad behaviour must not run away
                with self.lock:
                    self.behavior_error = f"{type(exc).__name__}: {exc}"
                    self._drive(Velocity.STOP)
                print(f"[control] {self.behavior.name}: {exc}", flush=True)
                time.sleep(0.5)

    # -- telemetry ----------------------------------------------------------

    def telemetry(self):
        world = self.world
        return {
            "behavior": self.behavior.name,
            "behavior_label": self.behavior.label,
            "behavior_status": self.behavior.status(),
            "behavior_error": self.behavior_error,
            "behavior_category": getattr(self.behavior, "category", "skill"),
            "accepts_target": getattr(self.behavior, "accepts_target", False),
            "moving": self.allowed.moving,
            "requested": self.requested.as_dict(),
            "allowed": self.allowed.as_dict(),
            "safety": self.safety.status(),
            "path": self.recorder.status(),
            "drivetrain": (
                {**self.drivetrain.status(), "kinds": sorted(DRIVETRAINS),
                 "kind": getattr(self.drivetrain, "kind", None)}
                if self.drivetrain else None
            ),
            "range_cm": None if world.range_cm is None else round(world.range_cm, 1),
            "proximity": world.proximity.as_dict() if world.proximity else None,
            "floor": world.floor.as_dict() if world.floor else None,
            "stream_fps": round(self.vision.stream_fps, 1) if self.vision else 0.0,
            "detect_fps": round(self.vision.detect_fps, 1) if self.vision else 0.0,
            "detect_enabled": self.vision.detect_enabled if self.vision else False,
            "camera_enabled": bool(self.camera and self.camera.enabled),
            # Boxes go out too so the page can work out which object was
            # tapped without asking the robot to hit-test for it.
            "objects": [
                {**d.as_dict(), "box": list(d.box)} for d in world.detections
            ],
            "frame_size": list(world.frame_size),
            "detector": self.detector.status() if self.detector else None,
            "available": self.available,
        }

    def close(self):
        self.running = False
        try:
            self.emergency_stop()
        finally:
            if self.vision:
                self.vision.close()
            if self.camera:
                self.camera.close()
