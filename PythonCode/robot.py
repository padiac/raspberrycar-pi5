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

import os
import threading
import time

from behaviors import build_behaviors
from calibration import Calibration
from control import PathRecorder, SafetyLayer
from hal import Velocity, build_drivetrain
from hal.drivetrain import DRIVETRAINS
from hal.camera import Camera
from hal.gamepad import Gamepad
from hal.sensors import (
    FloorArray, PanServo, ProximityArray, RangeFinder, SerialRangeFinder,
)
from perception.clip import ClipRecorder
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
    #: Scales every behaviour's request on the way to the safety layer, so
    #: the speed control is a property of the robot rather than of manual
    #: driving. Without it the slider governed only the d-pad and every
    #: autonomous behaviour ran at whatever its constants happened to say,
    #: which is how patrol came to launch itself across the room at 0.8.
    #:
    #: Like the safety layer this only ever removes motion. Unlike it, this
    #: is the driver's choice rather than the machine's, so it lives here
    #: and not there -- safety refuses what is dangerous, the governor sets
    #: how briskly the rest happens.
    #: Nothing happening for this long and the camera goes off. It and the
    #: detector are the largest thing on the power budget by some way --
    #: measured at the wall, 0.96A idle against 1.42A with detection running
    #: -- so a car left parked with a battery in it is spending half its
    #: energy looking at a room nobody is asking about.
    idle_camera_after = 300.0

    speed_limit = 0.15
    #: Momentary, from a controller's trigger: lifts the limit for as long
    #: as it is held rather than changing what the slider says. Zero unless
    #: something is actively holding it there, so it cannot be left on.
    boost = 0.0

    def __init__(self):
        # Read before any hardware is built: the pan servo is aimed the
        # moment it is constructed, and aiming it at a remembered angle is
        # the whole point.
        self.calibration = Calibration()
        self.drivetrain = _optional("drivetrain", build_drivetrain)

        # When an ESP32 is driving, the sensors wired to it come back over the
        # same link rather than off this board's pins. Everything above still
        # just sees a WorldState and cannot tell the difference.
        self.link = getattr(self.drivetrain, "link", None)
        if self.link is not None:
            self.rangefinder = _optional(
                "rangefinder (via ESP32)", lambda: SerialRangeFinder(self.link)
            )
            self.pan = _optional(
                "pan servo",
                lambda: PanServo(self.link,
                                 trim=self.calibration.get("pan_trim")),
            )
            self.proximity = None
            self.floor = None
        else:
            self.pan = None
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
        # Clips live beside the code rather than in the home directory, so
        # the card's contents are obvious to anyone who goes looking.
        self.clips = ClipRecorder(
            self.vision.frames,
            os.path.join(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__))), "clips"),
            camera_off=lambda: not (self.camera and self.camera.enabled),
        ) if self.vision else None
        self.safety = SafetyLayer()
        self.recorder = PathRecorder()

        # A controller is not a behaviour, it is a driver -- so it goes in
        # through the same door as the on-screen pad and gets the same
        # governor and the same safety layer. Optional: no evdev, no pad, no
        # bother.
        self.gamepad = _optional(
            "gamepad",
            lambda: Gamepad(
                self.manual,
                actions={
                    "stop": self.emergency_stop,
                    "manual": lambda: self.select("manual"),
                    "cycle_behavior": self.cycle_behavior,
                    "toggle_record": self.toggle_record,
                    "pan_by": self.pan_by,
                    "boost": self.set_boost,
                    "pan_centre": self.pan_centre,
                },
                # A callable, not a value: the chassis can be swapped at
                # runtime and a bool read here would be the answer to a
                # question asked at start-up.
                holonomic=lambda: bool(self.drivetrain
                                       and self.drivetrain.dof >= 3),
            ),
        )

        self.behaviors = build_behaviors(self.recorder)
        self.behavior = self.behaviors["idle"]
        self._attach_overlay()

        #: The last distance measured while the head was actually pointing
        #: forward, and when. Kept because the head is on a servo and has
        #: reasons to look elsewhere -- and a reading taken a fraction of a
        #: second ago still describes where the car is, at the speeds this
        #: thing travels. Measured creeping toward a wall: 2 to 3 cm/s.
        self._forward_cm = None
        self._forward_at = 0.0

        self.last_activity = time.monotonic()
        #: True only when *this* turned the camera off. Someone who switched
        #: it off by hand has made a decision, and waking it up for them
        #: would be overruling it -- so only what was slept here is woken.
        self.camera_slept = False
        #: How many browsers are pulling the video right now. A phone that
        #: locks stops pulling on its own, so this is a decent proxy for
        #: whether anyone is actually looking.
        self.stream_clients = 0

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
            "pan": self.pan is not None,
            "gamepad": bool(self.gamepad and self.gamepad.device),
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
        self.boost = 0.0
        self.select("idle")
        with self.lock:
            self.allowed = Velocity.STOP
            if self.drivetrain is not None:
                # stop() rather than a zero velocity: a drivetrain that talks
                # to another board gets to send a distinct 'stop' rather than
                # a velocity that happens to be zero, so a panic never has to
                # survive the same code path as ordinary driving.
                self.drivetrain.stop()

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
        if velocity.moving:
            self.touch()
        if self.behavior.name != "manual":
            ok, message = self.select("manual")
            if not ok:
                return False, message
        self.behaviors["manual"].command(velocity)
        return True, "ok"

    # -- staying awake ------------------------------------------------------

    def touch(self):
        """Something happened. Called for anything that changes the robot."""
        self.last_activity = time.monotonic()
        if self.camera_slept:
            self.camera_slept = False
            if self.vision is not None:
                self.vision.set_camera(True)
                print("[idle] camera back on", flush=True)

    def _idle_check(self):
        """Turn the camera off if nothing has wanted it for a while.

        Three things count as wanting it, and all three have to be absent:
        somebody doing something, somebody watching the stream, and a behaviour
        that cannot work without it. That last one matters -- following an
        object while the camera sleeps would not be a saving, it would be a
        behaviour quietly broken by a power-saving feature it never heard of.
        """
        if self.vision is None or self.camera_slept:
            return
        if self.idle_camera_after <= 0:
            return                      # switched off; zero means never, not now
        if not (self.camera and self.camera.enabled):
            return                      # already off, by hand
        if self.stream_clients > 0:
            return
        if "camera" in self.behavior.requires or self.behavior.wants_detection:
            return
        if time.monotonic() - self.last_activity < self.idle_camera_after:
            return

        self.camera_slept = True
        self.vision.set_camera(False)
        print(f"[idle] nothing for {self.idle_camera_after:.0f}s, camera off",
              flush=True)

    # -- things a driver can ask for, from a pad or from the page -----------

    def set_boost(self, amount):
        """Momentary speed, held by whoever is holding it."""
        try:
            self.boost = max(0.0, min(1.0, float(amount)))
        except (TypeError, ValueError):
            self.boost = 0.0
        return self.boost

    def pan_by(self, degrees):
        """Nudge the sensor head. Returns where it ended up, or None."""
        if self.pan is None:
            return None
        return self.pan.look(self.pan.angle + float(degrees))

    def pan_centre(self):
        return self.pan.centre() if self.pan else None

    def set_pan_trim(self, trim):
        """Aim the head, and remember where forward turned out to be.

        The saving lives here rather than in PanServo because a HAL device
        should not know what a filesystem is; it knows how to point a servo.
        """
        if self.pan is None:
            return None
        self.pan.set_trim(trim)
        self.calibration.set("pan_trim", self.pan.trim)
        return self.pan.trim

    def cycle_behavior(self):
        """Next behaviour in the list, skipping any the hardware cannot do.

        Deliberately in the order the page groups them -- control first, then
        skills, then missions -- so pressing the button walks the same path the
        eye does down the screen, rather than through dictionary order.
        """
        order = [b for b in self.behavior_list() if not b["missing"]]
        if not order:
            return False, "nothing available"
        names = [b["name"] for b in order]
        try:
            index = names.index(self.behavior.name)
        except ValueError:
            index = -1
        return self.select(names[(index + 1) % len(names)])

    def toggle_record(self):
        if self.recorder.recording:
            self.recorder.disarm()
        else:
            self.recorder.arm()
        return self.recorder.status()

    def alert_level(self, world):
        """How urgent the obstacle ahead is, 0 to 1, for the pad to buzz with.

        Derived from the same numbers the safety layer clamps on, so the buzz
        and the braking cannot tell different stories. Nothing to say when the
        head is not looking forward: a reading from off to one side is not a
        statement about what the car is driving into.
        """
        # Only while actually driving into it. A distance on its own is not
        # news: parked in a room, something is nearly always inside the slow
        # zone, and a controller that buzzes continuously from the moment it
        # connects is not warning anyone of anything.
        if self.allowed.vx <= 0.0:
            return 0.0
        distance = world.range_cm
        if distance is None or distance <= 0 or not world.range_forward:
            return 0.0
        if distance <= self.safety.stop_cm:
            return 1.0
        if distance >= self.safety.slow_cm:
            return 0.0
        span = self.safety.slow_cm - self.safety.stop_cm
        return 0.8 * (self.safety.slow_cm - distance) / span

    # -- the loop -----------------------------------------------------------

    def _perceive(self, dt):
        frame, size, detections = (
            self.vision.snapshot() if self.vision else (None, (0, 0), [])
        )

        # A reading taken mid-swing describes where the head used to be, so
        # throw the average away rather than carrying it across.
        if self.pan is not None and self.rangefinder is not None and not self.pan.settled:
            self.rangefinder.forget()

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

        looking_forward = self.pan.looking_forward if self.pan else True
        if looking_forward and range_cm is not None:
            self._forward_cm = range_cm
            self._forward_at = time.monotonic()
        forward_age = (time.monotonic() - self._forward_at
                       if self._forward_at else None)

        return WorldState(
            dt=dt, frame=frame, frame_size=size, detections=detections,
            range_cm=range_cm, proximity=proximity, floor=floor,
            range_expected=self.rangefinder is not None,
            forward_cm=self._forward_cm,
            forward_age=forward_age,
            # A distance without a direction is half a measurement. With no
            # servo fitted the head cannot be anywhere but forward, so that
            # is the honest default rather than a convenient one.
            range_angle=self.pan.angle if self.pan else None,
            range_forward=self.pan.looking_forward if self.pan else True,
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
                    requested = self.behavior.update(self, world)
                    if self.behavior.governed:
                        requested = requested.scaled(
                            max(self.speed_limit, self.boost))
                    self.requested = requested
                    self._drive(self.safety.filter(requested, world))
                    self.behavior_error = None
                    # Record what reached the wheels rather than what was asked
                    # for, so a replay reproduces the drive that happened and
                    # not one the safety layer refused. Replaying is excluded,
                    # or a playback would record itself.
                    if self.behavior.name != "replay":
                        self.recorder.observe(self.allowed, dt)
                if self.gamepad is not None:
                    self.gamepad.alert(self.alert_level(world))
                self._idle_check()
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
            "speed_limit": self.speed_limit,
            "idle": {
                "after_s": self.idle_camera_after,
                "quiet_s": round(time.monotonic() - self.last_activity, 1),
                "camera_slept": self.camera_slept,
                "watchers": self.stream_clients,
            },
            "path": self.recorder.status(),
            "drivetrain": (
                {**self.drivetrain.status(), "kinds": sorted(DRIVETRAINS),
                 "kind": getattr(self.drivetrain, "kind", None)}
                if self.drivetrain else None
            ),
            "range_cm": None if world.range_cm is None else round(world.range_cm, 1),
            # What the safety layer actually judges on, which is not always
            # what the head happens to be pointing at this instant.
            "forward_cm": (None if world.forward_cm is None
                           else round(world.forward_cm, 1)),
            "forward_age": (None if world.forward_age is None
                            else round(world.forward_age, 2)),
            "pan": self.pan.status() if self.pan else None,
            "clip": self.clips.status() if self.clips else None,
            "gamepad": self.gamepad.status() if self.gamepad else None,
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
