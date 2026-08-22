"""Distance and proximity sensors.

Thin wrappers whose only job is to turn a pin reading into a plain number or
boolean with a documented meaning, so nothing above has to remember that these
particular sensors pull their output *low* when they see something.
"""

import time
from dataclasses import dataclass

from gpiozero import DigitalInputDevice, DistanceSensor

from . import gpio  # noqa: F401 - selects the right gpiochip on import


@dataclass(frozen=True)
class Proximity:
    """Side-looking obstacle sensors. True means something is there."""

    left: bool = False
    right: bool = False

    @property
    def any(self):
        return self.left or self.right

    def as_dict(self):
        return {"left": self.left, "right": self.right}


@dataclass(frozen=True)
class Floor:
    """Downward line-tracking sensors. True means dark (over the line)."""

    left: bool = False
    right: bool = False

    def as_dict(self):
        return {"left": self.left, "right": self.right}


class RangeFinder:
    """HC-SR04 ultrasonic, reported in centimetres."""

    # BCM pins, inherited from upstream.
    TRIGGER = 20
    ECHO = 21
    MAX_METRES = 4.0        # the sensor's real range; gpiozero defaults to 1 m
                            # and would clip valid longer readings

    def __init__(self):
        # partial=True returns as soon as a reading exists instead of blocking
        # until gpiozero's smoothing queue fills, which otherwise hangs when
        # nothing is wired to the pins yet.
        self.sensor = DistanceSensor(
            echo=self.ECHO, trigger=self.TRIGGER,
            max_distance=self.MAX_METRES, partial=True,
        )
        self.average_cm = 0.0

    def read_cm(self):
        return self.sensor.distance * 100.0

    def read_smoothed_cm(self):
        """Moving average -- single ultrasonic reads are noisy."""
        current = self.read_cm()
        if self.average_cm is None:
            self.average_cm = current      # do not drag the first reading down
        else:
            self.average_cm = 0.8 * current + 0.2 * self.average_cm
        return self.average_cm


#: Nothing but "no echo", from every direction, for this long. A room does
#: not do that: turn a working sensor through a full circle in a house and
#: something comes back. What does do it is the sensor losing power, and the
#: way this car is wired that is what happens first as the battery goes down
#: -- the motors take 7.4V directly and merely slow, while the ultrasonic runs
#: off a buck that drops out of regulation around 6.5V in and stops answering.
#:
#: The reason this cannot be left as "clear" is the direction the mistake goes
#: in. A timed-out ping means open road, so a flat battery reads as permission
#: to drive, and the car gets blinder and more confident at the same time. It
#: was observed doing exactly that for 28 seconds.
#:
#: Generous, because a large empty room is a real thing and stopping is a real
#: cost. Eight seconds of hearing nothing at all is not that room.
DEAF_AFTER_S = 8.0


class SerialRangeFinder:
    """Distance as reported by the ESP32.

    The measurement itself happens over there because it is a microsecond
    pulse-width reading, and Linux cannot be trusted to time one: a scheduling
    hiccup of a millisecond turns into 17cm of error. All this does is read the
    number off the link.
    """

    def __init__(self, link):
        self.link = link
        self.average_cm = None
        #: When something last came back. Starts now, so a car switched on
        #: in the middle of a hall gets its grace period like anyone else.
        self.last_echo = time.monotonic()
        self.deaf = False

    def read_cm(self):
        """Distance in cm, negative for nothing within range, None for no idea.

        Three outcomes, and conflating any two of them is how a safety layer
        ends up doing nothing at the moment it is needed:

          a number    something is that far away
          negative    the pulse timed out, so nothing is within about 4m. This
                      is open road, and an honest answer.
          None        the board has stopped talking. Not an answer at all.

        Telemetry keeps its last value forever once the link goes quiet, so
        `fresh` is the only thing that can tell the difference between a live
        reading and a photograph of one.
        """
        if not self.link.telemetry.fresh:
            return None
        # Passed through as it stands, negative included. Deciding what a
        # negative means is the safety layer's business, not this one's.
        return self.link.telemetry.range_cm

    def forget(self):
        """Drop the running average, because it is about somewhere else now.

        The head can move. An average that survives a swing is a blend of two
        directions, and the blend looks like a perfectly ordinary distance.
        """
        self.average_cm = None

    def read_smoothed_cm(self):
        current = self.read_cm()
        if current is None:
            # Deliberately not the last good value. Holding one would let the
            # safety layer keep clamping against a number nothing is measuring
            # any more, and it would look exactly like a working sensor.
            self.average_cm = None
            return None
        if current < 0:
            self.average_cm = None      # nothing to average towards
            if time.monotonic() - self.last_echo > DEAF_AFTER_S:
                if not self.deaf:
                    self.deaf = True
                    print("[range] nothing has come back for "
                          f"{DEAF_AFTER_S:.0f}s; treating the sensor as "
                          "unreliable rather than the road as clear",
                          flush=True)
                return None             # unknown, not clear
            return current
        self.last_echo = time.monotonic()
        if self.deaf:
            self.deaf = False
            print("[range] hearing again", flush=True)
        if self.average_cm is None:
            self.average_cm = current
        else:
            self.average_cm = 0.8 * current + 0.2 * self.average_cm
        return self.average_cm


#: Servo degrees nominally straight ahead. The trim corrects the rest: a servo
#: horn only fits on its spline every few degrees, so the angle that actually
#: points down the middle of the car is a property of how it was assembled and
#: cannot be assumed.
CENTRE_DEG = 90
#: How far either side of forward the head may look. Beyond a right angle
#: the head is looking backwards down the side of the car, which tells you
#: nothing about where it is going.
PAN_LIMIT = 90
#: A hobby servo has no feedback, so the only honest way to know it has
#: arrived is to wait. These are generous; a wrong reading is worse than a
#: slow one.
SETTLE_PER_DEG = 0.004
SETTLE_MIN_S = 0.15
#: Within this of forward, a reading counts as a forward clearance.
FORWARD_TOLERANCE_DEG = 12


class PanServo:
    """Where the ultrasonic is pointing.

    The car has one distance sensor and it is on a servo, so "how far is the
    obstacle" is only half a question -- the other half is which way the head
    was facing at the time. Nothing used to ask: the firmware centred the servo
    once at boot and no one on this side ever touched it again, so the reading
    was whatever direction the head happened to have been left in. A steady
    3.3m while the car drove into a wall is exactly what that looks like.

    Angles here are relative to straight ahead, positive to the right, which is
    the same sense as vy and omega. The servo's own numbering is an
    implementation detail and stays inside this class.
    """

    #: Flip to -1 if the head turns the opposite way to the commanded sign.
    #: Which way a servo runs depends on how the linkage was put together, and
    #: this project has already been caught twice by an assumed sign.
    DIRECTION = 1

    def __init__(self, link, trim=0.0):
        self.link = link
        self.trim = float(trim)
        self.angle = 0.0
        self._arrived = 0.0
        self.look(0)

    def look(self, angle):
        """Aim the head. Returns the angle actually taken up."""
        angle = max(-PAN_LIMIT, min(PAN_LIMIT, float(angle)))
        travel = abs(angle - self.angle)
        self.angle = angle
        self._arrived = time.monotonic() + max(SETTLE_MIN_S, travel * SETTLE_PER_DEG)
        self.link.send_servo(CENTRE_DEG + self.trim + self.DIRECTION * angle)
        return self.angle

    def centre(self):
        return self.look(0)

    def set_trim(self, trim):
        self.trim = max(-40.0, min(40.0, float(trim)))
        return self.look(self.angle)      # re-aim so the change is visible

    @property
    def settled(self):
        """False while the head is still swinging, so readings are not trusted."""
        return time.monotonic() >= self._arrived

    @property
    def looking_forward(self):
        return self.settled and abs(self.angle) <= FORWARD_TOLERANCE_DEG

    def status(self):
        return {
            "angle": round(self.angle, 1),
            "trim": round(self.trim, 1),
            "settled": self.settled,
            "forward": self.looking_forward,
        }


class SerialProximity:
    """The side infrared, as reported by the ESP32.

    Reads nothing itself: the pins are on the other board, for the same reason
    the ultrasonic is. What this adds is the distinction between a sensor that
    is not fitted and a sensor that sees nothing -- a distinction this project
    has had to learn four separate times, always by watching a robot act
    confidently on an absence.

    `fitted` starts False so a pin that nobody has wired yet, sitting at its
    pull-up, does not read as a clear road to the safety layer. Turn it on once
    the readings can be seen changing.
    """

    def __init__(self, link, fitted=False):
        self.link = link
        self.fitted = fitted

    @property
    def reporting(self):
        """True if the firmware is sending the fields at all."""
        return self.link.telemetry.proximity is not None

    def read(self):
        if not self.fitted or not self.link.telemetry.fresh:
            return None
        pair = self.link.telemetry.proximity
        if pair is None:
            return None
        return Proximity(left=pair[0], right=pair[1])


class ProximityArray:
    """The two side-facing infrared obstacle sensors."""

    # BCM pins, inherited from upstream.
    LEFT = 7
    RIGHT = 8

    def __init__(self):
        self.left = DigitalInputDevice(self.LEFT, pull_up=False)
        self.right = DigitalInputDevice(self.RIGHT, pull_up=False)

    def read(self):
        # These modules pull the line LOW when something is in front, so an
        # obstacle is `value == 0`. Inverting here keeps that quirk out of
        # every caller.
        return Proximity(left=not self.left.value, right=not self.right.value)


class FloorArray:
    """The downward-facing line-tracking sensors.

    Only two of these are wired, which is one short of being useful: two
    sensors can tell you *which* side you have drifted to but not by how much,
    so a controller can only swing hard one way or the other. Three or more
    give a proportional error worth feeding to a PID.
    """

    # BCM pins, inherited from upstream.
    LEFT = 16
    RIGHT = 12

    def __init__(self):
        self.left = DigitalInputDevice(self.LEFT, pull_up=False)
        self.right = DigitalInputDevice(self.RIGHT, pull_up=False)

    def read(self):
        # 1 means dark on these modules.
        return Floor(left=bool(self.left.value), right=bool(self.right.value))
