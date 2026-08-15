"""Distance and proximity sensors.

Thin wrappers whose only job is to turn a pin reading into a plain number or
boolean with a documented meaning, so nothing above has to remember that these
particular sensors pull their output *low* when they see something.
"""

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
        self.average_cm = 0.8 * current + 0.2 * self.average_cm
        return self.average_cm


class SerialRangeFinder:
    """Distance as reported by the ESP32.

    The measurement itself happens over there because it is a microsecond
    pulse-width reading, and Linux cannot be trusted to time one: a scheduling
    hiccup of a millisecond turns into 17cm of error. All this does is read the
    number off the link.
    """

    def __init__(self, link):
        self.link = link
        self.average_cm = 0.0

    def read_cm(self):
        value = self.link.telemetry.range_cm
        return -1.0 if value is None else value

    def read_smoothed_cm(self):
        current = self.read_cm()
        if current < 0:
            return self.average_cm      # keep the last good value on a miss
        self.average_cm = 0.8 * current + 0.2 * self.average_cm
        return self.average_cm


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
