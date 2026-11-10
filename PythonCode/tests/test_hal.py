#!/usr/bin/env python3
"""Checks that the hardware abstraction is actually an abstraction.

The claim being tested is the one the layer exists to make: swapping a part
for a different part that does the same job touches that part and nothing
else. So these do not check that the ultrasonic works -- that needs an
ultrasonic -- they check that a new driver can be written, registered and
chosen without any other file knowing, and that a half-written one is
rejected at the point it is written rather than at the point the car needs
it.

Run directly: python3 tests/test_hal.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import hal
from hal.base import (
    KINDS, Context, Device, FloorSensor, PanMount, PowerMonitor,
    ProximitySensor, RangeSensor, build, catalogue, driver,
)

PASSES = []
FAILS = []


def check(label, condition):
    (PASSES if condition else FAILS).append(label)
    print("  %-6s %s" % ("PASS" if condition else "FAIL", label))


def section(title):
    print("\n" + title)
    print("-" * len(title))


section("the contracts are real contracts")


class Incomplete(RangeSensor):
    """Forgot read_cm."""


try:
    Incomplete()
    refused = False
except TypeError:
    refused = True
check("a driver missing a required method cannot be built", refused)


class Minimal(RangeSensor):
    label = "a tape measure"

    def read_cm(self):
        return 42.0


sensor = Minimal()
check("one that implements it can", sensor.read_cm() == 42.0)
check("and gets the default smoothing for free",
      sensor.read_smoothed_cm() == 42.0)
check("and a status without writing one", sensor.status()["kind"] == "range")
check("and a close that does nothing rather than crashing",
      sensor.close() is None)

# The three-state reading is the contract most worth stating out loud: a
# timeout and a silent sensor are different answers.
class Quiet(RangeSensor):
    def read_cm(self):
        return None


class NothingInRange(RangeSensor):
    def read_cm(self):
        return -1.0


check("silence and nothing-in-range stay distinguishable",
      Quiet().read_cm() is None and NothingInRange().read_cm() < 0)


section("every real driver honours its kind")

EXPECTED = {
    "range": RangeSensor,
    "proximity": ProximitySensor,
    "floor": FloorSensor,
    "pan": PanMount,
    "power": PowerMonitor,
}
from hal import sensors as sensor_module          # noqa: E402
from hal.drivetrain import Drivetrain             # noqa: E402

check("the ultrasonic on the Pi is a RangeSensor",
      issubclass(sensor_module.RangeFinder, RangeSensor))
check("so is the one over serial",
      issubclass(sensor_module.SerialRangeFinder, RangeSensor))
check("both proximity arrays are ProximitySensors",
      issubclass(sensor_module.SerialProximity, ProximitySensor)
      and issubclass(sensor_module.ProximityArray, ProximitySensor))
check("the servo is a PanMount", issubclass(sensor_module.PanServo, PanMount))
check("the floor array is a FloorSensor",
      issubclass(sensor_module.FloorArray, FloorSensor))
check("a chassis is a Device", issubclass(Drivetrain, Device))

listed = catalogue()
check("every kind is listed", set(listed) == set(KINDS))
check("and the roles this car fills have drivers",
      all(listed[k] for k in ("range", "proximity", "pan", "camera", "power")))
check("each driver names itself",
      all(d["name"] for entries in listed.values() for d in entries))


section("choosing between drivers is data, not a branch")

built = []


@driver("range", "test-cheap", priority=1)
def _cheap(context):
    built.append("cheap")
    return Minimal()


@driver("range", "test-better", priority=5)
def _better(context):
    built.append("better")
    sensor = Minimal()
    sensor.label = "a better tape measure"
    return sensor


empty = Context()
device = build("range", empty)
check("the higher priority wins", device.label == "a better tape measure")

built.clear()
device = build("range", empty, prefer="test-cheap")
check("unless something asks for the other by name",
      device.label == "a tape measure")

# This is the swap the whole layer exists for: a new driver, nothing else
# edited, and it takes over.
@driver("range", "test-newest", priority=99)
def _newest(context):
    sensor = Minimal()
    sensor.label = "a laser"
    return sensor


check("a driver added later takes over with no other change",
      build("range", empty).label == "a laser")


section("a driver may decline, and that is not a crash")


# Everything below uses the floor kind, whose only real driver declines
# unless somebody has said the sensors are wired. Nothing here touches a
# camera or a GPIO pin: a test that reaches for the hardware is a test that
# fails on the desk and, worse, one that fights the car while it is running.
@driver("floor", "test-needs-board", priority=50, needs_link=True)
def _needs_board(context):
    return Minimal()


with_board = Context(link=object())
check("one that needs a board is skipped when there is none",
      build("floor", Context()) is None)
check("and used when there is",
      build("floor", with_board).label == "a tape measure")


@driver("floor", "test-absent", priority=80, needs_link=True)
def _absent(context):
    return None             # the hardware is simply not there


check("one that finds nothing falls through to the next",
      build("floor", with_board).label == "a tape measure")


@driver("floor", "test-explodes", priority=90, needs_link=True)
def _explodes(context):
    raise RuntimeError("no such bus")


problems = []
device = build("floor", with_board,
               on_error=lambda name, exc: problems.append((name, str(exc))))
check("one that throws says why", problems and problems[0][0] == "test-explodes")
check("rather than taking the robot down with it",
      device is not None and device.label == "a tape measure")

try:
    build("nonsense", empty)
    complained = False
except ValueError:
    complained = True
check("asking for a kind that does not exist is refused", complained)


section("the context hands over only what a driver may use")

context = Context(link="a link", calibration=None, preferences=None)
check("a missing calibration falls back", context.measured("anything", 7) == 7)
check("a missing preference falls back", context.chosen("anything", 8) == 8)


class Store:
    def __init__(self, **kw):
        self.values = kw

    def get(self, key):
        return self.values.get(key)


context = Context(calibration=Store(pan_trim=3.0),
                  preferences=Store(floor_fitted=True))
check("a measurement comes through", context.measured("pan_trim") == 3.0)
check("a choice comes through", context.chosen("floor_fitted") is True)
check("a stored None still falls back", context.measured("missing", 5) == 5)


print("\n" + "=" * 62)
# ---------------------------------------------------------------------
# Building the chassis from what was measured about it.
#
# Reported as "the Control panel is locked, nothing can be selected". The
# cause was one extra argument: `Calibration.get()` takes a key and
# nothing else -- it already falls back to the declared default, which is
# why it exists -- and it was called with a default as well. The
# drivetrain then failed to build, and because the serial link belongs to
# the drivetrain, every device on the ESP32 went with it: the rangefinder
# quietly fell back to the Pi's own pins, the pan mount and the floor
# array vanished, and every behaviour that needs motors became
# unselectable.
#
# Nothing here could have caught it, because nothing built a drivetrain
# with a calibration. A fake chassis is enough: the fault was never in the
# motors, it was in reading the measurement.

from hal.drivetrain import DRIVETRAINS, build_drivetrain   # noqa: E402


class OneArgCalibration:
    """A calibration with the real one's signature, and nothing else.

    The point of the fake is the shape of `get`. Anything that calls it
    with a default -- which looks completely natural, and is how every
    dict in the language works -- raises here exactly as it does on the
    car, instead of at the next restart.
    """

    def __init__(self, **values):
        self.values = values
        self.asked = []

    def get(self, key):
        self.asked.append(key)
        return self.values.get(key)


built = {}


def _fake_chassis(heading_reversed=False):
    built["heading_reversed"] = heading_reversed
    return "fake chassis"


def _fake_chassis_without_a_gyro():
    built["plain"] = True
    return "plain chassis"


DRIVETRAINS["test-only"] = _fake_chassis
DRIVETRAINS["test-only-plain"] = _fake_chassis_without_a_gyro

calibration = OneArgCalibration(heading_reversed=True)
result = build_drivetrain(kind="test-only", calibration=calibration)
check("a chassis is built from the calibration without the call blowing up",
      result == "fake chassis")
check("and the measurement reaches it",
      built.get("heading_reversed") is True)
check("read with the one argument the calibration accepts",
      calibration.asked == ["heading_reversed"])

built.clear()
check("a chassis with no gyro to be backwards is built anyway",
      build_drivetrain(kind="test-only-plain",
                       calibration=calibration) == "plain chassis")
check("and is not handed a setting it has no use for",
      "heading_reversed" not in built)

built.clear()
check("and a car with no calibration at all still assembles",
      build_drivetrain(kind="test-only", calibration=None) == "fake chassis")
check("defaulting to the ordinary wiring rather than refusing",
      built.get("heading_reversed") is False)

# The first version of this guarded the call with `except TypeError`,
# meaning "this chassis takes no such argument". It also swallowed a
# TypeError raised anywhere inside a chassis that does take one, which
# sends the next person looking at the wrong module entirely.


def _chassis_that_is_broken_inside(heading_reversed=False):
    raise TypeError("something inside this chassis is wrong")


DRIVETRAINS["test-only-broken"] = _chassis_that_is_broken_inside
try:
    build_drivetrain(kind="test-only-broken", calibration=calibration)
    raised = False
except TypeError:
    raised = True
check("a fault inside a chassis is reported, not mistaken for an old one",
      raised)

for name in ("test-only", "test-only-plain", "test-only-broken"):
    del DRIVETRAINS[name]


if FAILS:
    print("FAILED %d of %d:" % (len(FAILS), len(FAILS) + len(PASSES)))
    for name in FAILS:
        print("  - " + name)
    sys.exit(1)
print("all %d hardware-layer checks passed" % len(PASSES))
