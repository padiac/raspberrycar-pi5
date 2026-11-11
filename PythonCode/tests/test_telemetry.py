"""What the page is told, with no car attached.

This file exists because of a bug it would have caught in a second and
nobody caught for a day. Splitting the robot into a package left
`DRIVETRAINS` unimported here; everything compiled, every suite passed,
and the car served five hundreds to the only endpoint the page actually
uses. CONVENTIONS.md has carried "nothing in the suite calls telemetry()"
as a known hole ever since.

The two things checked are the two things that break. It must not raise
-- every attribute it reaches for has to exist, which is what the missing
import broke -- and the result must survive being turned into JSON,
because the web layer does that and a value that cannot be is a fault
that only appears when somebody opens the page.

Both are checked twice: once on a car with everything fitted, and once on
a car with nothing fitted at all, since absent hardware is ordinary here
and every one of those `if robot.x else None` branches is a line nobody
has ever executed otherwise.
"""

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from control import PathRecorder, SafetyLayer             # noqa: E402
from control.turning import TurnTracker                    # noqa: E402
from hal import Velocity                                     # noqa: E402
from perception.polar import PolarMemory                     # noqa: E402
from perception.types import Detection, WorldState           # noqa: E402
from robot.telemetry import describe                         # noqa: E402

FAILURES = []


def check(name, got, want):
    ok = got == want
    print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    if not ok:
        print(f"        got  {got!r}")
        print(f"        want {want!r}")
        FAILURES.append(name)


def truthy(name, value):
    check(name, bool(value), True)


class FakeBehavior:
    name = "follow"
    label = "Follow object"
    category = "skill"
    accepts_target = True

    def status(self):
        return {"target": "ball", "state": "ball 0.80 | off +0.01 | 51cm"}


class FakeStatus:
    """Anything whose only job here is to answer status()."""

    def __init__(self, **fields):
        self._fields = fields

    def status(self, *args):
        return dict(self._fields)


class FakeProximity:
    fitted = True
    reporting = True


class FakeTelemetry:
    heading = 12.5
    bumped = False
    proximity = (False, True)


class FakeLink:
    telemetry = FakeTelemetry()

    def status(self):
        return {"connected": True, "port": "/dev/ttyUSB0", "reboots": 0}


class FakeVision:
    stream_fps = 28.4
    detect_fps = 19.2
    detect_enabled = True


class FakeCamera:
    enabled = True


class FakeRobot:
    """Everything describe() reaches for, and nothing else.

    Spelled out rather than mocked, so that a field added to telemetry
    without being added here fails loudly here instead of quietly on the
    car -- which is the entire point of the file.
    """

    def __init__(self, equipped=True):
        self.behavior = FakeBehavior()
        self.behavior_error = None
        self.requested = Velocity(0.2, 0.0, -0.1)
        self.allowed = Velocity(0.2, 0.0, 0.0)
        self.safety = SafetyLayer()
        self.speed_limit = 0.4
        self.idle_camera_after = 300.0
        self.last_activity = time.monotonic()
        self.camera_slept = False
        self.stream_clients = 1
        self.recorder = PathRecorder()
        self.turning = TurnTracker()
        self.scan = PolarMemory()
        self.loop_hz = 17.6
        self.available = {"motors": equipped, "camera": equipped}

        self.world = WorldState(
            dt=0.05, frame_size=(1280, 720),
            detections=[Detection("ball", 0.81, (10, 20, 110, 120), 200)],
            range_cm=51.3, range_expected=equipped)

        if equipped:
            self.supply = FakeStatus(volts=5.01, sagging=False)
            self.drivetrain = FakeStatus(label="mecanum via ESP32", dof=3)
            self.drivetrain.kind = "serial"
            self.pan = FakeStatus(angle=0.0, settled=True)
            self.clips = FakeStatus(recording=False)
            self.gamepad = FakeStatus(connected=False)
            self.proximity = FakeProximity()
            self.link = FakeLink()
            self.vision = FakeVision()
            self.camera = FakeCamera()
            self.detector = FakeStatus(backend="ncnn", confidence=0.25)
        else:
            self.supply = None
            self.drivetrain = None
            self.pan = None
            self.clips = None
            self.gamepad = None
            self.proximity = None
            self.link = None
            self.vision = None
            self.camera = None
            self.detector = None

    def peer_list(self):
        return [{"address": "192.168.0.11", "quiet_s": 0.2}]


print("a car with everything fitted")

full = describe(FakeRobot(equipped=True))
truthy("telemetry can be built at all", isinstance(full, dict))
check("it names the running behaviour", full["behavior"], "follow")
check("and reports what reached the wheels, not only what was asked",
      (full["requested"]["omega"], full["allowed"]["omega"]), (-0.1, 0.0))

# The web layer hands this straight to jsonify. A value that cannot be
# serialised is a fault that appears only when somebody opens the page,
# which is the worst time to find out.
try:
    text = json.dumps(full)
    serialisable = True
except (TypeError, ValueError) as exc:
    serialisable = False
    print("        %s" % exc)
truthy("and the whole of it survives being turned into JSON", serialisable)

check("the drivetrain lists the kinds this car could be",
      isinstance(full["drivetrain"]["kinds"], list), True)
truthy("which is the import that was once missing here",
       len(full["drivetrain"]["kinds"]) > 0)

check("boxes go out with the labels, so the page can hit-test a tap",
      full["objects"][0]["box"], [10, 20, 110, 120])
check("and the frame they are measured in", full["frame_size"], [1280, 720])


print()
print("a car with nothing fitted")

# Absent hardware is ordinary in this project: no board, no camera, a
# library nobody installed. Every optional branch in describe() is a line
# that otherwise never runs anywhere.
bare = describe(FakeRobot(equipped=False))
truthy("telemetry can still be built", isinstance(bare, dict))
for field in ("supply", "drivetrain", "pan", "clip", "gamepad", "imu",
              "ir", "link", "detector"):
    check("  %s is absent rather than invented" % field, bare[field], None)
check("the camera is off rather than missing", bare["camera_enabled"], False)
check("and nothing is streaming", bare["stream_fps"], 0.0)

try:
    json.dumps(bare)
    serialisable = True
except (TypeError, ValueError):
    serialisable = False
truthy("and this survives JSON too", serialisable)


print()
print("the three states of a range reading survive the trip")

# Folding "nothing came back" into "nothing is there" has cost this
# project more than any other single mistake, and telemetry is the last
# place it could happen without anyone noticing.
for reading, expected, description in (
        (51.3, 51.3, "a measurement arrives as a measurement"),
        (-1.0, -1.0, "a timed-out ping stays negative, not zero"),
        (None, None, "a silent sensor stays None, not a distance")):
    robot = FakeRobot()
    robot.world = WorldState(dt=0.05, frame_size=(640, 360),
                             range_cm=reading, range_expected=True)
    check("  " + description, describe(robot)["range_cm"], expected)

print()
if FAILURES:
    print(f"{len(FAILURES)} FAILURE(S): {FAILURES}")
    sys.exit(1)
print("all telemetry checks passed")
