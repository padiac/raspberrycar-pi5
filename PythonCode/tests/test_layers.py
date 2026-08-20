"""Checks for the parts that are pure logic: kinematics and the safety layer.

Neither needs a motor, a sensor or a camera, which is the point -- the layering
is what makes them testable at all.
"""

import sys
from pathlib import Path

# Find the robot from wherever this is run. Relying on the working directory
# instead -- which is what "sys.path.insert(0, '.')" amounts to -- works until
# somebody runs the file from anywhere else, and then fails with an ImportError
# about `hal` that says nothing about the actual problem.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from hal.drivetrain import DifferentialDrive, MecanumDrive, Velocity, _normalise, SerialDrivetrain
from hal.sensors import Proximity
from control.safety import SafetyLayer
from perception.types import Detection, WorldState

FAILURES = []


def check(name, got, want, tol=1e-6):
    ok = all(abs(g - w) < tol for g, w in zip(got, want)) and len(got) == len(want)
    print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    if not ok:
        print(f"        got  {[round(g, 3) for g in got]}")
        print(f"        want {[round(w, 3) for w in want]}")
        FAILURES.append(name)


def check_eq(name, got, want):
    ok = got == want
    print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    if not ok:
        print(f"        got {got!r}  want {want!r}")
        FAILURES.append(name)


print("mecanum inverse kinematics  [front_left, front_right, rear_left, rear_right]")
# Build without touching GPIO by bypassing __init__.
mec = MecanumDrive.__new__(MecanumDrive)

diff_for_sign = DifferentialDrive.__new__(DifferentialDrive)
check("drive straight",        mec.wheel_speeds(Velocity(vx=1)),          [1, 1, 1, 1])
check("reverse",               mec.wheel_speeds(Velocity(vx=-1)),         [-1, -1, -1, -1])
# These four are the ones with a sign convention to get wrong, and they were
# wrong: the mixer had been written to the textbook form (ROS REP-103, y left
# and yaw counter-clockwise) while Velocity, the d-pad and the differential
# drive all mean vy right and omega right. The expectations below had been
# copied from the mixer rather than derived from the convention, so the test
# agreed with the car and both disagreed with the steering wheel. Derive them
# from what the driver asked for: strafing right means the wheels that push
# forward-right drive forward, and turning right means the left side outruns
# the right.
check("strafe right",          mec.wheel_speeds(Velocity(vy=1)),          [1, -1, -1, 1])
check("strafe left",           mec.wheel_speeds(Velocity(vy=-1)),         [-1, 1, 1, -1])
check("spin right",            mec.wheel_speeds(Velocity(omega=1)),       [1, -1, 1, -1])
check("spin left",             mec.wheel_speeds(Velocity(omega=-1)),      [-1, 1, -1, 1])
check("diagonal fwd+right",    mec.wheel_speeds(Velocity(vx=1, vy=1)),    [2, 0, 0, 2])
check("diagonal fwd+left",     mec.wheel_speeds(Velocity(vx=1, vy=-1)),   [0, 2, 2, 0])

# The serial chassis predicts the wheels locally until the board reports back,
# so a disagreement here shows up as the diagram flickering the wrong way.
check("serial mixer agrees with mecanum",
      SerialDrivetrain.wheel_speeds(None, Velocity(vx=0.3, vy=0.7, omega=-0.2)),
      mec.wheel_speeds(Velocity(vx=0.3, vy=0.7, omega=-0.2)))

# Turning right must move the left side forward relative to the right on both
# chassis. Stated once, in the language of the driver, so the two cannot drift
# apart again.
left, right = diff_for_sign.wheel_speeds(Velocity(omega=1))
check_eq("differential turns right the same way", left > right, True)
check("stopped",               mec.wheel_speeds(Velocity()),              [0, 0, 0, 0])

print("\nnormalisation keeps direction (this is why we do not clip per wheel)")
check("diagonal scaled to unit", _normalise(mec.wheel_speeds(Velocity(vx=1, vy=1))),
      [1, 0, 0, 1])
# Full throttle plus full turn: clipping each wheel would give [1,1] on a
# differential chassis and drive straight instead of turning.
diff = DifferentialDrive.__new__(DifferentialDrive)
check("differential fwd+turn keeps turning",
      _normalise(diff.wheel_speeds(Velocity(vx=1, omega=1))), [1, 0])

print("\ndifferential ignores strafe (2-DOF chassis has no sideways axis)")
check("strafe does nothing", diff.wheel_speeds(Velocity(vy=1)), [0, 0])

print("\nsafety layer")
safety = SafetyLayer(stop_cm=20, slow_cm=80)


def world(range_cm=None, prox=None):
    return WorldState(range_cm=range_cm, proximity=prox)


out = safety.filter(Velocity(vx=1.0), world(range_cm=200))
check_eq("open road: forward untouched", round(out.vx, 3), 1.0)

out = safety.filter(Velocity(vx=1.0), world(range_cm=10))
check_eq("wall at 10cm: forward blocked", out.vx, 0.0)
check_eq("  and it says why", safety.last.intervened, True)

out = safety.filter(Velocity(vx=1.0), world(range_cm=50))
ok = 0.0 < out.vx < 1.0
print(f"  {'PASS' if ok else 'FAIL'}  wall at 50cm: forward limited, not zeroed ({out.vx:.2f})")
if not ok:
    FAILURES.append("slow zone")

out = safety.filter(Velocity(vx=-1.0), world(range_cm=10))
check_eq("wall ahead: reversing still allowed", out.vx, -1.0)

out = safety.filter(Velocity(omega=1.0), world(range_cm=10))
check_eq("wall ahead: turning still allowed", out.omega, 1.0)

out = safety.filter(Velocity(vy=-1.0), world(prox=Proximity(left=True)))
check_eq("obstacle left: strafe left blocked", out.vy, 0.0)

out = safety.filter(Velocity(vy=1.0), world(prox=Proximity(left=True)))
check_eq("obstacle left: strafe right allowed", out.vy, 1.0)

out = safety.filter(Velocity(vx=1.0), world(prox=Proximity(left=True, right=True)))
check_eq("boxed in: forward blocked", out.vx, 0.0)

print("\nthe head may look away, but not for long")
# The head is on a servo and has reasons to look elsewhere -- a chair leg is a
# single ten-degree spike, invisible to a car that only ever stares ahead. So
# the layer accepts the last reading taken while forward, if it is recent.
# "Recent" is the whole of the safety argument: at the 2-3 cm/s this car
# approaches things, a second-old reading is wrong by about three centimetres
# against a stopping distance of twenty.
out = safety.filter(Velocity(vx=1.0), WorldState(
    range_cm=8.0, range_forward=False,          # head is off to one side
    forward_cm=100.0, forward_age=0.2,          # but it saw clear road just now
    range_expected=True))
check_eq("glancing aside: drives on the reading it just took", out.vx, 1.0)

out = safety.filter(Velocity(vx=1.0), WorldState(
    range_cm=100.0, range_forward=False,
    forward_cm=10.0, forward_age=0.2,           # and it saw a wall just now
    range_expected=True))
check_eq("  and still stops for what that reading said", out.vx, 0.0)

out = safety.filter(Velocity(vx=1.0), WorldState(
    range_cm=100.0, range_forward=False,
    forward_cm=100.0, forward_age=5.0,          # last looked forward ages ago
    range_expected=True))
check_eq("looked away too long: forward blocked", out.vx, 0.0)

out = safety.filter(Velocity(vx=-1.0), WorldState(
    range_cm=100.0, range_forward=False,
    forward_cm=100.0, forward_age=5.0,
    range_expected=True))
check_eq("  but reversing out of it is still allowed", out.vx, -1.0)


print("\na sensor that has gone quiet is a fault, not open road")
# Three states that used to look alike. A missed reading came back as the last
# good value, or as zero before there had been one, and zero fails the layer's
#  test -- so the limiter did nothing at the moment it mattered most.
out = safety.filter(Velocity(vx=1.0), WorldState(range_cm=None, range_expected=True))
check_eq("rangefinder silent: forward blocked", out.vx, 0.0)

out = safety.filter(Velocity(vx=-1.0), WorldState(range_cm=None, range_expected=True))
check_eq("  and reversing away from it still allowed", out.vx, -1.0)

# A timed-out pulse means nothing within about 4m. That is an answer, and the
# answer is that the road is clear.
out = safety.filter(Velocity(vx=1.0), WorldState(range_cm=-1.0, range_expected=True))
check_eq("nothing within range: forward allowed", out.vx, 1.0)

# No sensor fitted at all is different again, and must not immobilise the car.
out = safety.filter(Velocity(vx=1.0), WorldState(range_cm=None, range_expected=False))
check_eq("no rangefinder fitted: forward allowed", out.vx, 1.0)

safety.enabled = False
out = safety.filter(Velocity(vx=1.0), world(range_cm=5))
check_eq("disabled: nothing is clamped", out.vx, 1.0)
safety.enabled = True

print("\nsafety never invents motion (it may only remove)")
bad = []
for req in [Velocity(vx=1), Velocity(vy=1), Velocity(omega=1), Velocity(vx=-1),
            Velocity(vx=1, vy=1, omega=1), Velocity()]:
    for w in [world(5), world(50), world(300),
              world(5, Proximity(True, True)), world(300, Proximity(True, False))]:
        got = safety.filter(req, w)
        for axis in ("vx", "vy", "omega"):
            r, g = getattr(req, axis), getattr(got, axis)
            # Same sign or zero, and never larger in magnitude.
            if abs(g) > abs(r) + 1e-9 or (r == 0 and g != 0) or (r * g < 0):
                bad.append((req, w.range_cm, axis, r, g))
print(f"  {'PASS' if not bad else 'FAIL'}  checked all axes over 30 combinations")
if bad:
    FAILURES.append("safety added motion")
    for entry in bad[:5]:
        print("        ", entry)

print("\nworld state helpers")
w = WorldState(detections=[
    Detection("person", 0.9, (0, 0, 10, 10), 0),
    Detection("person", 0.8, (0, 0, 50, 50), 0),
    Detection("bottle", 0.7, (0, 0, 20, 20), 39),
])
check_eq("largest() picks the biggest match", w.largest("person").box, (0, 0, 50, 50))
check_eq("largest() is case-insensitive", w.largest("PERSON") is not None, True)
check_eq("largest() returns None when absent", w.largest("cat"), None)
check_eq("labels()", w.labels(), ["bottle", "person"])

print()
if FAILURES:
    print(f"{len(FAILURES)} FAILURE(S): {FAILURES}")
    raise SystemExit(1)
print("all checks passed")
