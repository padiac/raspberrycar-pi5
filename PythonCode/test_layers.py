"""Checks for the parts that are pure logic: kinematics and the safety layer.

Neither needs a motor, a sensor or a camera, which is the point -- the layering
is what makes them testable at all.
"""

from hal.drivetrain import DifferentialDrive, MecanumDrive, Velocity, _normalise
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

check("drive straight",        mec.wheel_speeds(Velocity(vx=1)),          [1, 1, 1, 1])
check("reverse",               mec.wheel_speeds(Velocity(vx=-1)),         [-1, -1, -1, -1])
check("strafe right",          mec.wheel_speeds(Velocity(vy=1)),          [-1, 1, 1, -1])
check("strafe left",           mec.wheel_speeds(Velocity(vy=-1)),         [1, -1, -1, 1])
check("spin right",            mec.wheel_speeds(Velocity(omega=1)),       [-1, 1, -1, 1])
check("diagonal fwd+right",    mec.wheel_speeds(Velocity(vx=1, vy=1)),    [0, 2, 2, 0])
check("stopped",               mec.wheel_speeds(Velocity()),              [0, 0, 0, 0])

print("\nnormalisation keeps direction (this is why we do not clip per wheel)")
check("diagonal scaled to unit", _normalise(mec.wheel_speeds(Velocity(vx=1, vy=1))),
      [0, 1, 1, 0])
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
