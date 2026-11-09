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
from behaviors.patrol import PatrolBehavior
from control.safety import NO_ECHO_CEILING, SafetyLayer
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

print()
print("no-echo resolution")

from behaviors.patrol import (
    PatrolBehavior as _Patrol, UNKNOWN_CM as _UNK, OPEN_UNKNOWN_CM as _OPEN)


def _resolve(clearance, unknown):
    p = _Patrol()
    p.clearance = dict(clearance)
    p.unknown = set(unknown)
    return p._resolve_unknowns()

# The emptiest direction in the room reports nothing at all, and used to score
# below a wall an arm's length away.
_r = _resolve({-90: 74.0, -45: 240.0, 0.0: None, 45: 77.0, 90: 103.0}, {0.0})
check_eq("silence between two long readings is distance", _r[0.0], _OPEN)

# A soft thing is wider than the beam, so its neighbours see it too.
_r = _resolve({-90: 74.0, -45: 25.0, 0.0: None, 45: 30.0, 90: 103.0}, {0.0})
check_eq("silence with something close beside it stays suspect", _r[0.0], _UNK)

# Silence everywhere is a room with no wall within range -- the dead sensor
# case is caught by the rule about going deaf, not here.
_r = _resolve({a: None for a in (-90, -45, 0.0, 45, 90)},
              {-90, -45, 0.0, 45, 90})
check_eq("silence all round reads as open", _r[0.0], _OPEN)

# Readings that were heard are left exactly as they were.
_r = _resolve({-45: 25.0, 0.0: None, 45: 240.0}, {0.0})
check_eq("a real reading is not touched", _r[-45], 25.0)

print()
print("pan servo travel")


class _FakeLink:
    def __init__(self):
        self.sent = []

    def send_servo(self, deg):
        self.sent.append(deg)


from hal.sensors import PanServo, CENTRE_DEG, PAN_LIMIT

# Out and back has to land where it started. With any trim at all one side
# reaches the end of the servo travel first, and a head that reports the angle
# it was asked for rather than the one it reached accumulates that difference
# on every relative move -- two steps out and two back miss the middle.
for _trim in (0.0, 10.0, -25.0):
    for _way in (45, -45):
        _pan = PanServo(_FakeLink(), trim=_trim)
        # Two steps is as far as the head goes, so this is the round trip a
        # d-pad can actually make. Pressing a third time is pressing into the
        # limit, and no amount of coming back undoes a press that never moved.
        _n = int(_pan.limit / 45)
        for _ in range(_n + 2):
            _pan.nudge(_way)
        for _ in range(_n):
            _pan.nudge(-_way)
        check_eq("trim %+.0f, %s: out to the stop and back is centred"
                 % (_trim, "left" if _way < 0 else "right"), _pan.angle, 0.0)

# Both sides have to reach the same distance, or one direction takes shorter
# steps than the other and the head walks off centre.
for _trim in (0.0, 10.0, -25.0):
    _pan = PanServo(_FakeLink(), trim=_trim)
    _right = _pan.look(PAN_LIMIT)
    _left = _pan.look(-PAN_LIMIT)
    check_eq("trim %+.0f: travel is symmetric" % _trim, _right, -_left)

# And nothing may be commanded past what the servo can actually do.
_link = _FakeLink()
_pan = PanServo(_link, trim=20.0)
for _a in (-90, -45, 0, 45, 90):
    _pan.look(_a)
check_eq("no command outside the servo range",
         [d for d in _link.sent if d < 0 or d > 180], [])

# A reversed head commands the other way but still reports the angle in the
# car own sense, which is what the patrol sweep labels its readings with.
_link = _FakeLink()
_pan = PanServo(_link, reversed_=True)
check_eq("reversed head reports the requested angle", _pan.look(45), 45.0)
check_eq("  but commands the mirror of it", _link.sent[-1], CENTRE_DEG - 45)

print()
print("safety layer")
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

# One corner is enough. Requiring both meant the only thing the pair ever
# objected to was a gap narrower than the car, while the collision that
# actually happens -- clipping a table leg with one corner -- went unopposed.
out = safety.filter(Velocity(vx=1.0), world(prox=Proximity(left=True)))
check_eq("left corner alone: forward blocked", out.vx, 0.0)
out = safety.filter(Velocity(vx=1.0), world(prox=Proximity(right=True)))
check_eq("right corner alone: forward blocked", out.vx, 0.0)
out = safety.filter(Velocity(vx=1.0), world(prox=Proximity()))
check_eq("corners clear: forward untouched", round(out.vx, 3), 1.0)

# Escape has to survive the rule that stopped the car.
out = safety.filter(Velocity(vx=-1.0), world(prox=Proximity(left=True, right=True)))
check_eq("corners blocked: reversing still allowed", out.vx, -1.0)
# Rotation is gated by which corner it drives forward. Turning right carries
# the left corner forward, so a blocked left corner forbids it -- and permits
# turning left, which withdraws that corner. Escape has to survive the rule
# that stopped the car, or it wedges there.
out = safety.filter(Velocity(omega=1.0), world(prox=Proximity(left=True)))
check_eq("left corner blocked: turning right blocked", out.omega, 0.0)
out = safety.filter(Velocity(omega=-1.0), world(prox=Proximity(left=True)))
check_eq("  but turning left, which pulls it back, is allowed", out.omega, -1.0)
out = safety.filter(Velocity(omega=-1.0), world(prox=Proximity(right=True)))
check_eq("right corner blocked: turning left blocked", out.omega, 0.0)
out = safety.filter(Velocity(omega=1.0), world(prox=Proximity(right=True)))
check_eq("  but turning right is allowed", out.omega, 1.0)
out = safety.filter(Velocity(vx=-1.0), world(prox=Proximity(left=True, right=True)))
check_eq("both corners blocked: reversing is the way out", out.vx, -1.0)

# The infrared writes into the same table the ultrasonic glance fills, which
# is the whole of the integration: every rule already built on that table
# starts working from the corners for free.
print()
print("infrared feeds the side memory")
import time as _time
from behaviors.patrol import PatrolBehavior, IR_ASSUMED_CM, SIDE_MEMORY_S
_p = PatrolBehavior()
_now = _time.monotonic()
_p._note_infrared(world(prox=Proximity(left=True)), _now)
check_eq("left corner recorded as close", _p._side(-1, _now), IR_ASSUMED_CM)
check_eq("right side left alone", _p._side(1, _now), 999.0)
check_eq("and it fades like any other sighting",
         _p._side(-1, _now + SIDE_MEMORY_S + 0.1), 999.0)
_p2 = PatrolBehavior()
_p2._note_infrared(world(prox=None), _now)
check_eq("no infrared fitted: nothing recorded", _p2._side(-1, _now), 999.0)

print("\nnoticing that the wheels turn and the car does not move")
# The safety layer cannot catch this. It refuses to drive at something close,
# and this is the case where the reading never becomes close: pressed against
# something the beam reads past, or a wheel on a cable. Everything downstream
# is being told what it expects to hear.
patrol = PatrolBehavior()


def cruise_for(seconds, distance, step=0.05, start=0.0):
    """Feed the behaviour a stretch of driving, and say if it cried foul."""
    t = start
    verdict = False
    while t < start + seconds:
        ahead = distance(t) if callable(distance) else distance
        verdict = patrol._going_nowhere(
            WorldState(range_cm=ahead, range_expected=True), t)
        t += step
    return verdict


check_eq("four seconds of no change: too early to say",
         cruise_for(4.0, 65.5), False)

patrol.progress = []
check_eq("eight seconds of no change: not going anywhere",
         cruise_for(8.0, 65.5), True)

patrol.progress = []
check_eq("closing on something steadily: fine",
         cruise_for(8.0, lambda t: 200.0 - 12.0 * t), False)

patrol.progress = []
# Creeping is what the safety layer makes it do near an obstacle, measured at
# 2-3 cm/s, and it must not read as stuck.
check_eq("creeping at 2.5 cm/s: fine",
         cruise_for(8.0, lambda t: 60.0 - 2.5 * t), False)

patrol.progress = []
# Silence is not a small number. Reading it as one would make an open room and
# a sofa the same thing, again.
check_eq("nothing coming back: says nothing either way",
         cruise_for(8.0, -1.0), False)


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

# A timed-out pulse is an answer, but a poor one. In a house it is usually not
# "nothing within four metres" at all: sound leaves a pane of glass like light
# leaves a mirror, so a floor-to-ceiling window is invisible from every
# approach but head-on, and a smooth wall met at an angle is the same. Soft
# things swallow the pulse instead. All of them report what an empty room
# reports -- which is how this car kept driving into a window.
#
# So it may go, because refusing to cross open space would make it useless,
# but only at a crawl.
# Spelled out rather than taken from the register, which is where the
# running car gets it: this is a check on the mechanism, not on whatever
# somebody has set today. Written the other way it failed the moment the
# creep was switched off from the web page, which is a test reporting on
# the operator rather than on the code.
creeping = SafetyLayer(no_echo_ceiling=NO_ECHO_CEILING)
out = creeping.filter(Velocity(vx=1.0),
                      WorldState(range_cm=-1.0, range_expected=True))
check_eq("nothing came back: creeps rather than charging", out.vx, NO_ECHO_CEILING)
check_eq("  and says why", any("creep" in r for r in creeping.last.interventions), True)

out = safety.filter(Velocity(vx=0.1), WorldState(range_cm=-1.0, range_expected=True))
check_eq("  a slow request is left alone", out.vx, 0.1)

out = safety.filter(Velocity(vx=-1.0), WorldState(range_cm=-1.0, range_expected=True))
check_eq("  and reversing is not a forward clearance question", out.vx, -1.0)

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

# =========================================================================
# What has been seen in each direction lately.
#
# One rangefinder on a servo measures one direction at a time, so the shape
# of a room only exists if somebody keeps the spokes together. These check
# that keeping them does not quietly invent anything.

from perception.polar import PolarMemory, bucket     # noqa: E402

check_eq("directions are rounded to the resolution the beam supports",
         bucket(32.0), 30.0)
check_eq("and rounding goes to the nearest, not down", bucket(33.0), 35.0)

memory = PolarMemory(memory_s=4.0)
memory.note(30.0, 120.0, 100.0)
memory.note(-30.0, 40.0, 100.5)
fresh = memory.readings(101.0)
check_eq("both spokes are remembered", len(fresh), 2)
check_eq("and come back in order, left first", fresh[0]["angle"], -30.0)
check_eq("carrying how old they are", round(fresh[1]["age"], 1), 1.0)

# A silence is worth recording. It is not a distance, but "I looked there
# and heard nothing" is a different thing from "I have not looked there",
# and a picture that cannot tell them apart is the one that drove this car
# into a sofa.
memory.note(60.0, -1.0, 101.0)
silent = [r for r in memory.readings(101.0) if r["angle"] == 60.0][0]
check_eq("a silence is kept", silent is not None, True)
check_eq("and is marked as not an echo", silent["echo"], False)
check_eq("a measurement is marked as one",
         memory.readings(101.0)[0]["echo"], True)

# Nothing to file is not the same as filing nothing.
before = len(memory.readings(101.0))
memory.note(0.0, None, 101.0)
memory.note(None, 50.0, 101.0)
check_eq("a reading that never arrived is not invented",
         len(memory.readings(101.0)), before)

# The car moves while it looks, so an old spoke describes a room it has
# left. Drawing that as current is worse than drawing nothing.
check_eq("a stale reading drops out", len(memory.readings(106.0)), 0)
memory.prune(106.0)
check_eq("and is not kept around", len(memory._seen), 0)

# Two readings in the same direction: the later one wins outright rather
# than being averaged with a measurement of somewhere else.
memory.note(45.0, 200.0, 110.0)
memory.note(44.0, 30.0, 110.5)
only = memory.readings(111.0)
check_eq("the same direction twice is one spoke", len(only), 1)
check_eq("and it is the newer reading", only[0]["cm"], 30.0)

if FAILURES:
    print(f"{len(FAILURES)} FAILURE(S): {FAILURES}")
    raise SystemExit(1)
print("all checks passed")
