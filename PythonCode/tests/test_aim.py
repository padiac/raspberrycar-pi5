"""Turning a measured number of degrees, with no car attached.

Every check names something that went wrong in a room, or would. Nothing
reads the live register: each Aim is built with explicit numbers, because
a test that moved a setting would change what the car does on its next
tick, and on this machine the car is usually running.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from control.aim import Aim, EFFORT                    # noqa: E402

FAILURES = []

#: Measured on this car: it turns at about thirty degrees a second once
#: moving, and the shortest pulse that moves it at all is worth under two.
RATE = 30.0
TUNING = dict(settle_s=0.22, smallest_deg=1.5, longest_s=0.55,
              tolerance_deg=1.2, patience=2.0, brake_s=0.14)


def check(name, got, want):
    ok = got == want
    print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    if not ok:
        print(f"        got  {got!r}")
        print(f"        want {want!r}")
        FAILURES.append(name)


def close(name, got, want, tol):
    ok = got is not None and abs(got - want) <= tol
    print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    if not ok:
        print(f"        got  {got!r}, wanted {want!r} +/- {tol}")
        FAILURES.append(name)


def truthy(name, value):
    check(name, bool(value), True)


def falsy(name, value):
    check(name, bool(value), False)


def aim(**over):
    settings = dict(TUNING)
    settings.update(over)
    return Aim(RATE, **settings)


def drive(mover, turned, now, ticks=60, dt=0.05, rate=RATE, backwards=False):
    """Run a step to completion against a car that turns at `rate`.

    `rate` is the speed the chassis reaches at the one command this
    module ever sends, not a speed per unit of command -- below the turn
    floor these motors do not rotate at all, so there is no such thing as
    half a command. Multiplying by the command as well made the simulated
    car five times slower than the real one, and every step then ran out
    of time and was recorded as a wheel against a chair leg.
    """
    for _ in range(ticks):
        omega = mover.omega(now, turned)
        if omega == 0.0 and not mover.stepping:
            return turned, now
        way = 1.0 if omega > 0 else -1.0
        turned += way * rate * dt * (-1.0 if backwards else 1.0)
        now += dt
    return turned, now


print("asking for a turn")

small = aim()
falsy("a turn smaller than the car can make is not attempted",
      small.request(0.9, 0.0, 100.0))

# ...and "smaller than the car can make" is not a fixed number. Once the
# lead is known -- a tick of commanded turning plus the coast -- a
# request inside it would end on the tick it began, commanding nothing
# while the step counter went up. The module works that out rather than
# being told, because a separately tuned floor has to be kept in step
# with the lead by hand and would not be.
coasting = aim(brake_s=0.0)
coasting.coast = 4.0
coasting.request(30.0, 0.0, 100.0)
coasting.omega(100.0, 0.0)
coasting.omega(100.06, 1.0)          # two calls, so a tick is known
truthy("the floor rises to cover the coast and the tick",
       coasting.useless_below >= 5.0)
coasting.forget()
falsy("and a request inside it is refused rather than silently doing "
      "nothing", coasting.request(coasting.useless_below * 0.8, 0.0, 200.0))
check("so the step counter does not climb while the car sits still",
      coasting.steps, 1)
falsy("and nothing is commanded for it", small.stepping)
check("nor is it counted as a step", small.steps, 0)

ready = aim()
truthy("a turn worth making is accepted", ready.request(6.0, 0.0, 100.0))
truthy("and a step is now running", ready.stepping)

# A decision taken while the car is mid-step was taken on a picture from
# before the step began. Acting on the same stale picture twice in a row
# is the whole fault this replaces.
falsy("a second request while still turning is refused",
      ready.request(4.0, 0.0, 100.1))

# Nonsense in, nothing out, rather than a step of NaN degrees that never
# ends because every comparison against it is false.
blank = aim()
falsy("a request of nothing at all is refused",
      blank.request(None, 0.0, 100.0))
falsy("and so is one that is not a number",
      blank.request(float("inf"), 0.0, 100.0))


print()
print("the gyro ends the step, not the clock")

closing = aim()
closing.request(6.0, 0.0, 100.0)
landed, _when = drive(closing, 0.0, 100.0)
close("it stops within a tolerance of where it was asked to go",
      landed, 6.0, 2.0)
falsy("and the step is over", closing.stepping)
check("nothing was abandoned", closing.abandoned, 0)

left = aim()
left.request(-6.0, 0.0, 100.0)
landed, _when = drive(left, 0.0, 100.0)
close("and the same the other way", landed, -6.0, 2.0)

# A big correction is taken in pieces with a look between them, because
# whatever is being chased may move while the car is swinging.
long_way = aim()
long_way.request(90.0, 0.0, 100.0)
close("a huge request is cut to one step's worth",
      abs(long_way.asked_deg), RATE * TUNING["longest_s"], 0.1)


print()
print("when the car does not do as it is told")

# A wheel against a chair leg. The gyro target never arrives, and hanging
# on a goal that will never come is worse than a turn of the wrong size.
stuck = aim()
stuck.request(6.0, 0.0, 100.0)
now = 100.0
for _ in range(200):
    omega = stuck.omega(now, 0.0)        # heading never changes
    if omega == 0.0 and not stuck.stepping:
        break
    now += 0.05
falsy("a step that cannot finish is abandoned", stuck.stepping)
check("and is counted, because a car that silently fails to turn looks "
      "exactly like one that decided not to", stuck.abandoned, 1)
close("within about the patience it was given",
      now - 100.0, 6.0 / RATE * TUNING["patience"], 0.3)

# No board, no heading. The clock has to do it, which is worse and still
# works -- and must not be counted as a failure, because nothing failed.
blind = aim()
blind.request(6.0, None, 100.0)
now = 100.0
for _ in range(200):
    omega = blind.omega(now, None)
    if omega == 0.0 and not blind.stepping:
        break
    now += 0.05
falsy("with no gyro the step still ends", blind.stepping)
check("and is not recorded as having gone wrong", blind.abandoned, 0)


print()
print("stopping, which is not the same as letting go")

# The firmware's idea of stopping is to set the PWM to zero, which
# switches off the driver's output stage and leaves the wheels free to
# keep turning. They do: a median of 8.6 degrees, measured over sixteen
# runs. So a step ends with a push the other way.
braking = aim()
braking.request(6.0, 0.0, 100.0)
landed, when = drive(braking, 0.0, 100.0)
pushed = braking.omega(when + 0.01, landed)
check("a step that has just ended is pushing the other way",
      pushed, -EFFORT)
truthy("and says so", braking.status(when + 0.01)["braking"])
falsy("the step itself is over", braking.stepping)
falsy("and no new one can start mid-brake",
      braking.request(6.0, landed, when + 0.01))

check("the push stops when it has had long enough",
      braking.omega(when + 0.2, landed), 0.0)
falsy("and then it is just settling", braking.status(when + 0.2)["braking"])

turning_left = aim()
turning_left.request(-6.0, 0.0, 100.0)
landed, when = drive(turning_left, 0.0, 100.0)
check("a left turn is braked to the right",
      turning_left.omega(when + 0.01, landed), EFFORT)

# Braking can be switched off, for a chassis that stops on its own or
# one where pushing back does more harm than good.
coasting = aim(brake_s=0.0)
coasting.request(6.0, 0.0, 100.0)
landed, when = drive(coasting, 0.0, 100.0)
check("with braking off, a finished step commands nothing",
      coasting.omega(when + 0.01, landed), 0.0)


print()
print("holding still between steps")

pausing = aim()
pausing.request(6.0, 0.0, 100.0)
landed, when = drive(pausing, 0.0, 100.0)
truthy("after a step it settles", pausing.settling(when))
falsy("and will not take a new one yet", pausing.request(6.0, landed, when))
check("commanding nothing once the braking is done",
      pausing.omega(when + 0.2, landed), 0.0)
falsy("still settling a tenth of a second later", pausing.ready(when + 0.1))
# The settle is for the camera and starts once the car has actually
# stopped, which is after the brake rather than after the decision.
falsy("and still settling while the brake is on",
      pausing.ready(when + TUNING["settle_s"]))
truthy("ready once braking and settling are both over",
       pausing.ready(when + TUNING["brake_s"] + TUNING["settle_s"] + 0.01))
truthy("and accepting work again",
       pausing.request(6.0, landed,
                       when + TUNING["brake_s"] + TUNING["settle_s"] + 0.01))


print()
print("what it commands and what it reports")

pushing = aim()
pushing.request(6.0, 0.0, 100.0)
check("a turn to the right is commanded to the right",
      pushing.omega(100.0, 0.0), EFFORT)
pulling = aim()
pulling.request(-6.0, 0.0, 100.0)
check("and a turn to the left, to the left",
      pulling.omega(100.0, 0.0), -EFFORT)

# Below the floor these motors do not rotate the chassis at all, so a
# gentler command is not a gentler turn, it is no turn.
truthy("never commands less than will actually move the car",
       abs(pushing.omega(100.0, 0.0)) >= EFFORT)

reporting = aim()
reporting.request(7.0, 0.0, 100.0)
status = reporting.status(100.0)
check("status says a step is running", status["stepping"], True)
close("and how far it was asked to go", status["asked_deg"], 7.0, 0.1)
check("and how many have been taken", status["steps"], 1)

mid_brake = aim()
mid_brake.request(6.0, 0.0, 100.0)
landed, when = drive(mid_brake, 0.0, 100.0)
mid_brake.forget()
check("forgetting cancels a brake rather than leaving the car pushing "
      "back for a step that no longer exists",
      mid_brake.omega(when + 0.01, landed), 0.0)

reporting.forget()
falsy("forgetting stops the step", reporting.stepping)
check("and commands nothing", reporting.omega(100.0, 0.0), 0.0)
falsy("and is immediately ready again", reporting.settling(100.0))


print()
print("settings")

try:
    Aim(RATE, smallest_deg=2.0)
    named = True
except TypeError:
    named = False
truthy("a setting that exists is accepted", named)

try:
    Aim(RATE, smalest_deg=2.0)          # deliberate typo
    caught = False
except TypeError:
    caught = True
truthy("a setting that does not is refused by name, not ignored", caught)

# A rate of nothing would make every step's deadline zero and every step
# end on the tick it began.
fallback = Aim(0.0, **TUNING)
truthy("a car with no measured turn rate still turns", fallback.rate > 0)
fallback.request(6.0, 0.0, 100.0)
truthy("and takes steps", fallback.stepping)

print()
if FAILURES:
    print(f"{len(FAILURES)} FAILURE(S): {FAILURES}")
    sys.exit(1)
print("all aim checks passed")
