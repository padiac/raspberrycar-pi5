"""Looking for something that has gone, with no camera and no car attached.

The case that produced this file, in the owner's words: the ball was in
view, the car turned, the ball was not in view, and the car sat there.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from behaviors.reacquire import Reacquire          # noqa: E402

FAILURES = []

#: Explicit everywhere, so no check here depends on the live register --
#: which on this machine is also what the car is using to drive.
TUNING = dict(enabled=True, patience_s=6.0, sweep_degrees=70.0,
              dwell_s=0.4, opening_rate=45.0, sweep_period_s=2.4,
              effort=0.45, gain=0.02, blind_rate=0.30,
              divergence_ticks=6, runaway_factor=1.5)


def check(name, got, want):
    ok = got == want
    print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    if not ok:
        print(f"        got  {got!r}")
        print(f"        want {want!r}")
        FAILURES.append(name)


def sign(name, value, want):
    got = 0 if abs(value) < 1e-6 else (1 if value > 0 else -1)
    print(f"  {'PASS' if got == want else 'FAIL'}  {name} (w={value:+.3f})")
    if got != want:
        FAILURES.append(name)


def truthy(name, value):
    check(name, bool(value), True)


def falsy(name, value):
    check(name, bool(value), False)


def hunt(**over):
    settings = dict(TUNING)
    settings.update(over)
    return Reacquire(**settings)


print("nothing lost, nothing to do")

idle = hunt()
check("a search that was never started does not steer",
      idle.steer(0.0, 0.05), None)
check("and says so", idle.status()["searching"], False)


print()
print("the car turned away from it, which the gyro knows about")

# The whole point. The ball was seen with the car at heading 0. The car
# then turned right to +30, which is why the ball left the frame on the
# left. Turning back to 0 must put it back in view, and that is a number,
# not a guess: it needs nothing from the lens.
search = hunt()
search.lost(last_offset=-0.9, last_drift=-0.2, turned=0.0)
first = search.steer(turned=30.0, dt=0.05)
sign("lost off the left after turning right: turn left again", first, -1)

# ...and it keeps turning that way until it has undone the rotation.
sign("still turning left while it is still 20 degrees out",
     search.steer(turned=10.0, dt=0.05), -1)

# The mirror image, because a sign error here would send the car exactly
# the wrong way and still look like a working search.
mirror = hunt()
mirror.lost(last_offset=+0.9, last_drift=+0.2, turned=0.0)
sign("lost off the right after turning left: turn right again",
     mirror.steer(turned=-30.0, dt=0.05), +1)

# Arriving back where it was seen, the first thing it does is stop
# turning, not sail past. If the target is still there this is the whole
# search, and it is exact.
arrived = hunt()
arrived.lost(last_offset=-0.9, last_drift=0.0, turned=15.0)
sign("back at the heading it last saw it from: stop turning",
     arrived.steer(turned=15.0, dt=0.01), 0)
truthy("and it says that is what it is doing",
       "looking back where" in arrived.describe("ball"))

# It holds there long enough for the detector to actually get a look. A
# sweep that starts widening on arrival has swung past before the first
# frame it was supposed to search came back.
held = []
for _ in range(30):                       # 0.3s, inside the 0.4s dwell
    held.append(arrived.steer(turned=15.0, dt=0.01))
truthy("and holds still there rather than sweeping straight past",
       max(abs(w) for w in held) < 1e-9)
for _ in range(30):                       # past the dwell now
    arrived.steer(turned=15.0, dt=0.01)
truthy("then starts widening once it has had a proper look",
       abs(arrived.aim - arrived.anchor) > 1.0)


print()
print("when turning back is not enough, because it moved too")

widening = hunt()
widening.lost(last_offset=+0.8, last_drift=+0.5, turned=0.0)
spreads = []
for _ in range(60):                       # three seconds
    widening.steer(turned=0.0, dt=0.05)
    spreads.append(abs(widening.aim - widening.anchor))
truthy("the search opens out rather than staying on one heading",
       spreads[-1] > spreads[4])
truthy("but not past the limit it was given",
       max(spreads) <= TUNING["sweep_degrees"] + 1e-6)

# It should look on the side the target went first. Sampling early, while
# the sweep is still opening, is where that shows.
leaning = hunt()
leaning.lost(last_offset=+0.8, last_drift=+0.5, turned=0.0)
for _ in range(14):                       # 0.7s: past the dwell, sweep open
    leaning.steer(turned=0.0, dt=0.05)
truthy("and it looks first towards the side the target went",
       leaning.aim > leaning.anchor)

other = hunt()
other.lost(last_offset=-0.8, last_drift=-0.5, turned=0.0)
for _ in range(14):
    other.steer(turned=0.0, dt=0.05)
truthy("and the other way when it went the other way",
       other.aim < other.anchor)

# A target lost from near the middle has no edge to tell you which way it
# went, so which way it was moving is all there is.
drifting = hunt()
drifting.lost(last_offset=-0.02, last_drift=+1.5, turned=0.0)
check("lost near the middle, it follows which way it was moving",
      drifting.status()["side"], "right")


print()
print("not turning for ever")

patient = hunt(patience_s=1.0)
patient.lost(last_offset=0.5, last_drift=0.0, turned=0.0)
steps = 0
while patient.steer(turned=0.0, dt=0.05) is not None:
    steps += 1
    if steps > 200:
        break
truthy("it gives up after about the time it was given",
       18 <= steps <= 22)
check("and stops claiming to be searching", patient.status()["searching"], False)
# ...but it has to stay tellable from a search that never started, because
# "point me at something" and "it went somewhere I could not find" are
# different messages for the person reading the page, and `searching` is
# False in both cases.
check("  while still being distinguishable from never having looked",
      patient.status()["gave_up"], True)
check("  which a search that was never started is not",
      hunt().status()["gave_up"], False)

again = hunt(patience_s=1.0)
again.lost(last_offset=0.5, last_drift=0.0, turned=0.0)
while again.steer(turned=0.0, dt=0.05) is not None:
    pass
again.found()
check("  and finding it afterwards clears that", again.status()["gave_up"], False)
check("which reads as having lost it, not as never having seen it",
      patient.describe("ball"), "lost the ball")

stopped = hunt()
stopped.lost(last_offset=0.5, last_drift=0.0, turned=0.0)
stopped.found()
check("a target that comes back ends the search at once",
      stopped.steer(turned=40.0, dt=0.05), None)

turned_off = hunt(enabled=False)
turned_off.lost(last_offset=0.5, last_drift=0.0, turned=0.0)
check("switched off, it does not go hunting", turned_off.steer(0.0, 0.05), None)


print()
print("no gyro, which is not the same as not turning")

# `turn_rate` is None when the board is not answering. Closing a loop on a
# heading that is not being measured would hold the car at whatever number
# it last believed, which looks like a car that has decided to stare at a
# wall. Sweeping open-loop is worse and honest.
blind = hunt()
blind.lost(last_offset=+0.9, last_drift=0.0, turned=0.0, have_gyro=False)
sign("with no gyro it still sweeps towards where it went",
     blind.steer(turned=None, dt=0.05), +1)
truthy("and says that is what it is reduced to",
       "no gyro" in blind.describe("ball"))

reverses = [blind.steer(turned=None, dt=0.1) for _ in range(40)]
truthy("and the sweep does come back the other way",
       min(reverses) < 0 < max(reverses))


print()
print("wired up backwards, which is how this was found")

# Reported from the room: the ball was dead centre and the car turned
# itself most of the way round. The flight recorder had it -- one follow
# run swept 353 degrees -- and the cause: over every tick where a turn was
# commanded and the heading moved, the two agreed 15 times and disagreed
# 177. The gyro counts the opposite way to the wheels, so steering towards
# the aim takes the car away from it, and turning harder makes it worse.
#
# The sign itself is fixed elsewhere once it has been measured on the car.
# What is checked here is that getting it wrong again costs a poor search
# rather than a pirouette, because a closed loop that cannot tell it is
# diverging is not a closed loop.
#: Degrees of real rotation per unit of commanded turn per tick. This car
#: at full effort turns at something like eighty degrees a second, which
#: over a 0.05s tick is four degrees; the 0.45 effort cap makes nine the
#: right figure here. It matters because the guard needs a few ticks to
#: be sure, and how far the car gets in those ticks is the whole question.
DEGREES_PER_COMMAND = 9.0


def spin(search, backwards, ticks=200):
    """Drive the search against a car, correctly or incorrectly wired.

    Returns how far it got, and what it was saying at the moment it
    noticed something was wrong -- which has to be captured as it happens,
    because by the time the search has finished it is saying it gave up,
    and that is a different and less useful sentence.
    """
    heading = 0.0
    worst = 0.0
    said = None
    for _ in range(ticks):
        command = search.steer(turned=heading, dt=0.05)
        if search.blind and said is None:
            said = search.describe("ball")
        if command is None:
            return worst, said
        heading += command * DEGREES_PER_COMMAND * (-1.0 if backwards else 1.0)
        worst = max(worst, abs(heading - search.anchor))
    return worst, said


backwards = hunt()
backwards.lost(last_offset=+0.9, last_drift=0.0, turned=0.0)
worst, said = spin(backwards, backwards=True)
truthy("a backwards gyro does not send the car round and round (%.0f deg)"
       % worst,
       worst < TUNING["sweep_degrees"] * TUNING["runaway_factor"])
truthy("  the loop works out that it is diverging", backwards.blind)
truthy("  and says which of the two things is wrong",
       "which way is positive" in (backwards.broke or ""))
truthy("  in words the page can show, at the moment it notices",
       "open-loop" in (said or ""))

# And the backstop, for anything the divergence test does not catch: a
# hard ceiling on how far round the search may take the car at all.
runaway = hunt(divergence_ticks=99)
runaway.lost(last_offset=+0.9, last_drift=0.0, turned=0.0)
far = None
for _ in range(200):
    far = runaway.steer(turned=200.0, dt=0.05)
    if far is None:
        break
check("and a hard ceiling stops it however well it thought it was doing",
      far, None)
# Worth being exact about what that buys, because it is not "the car never
# goes further than the ceiling". The search stops commanding once the car
# is past it; how far the car carries on afterwards belongs to the car. A
# faster chassis than this one would overshoot more, and the only thing
# that makes the number small is noticing early.
truthy("  recorded as having gone too far rather than as a clean finish",
       "too far" in (runaway.broke or ""))

# The guard must not fire on an honest search. The sweep opens faster than
# the car follows it, so the distance to the aim grows routinely; reading
# that as divergence would disable the good loop on every single search.
honest = hunt()
honest.lost(last_offset=+0.9, last_drift=0.0, turned=0.0)
spin(honest, backwards=False)
falsy("an honest search is not mistaken for a broken one", honest.blind)
check("  and nothing is reported as broken", honest.broke, None)


print()
print("staying inside its limits")

hard = hunt(gain=5.0, effort=0.4)
hard.lost(last_offset=0.9, last_drift=0.0, turned=0.0)
# Inside the runaway ceiling, so that what is being checked is the
# effort clamp and not the ceiling that would stop it anyway.
worst = max(abs(hard.steer(turned=-60.0, dt=0.05) or 0.0) for _ in range(10))
truthy("a target a long way round does not produce a bigger command "
       "than the effort allows", worst <= 0.4 + 1e-9)

twice = hunt()
twice.lost(last_offset=+0.9, last_drift=0.0, turned=0.0)
twice.steer(turned=0.0, dt=1.0)
twice.lost(last_offset=-0.9, last_drift=0.0, turned=99.0)
check("a second report of the same loss does not restart the clock",
      twice.status()["anchor"], 0.0)

print()
if FAILURES:
    print(f"{len(FAILURES)} FAILURE(S): {FAILURES}")
    sys.exit(1)
print("all reacquire checks passed")
