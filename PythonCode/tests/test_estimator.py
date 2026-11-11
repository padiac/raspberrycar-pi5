"""What the car believes about a target, with no camera and no car attached.

Every check below names something that went wrong, or could, in a room.
Nothing here reads the live register: each filter is built with explicit
numbers, because a test that sets a global changes the next test and on
this machine would also change what the car does while it is driving.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from control.estimator import Scalar, TargetTrack      # noqa: E402
from perception.types import Detection                 # noqa: E402

FAILURES = []
WIDTH, HEIGHT = 640, 360

#: The numbers the car ships with, named here so a check that depends on
#: them says so rather than appearing to hold for any tuning.
JITTER = 0.004
MANOEUVRE = 1.0


def check(name, got, want):
    ok = got == want
    print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    if not ok:
        print(f"        got  {got!r}")
        print(f"        want {want!r}")
        FAILURES.append(name)


def close(name, got, want, tol):
    ok = abs(got - want) <= tol
    print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    if not ok:
        print(f"        got  {got!r}, wanted {want!r} +/- {tol}")
        FAILURES.append(name)


def truthy(name, value):
    check(name, bool(value), True)


def falsy(name, value):
    check(name, bool(value), False)


def ball(offset, fill=0.30, label="ball", confidence=0.9):
    """One sighting at a given bearing and apparent size."""
    box_h = fill * HEIGHT
    centre = WIDTH / 2 + offset * (WIDTH / 2)
    return Detection(label, confidence,
                     (centre - box_h / 2, (HEIGHT - box_h) / 2,
                      centre + box_h / 2, (HEIGHT + box_h) / 2),
                     200)


def scalar():
    return Scalar(jitter=JITTER, manoeuvre=MANOEUVRE)


#: The bar a sighting must clear to be worth starting a track on.
#: Measured on the car: a ball properly in view never scored below 0.72
#: and nothing else in the room rose above 0.62.
CONFIDENT = 0.65


def track(**kw):
    kw.setdefault("jitter", JITTER)
    kw.setdefault("manoeuvre", MANOEUVRE)
    kw.setdefault("confident", CONFIDENT)
    return TargetTrack("ball", **kw)


print("one quantity, estimated from noisy looks")

filt = scalar()
falsy("before anything is seen, nothing is known", filt.known)
check("and asking where it is gets None, not a plausible zero",
      filt.value, None)

filt.correct(0.4)
close("the first sighting is taken at face value", filt.value, 0.4, 1e-9)
close("with no claim about which way it was going", filt.rate, 0.0, 1e-9)

# A filter that keeps shrinking its own uncertainty while being told the
# same thing over and over has stopped doing arithmetic and started
# agreeing with itself.
filt = scalar()
for _ in range(40):
    filt.predict(0.05)
    filt.correct(0.4)
close("told the same thing forty times, it still says that thing",
      filt.value, 0.4, 1e-6)
close("and still says it is not moving", filt.rate, 0.0, 1e-6)

# The covariance has to stay symmetric or the gains drift apart and the
# filter diverges. This is the one piece of the longhand algebra that a
# matrix library would have got right for free.
filt = scalar()
for step in range(12):
    filt.predict(0.05)
    filt.correct(0.1 * step)
truthy("the covariance stays a covariance (p00 and p11 positive)",
       filt.p00 > 0 and filt.p11 > 0)
truthy("and stays correlated the right way for a thing moving forwards",
       filt.p01 > 0)

# Velocity is the whole reason this is a filter and not an average.
filt = scalar()
for step in range(40):
    filt.predict(0.05)
    filt.correct(step * 0.05 * 0.8)        # 0.8 half-frames per second
close("a target crossing at 0.8/s is believed to be doing about that",
      filt.rate, 0.8, 0.08)

# Noise in, much less noise out -- on the rate, which is what the steering
# loop's damping multiplies. The sequence is fixed rather than random so
# that a failure here is a change in the code, not an unlucky seed.
wobble = [0.004, -0.003, 0.005, -0.005, 0.002, 0.004, -0.004, 0.003,
          -0.002, 0.005, -0.005, 0.001, 0.003, -0.004, 0.004, -0.003,
          0.002, 0.005, -0.005, 0.004]
filt = scalar()
for value in wobble:
    filt.predict(0.05)
    filt.correct(value)
differenced = (wobble[-1] - wobble[-2]) / 0.05
truthy("a still target's believed rate is far quieter than a difference "
       f"of two looks ({abs(filt.rate):.3f} against {abs(differenced):.3f})",
       abs(filt.rate) < abs(differenced) / 4.0)

# Coasting. The point is that it does not stop dead, and that it does not
# wander off either.
filt = scalar()
for step in range(20):
    filt.predict(0.05)
    filt.correct(step * 0.05 * 1.0)
where = filt.value
filt.predict(0.1)
truthy("with nothing new to go on it keeps moving the way it was",
       filt.value > where)
truthy("and admits to being less sure than it was", filt.doubt > 0.0)


print()
print("which of these is the thing being followed")

# The old code took the largest box every frame. Two balls, one nearer,
# and a flicker in the sizes was enough to swap which one was chased --
# with nothing in the code that could have told you it had happened.
following = track()
following.update([ball(-0.3)], (WIDTH, HEIGHT), 0.05)
chosen = following.update(
    # A bigger one has appeared across the room. Bigger is not the
    # question; "is this the one I was following" is.
    [ball(-0.28, fill=0.30), ball(+0.60, fill=0.55)],
    (WIDTH, HEIGHT), 0.05)
close("with two in view it keeps the one it was already following",
      chosen.centre_x, WIDTH / 2 + (-0.28) * WIDTH / 2, 1.0)

# The first sighting has nothing to be near, so size is the only evidence
# there is, and nearer is the better guess.
fresh = track()
first = fresh.update([ball(-0.5, fill=0.20), ball(+0.5, fill=0.45)],
                     (WIDTH, HEIGHT), 0.05)
close("with no history it takes the nearest, judged by how big it looks",
      first.centre_x, WIDTH / 2 + 0.5 * WIDTH / 2, 1.0)

# Something appearing a long way from the prediction is not this target,
# and saying nothing beats saying the wrong thing.
jumpy = track(gate=0.3)
jumpy.update([ball(0.0)], (WIDTH, HEIGHT), 0.05)
check("a sighting far from where this one was predicted is not accepted",
      jumpy.update([ball(0.9)], (WIDTH, HEIGHT), 0.05), None)
truthy("and the track survives to see whether the real one comes back",
       jumpy.alive)

# The bug that would have been invisible: the control loop ticks at 20Hz
# and the detector looks at 24fps, so the same Detection object is handed
# over on consecutive ticks. Folding it in twice counts one look as two
# pieces of evidence; over a run of repeats the filter becomes certain of
# a number it has no new reason to believe and stops responding at all.
repeated = track()
sighting = ball(0.25)
repeated.update([sighting], (WIDTH, HEIGHT), 0.05)
after_one = repeated.x.p00
for _ in range(6):
    repeated.update([sighting], (WIDTH, HEIGHT), 0.05)
truthy("the same sighting handed over seven times does not make it seven "
       "times as sure", repeated.x.p00 >= after_one)
check("and it is still counted as one look, not seven", repeated.seen, 1)
truthy("while it still counts as current", repeated.fresh)

# ...but a sighting that is never replaced means the detector has stopped,
# and a car that cannot tell that from a target sitting still is a car
# that will drive at a memory.
stuck = track(stale_after=0.3)
held = ball(0.1)
for _ in range(3):
    stuck.update([held], (WIDTH, HEIGHT), 0.05)
truthy("a sighting a tenth of a second old is still current", stuck.fresh)
for _ in range(8):
    stuck.update([held], (WIDTH, HEIGHT), 0.05)
falsy("one that has not been replaced in half a second is not", stuck.fresh)

# Giving up, and giving up cleanly.
patient = track(patience=3)
patient.update([ball(0.2)], (WIDTH, HEIGHT), 0.05)
for _ in range(3):
    patient.update([], (WIDTH, HEIGHT), 0.05)
truthy("three empty looks is still worth coasting through", patient.alive)
for _ in range(3):
    patient.update([], (WIDTH, HEIGHT), 0.05)
falsy("six is not", patient.alive)

# A target of a different name is not this target, however obvious it is.
picky = track()
picky.update([Detection("person", 0.9, (10, 10, 100, 200), 0)],
             (WIDTH, HEIGHT), 0.05)
check("something that is not what was asked for is not followed",
      picky.offset, None)

# A frame with no size cannot be turned into a bearing, and inventing one
# puts the target dead centre -- which reads as "straight ahead, go".
blind = track()
check("with no frame there is nothing to be off-centre of",
      blind.update([ball(0.5)], (0, 0), 0.05), None)

print()
print("two bars: strict to start, the prediction to continue")

# A single threshold on how round a blob is cannot work here, and the
# measurements say why. A ball properly in view scores 0.72; the same
# ball half off the edge of the picture scores 0.42, because the clipped
# half of a disc is not a disc; and the darker furniture in this room
# goes up to 0.62. There is nowhere to put one number. Two work, because
# the question is different once something is already being followed.

# What the room actually offers: dark bits that are neither round nor
# large. Measured on the car -- the room's corners reach 0.62 for
# roundness and 0.66% of the picture for size, while the ball manages
# 0.72 and 4.8% in the middle, or 0.42 and 1.9% half out of view.
shy = track()
shy.update([ball(0.3, fill=0.055, confidence=0.50)], (WIDTH, HEIGHT), 0.05)
check("a dark corner is neither round enough nor big enough to chase",
      shy.offset, None)
falsy("and nothing is being followed", shy.alive)

shy.update([ball(0.3, fill=0.055, confidence=0.80)], (WIDTH, HEIGHT), 0.05)
truthy("but something that really looks like the ball is chased, however "
       "small", shy.alive)

# The other half of the pair, and the half that matters. A ball off to
# one side is clipped by the edge of the frame and stops looking round --
# which is exactly when a chase has to start, because off to one side is
# where a target is acquired. Trusting roundness alone here left the car
# reporting "no ball in view" for nine seconds with the ball visible at
# 0.5, taking no steps at all.
bulky = track()
bulky.update([ball(0.65, fill=0.30, confidence=0.48)], (WIDTH, HEIGHT), 0.05)
truthy("a clipped ball at the edge is chased on its size alone",
       bulky.alive)

# The price of that, stated rather than discovered later: something large
# and dark and not at all round will be chased. Roundness is no longer
# able to veto it. Size and the track's own gate are what stand between
# the car and a sofa, and this check exists so that nobody is surprised.
sofa = track()
sofa.update([ball(0.0, fill=0.40, confidence=0.20)], (WIDTH, HEIGHT), 0.05)
truthy("and so, deliberately, is anything else that fills the picture",
       sofa.alive)

# ...and from then on, where it is beats what it looks like. A ball
# wandering off the edge of the picture is exactly when its shape stops
# being recognisable, and exactly when dropping it is least useful.
held = track()
held.update([ball(0.3, confidence=0.80)], (WIDTH, HEIGHT), 0.05)
kept = held.update([ball(0.34, confidence=0.42)], (WIDTH, HEIGHT), 0.05)
check("once following, a poor view in the right place is kept",
      kept is not None, True)
truthy("and the track survives on it", held.alive)

# But position has to beat looks in both directions, or the first
# convincing thing to walk past steals the car.
loyal = track(gate=0.3)
loyal.update([ball(0.0, confidence=0.80)], (WIDTH, HEIGHT), 0.05)
check("a convincing sighting in the wrong place is still refused",
      loyal.update([ball(0.9, confidence=0.95)], (WIDTH, HEIGHT), 0.05), None)

# The strict bar must not apply after a track exists, or a target that
# merely gets harder to see would be dropped and immediately refused
# re-acquisition at the same spot.
reacquired = track()
reacquired.update([ball(-0.2, confidence=0.9)], (WIDTH, HEIGHT), 0.05)
for _ in range(3):
    reacquired.update([], (WIDTH, HEIGHT), 0.05)
back = reacquired.update([ball(-0.22, confidence=0.45)], (WIDTH, HEIGHT), 0.05)
check("a target that reappears looking worse is picked up again",
      back is not None, True)


print()
print("what it reports")

reporting = track()
reporting.update([ball(0.4, confidence=0.83)], (WIDTH, HEIGHT), 0.05)
status = reporting.status()
check("status names the thing being followed", status["label"], "ball")
truthy("and says whether the belief is still worth acting on",
       status["alive"] is True)
truthy("and separates seen-just-now from coasting", status["fresh"] is True)
close("and carries the bearing", status["offset"], 0.4, 0.01)

reporting.forget()
check("forgetting really forgets", reporting.offset, None)
check("including how many times it had been seen", reporting.seen, 0)

print()
if FAILURES:
    print(f"{len(FAILURES)} FAILURE(S): {FAILURES}")
    sys.exit(1)
print("all estimator checks passed")
