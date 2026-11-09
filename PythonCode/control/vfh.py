"""Choosing a direction from a sweep, the way VFH+ does it.

Borenstein and Koren wrote the Vector Field Histogram to fix a specific
failure of potential fields, and the failure is worth stating because this
car had its own version of it. A potential field adds every obstacle into one
repulsive vector, and a sum destroys the thing you most need to know: a solid
wall and two obstacles with a passable gap between them produce nearly the
same resultant, because the two sides of the gap cancel. The robot either
refuses a route it could take or dithers in front of it.

VFH keeps the distribution instead of summing it. Build one number per
direction, then look for valleys -- runs of directions that are free -- and
pick between the valleys rather than between individual rays.

What this module adds over what the sweep used to do:

**Width.** The old code scored each angle by the smallest clearance among
itself and its two neighbours, which is a width test by proxy and does not
know how wide the car is. Here the car's width enters as geometry: an
obstacle at distance d is grown by the half-width of the chassis plus a
margin, which blocks an arc of half-angle asin(R/d) around it. The car is
then a point, and a valley that survives is one the car fits through. This is
the fix for aiming at the gap between a chair's castors: the gap is real, it
is just narrower than the robot, and nothing in the old scoring could say so.

**Distance weighting, for free.** VFH weights near obstacles more heavily
through a term in the histogram. Here it falls out of the geometry instead --
asin(R/d) grows as d shrinks, so a close obstacle blocks a wide arc and a
distant one barely blocks anything. Nothing needs tuning.

**Hysteresis.** Two directions that score within noise of each other made the
old chooser flip a coin every time it was asked. Each choice was defensible
and the sequence was a car twitching between them. The cost function carries
a term for distance from the previous choice, which is what VFH+ does and is
the cheapest fix for oscillation there is.

The one assumption to keep in mind: VFH expects a *scan* -- every direction
measured at the same moment, from a ring of sonars or a lidar. A single
rangefinder on a servo gives readings a second or more apart, taken from
slightly different places. That is why `samples` is whatever the caller has
been able to collect, old readings included, and why nothing here tries to be
more precise than the input deserves.
"""

import math

#: Angular resolution of the working histogram, in degrees.
#:
#: Finer than the sweep samples on purpose. The samples are coarse because the
#: head takes time to move; the blocked arcs computed from them are not, and
#: rounding a 23-degree arc to the nearest 45-degree sample would throw away
#: exactly the information this module exists to use.
FINE_STEP = 5.0

#: Beyond this a reading is "as far as it matters" and not an obstacle.
REACH_CM = 400.0

#: How far ahead a direction is judged over.
#:
#: Something three metres away does not stop the car turning towards it: it
#: would stop long before arriving, and by then it will have looked again.
#: Treating every reading as a hard obstacle whatever its distance made an
#: empty room -- three metres in every direction -- come back as a row of
#: narrow blocked arcs with slivers of valley between them. VFH avoids this
#: by weighting a far obstacle down to almost nothing in the histogram; a
#: mask has no weights, so it needs a horizon instead.
#:
#: About one move's worth of travel. Readings beyond it still count towards
#: how roomy a direction is, they just stop blocking it.
CONCERN_CM = 100.0

#: Cost weights. The first must dominate, or the car stops caring about room;
#: the other two only break ties between directions that are both open.
#:
#: mu_room    how much the clearance in that direction matters
#: mu_ahead   a preference for not turning further than necessary
#: mu_keep    a preference for the direction chosen last time
#:
#: VFH+ asks that mu_room > mu_ahead + mu_keep, so no amount of agreement
#: between the two tie-breakers can outvote the actual obstacle data.
MU_ROOM = 5.0
MU_AHEAD = 2.0
MU_KEEP = 2.0
assert MU_ROOM > MU_AHEAD + MU_KEEP


def fine_angles(span):
    """Every direction the histogram is built over, left to right."""
    low, high = span
    steps = int(round((high - low) / FINE_STEP))
    return [low + i * FINE_STEP for i in range(steps + 1)]


def blocked_arc(distance_cm, radius_cm):
    """Half-angle, in degrees, that an obstacle at this distance blocks.

    This is the whole of VFH+'s obstacle enlargement: grow every obstacle by
    the radius the robot needs, and the robot becomes a point. At distance d
    a circle of radius R subtends asin(R/d), and once d is inside R there is
    no angle at which the car clears it, which is what the clamp to 90 says.
    """
    if distance_cm <= 0.0:
        return 90.0
    ratio = radius_cm / distance_cm
    if ratio >= 1.0:
        return 90.0
    return math.degrees(math.asin(ratio))


def histogram(samples, radius_cm, span=(-90.0, 90.0), reach=REACH_CM,
              concern=CONCERN_CM):
    """Build the polar histogram from measured directions.

    `samples` maps angle in degrees (positive to the right, matching the pan
    servo and omega) to a clearance in centimetres. Returns a list of
    (angle, free, room) -- room being the clearance of the nearest sample by
    angle, which is the honest reading of a sweep that measured five
    directions and is being asked about thirty-seven.
    """
    out = []
    if not samples:
        return out
    measured = sorted(samples)
    arcs = []
    for angle in measured:
        distance = min(float(samples[angle]), reach)
        if distance >= concern:
            continue            # too far ahead to be in the way of a turn
        arcs.append((angle, blocked_arc(distance, radius_cm)))

    for fine in fine_angles(span):
        nearest = min(measured, key=lambda a: abs(a - fine))
        room = min(float(samples[nearest]), reach)
        free = all(abs(fine - angle) > arc for angle, arc in arcs)
        out.append((fine, free, room))
    return out


def valleys(bins):
    """Runs of consecutive free directions, as (start, end) in degrees."""
    found = []
    start = None
    previous = None
    for angle, free, _room in bins:
        if free and start is None:
            start = angle
        elif not free and start is not None:
            found.append((start, previous))
            start = None
        if free:
            previous = angle
    if start is not None:
        found.append((start, previous))
    return found


def choose(samples, radius_cm, previous=None, span=(-90.0, 90.0),
           reach=REACH_CM, forbid=(), concern=CONCERN_CM):
    """Pick a direction to turn towards, or None if nothing fits.

    Returns (angle, trace) where trace is a short string for the log, because
    a decision nobody can read afterwards is a decision nobody can debug.
    """
    bins = histogram(samples, radius_cm, span, reach, concern)
    if not bins:
        return None, "no readings"
    open_valleys = valleys(bins)
    if not open_valleys:
        return None, "every direction is inside an obstacle once the car is "\
                     "grown to its own width"

    room_at = {angle: room for angle, _free, room in bins}

    # Inside a wide valley, aim for the point nearest the direction the car
    # would rather go, but no closer to either edge than the car's own width
    # needs. That is VFH+'s rule for hugging the near edge of a doorway
    # instead of ceremonially driving down its centre line, and it falls out
    # of the same arithmetic.
    best = None
    for low, high in open_valleys:
        room = max(room_at.get(a, 0.0)
                   for a in fine_angles((low, high)) if a in room_at) \
            if high >= low else 0.0
        margin = blocked_arc(max(room, 1.0), radius_cm)
        inner_low, inner_high = low + margin, high - margin
        if inner_low > inner_high:
            # The valley is exactly as wide as the car needs, or narrower
            # than the margin allows: its middle is the only candidate.
            inner_low = inner_high = (low + high) / 2.0
        for angle in fine_angles((span[0], span[1])):
            if angle < inner_low or angle > inner_high:
                continue
            if any(abs(angle - bad) < 1e-6 for bad in forbid):
                continue
            here = room_at.get(angle, 0.0)
            cost = (MU_ROOM * (1.0 - min(here, reach) / reach)
                    + MU_AHEAD * abs(angle) / 180.0
                    + MU_KEEP * (abs(angle - previous) / 180.0
                                 if previous is not None else 0.0))
            if best is None or cost < best[0]:
                best = (cost, angle, here, (low, high))

    if best is None:
        return None, "valleys exist but every direction in them is excluded"

    cost, angle, room, (low, high) = best
    trace = ("%+.0f (%.0fcm, valley %+.0f..%+.0f of %d, cost %.2f)"
             % (angle, room, low, high, len(open_valleys), cost))
    return angle, trace
