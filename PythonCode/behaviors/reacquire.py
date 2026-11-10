"""Where to look when the thing you were following has gone.

Reported from the room: the car was following the ball, the car turned,
the ball went out of frame, and the car then sat there for ever. Which is
exactly what the code said to do -- no target, no velocity -- and exactly
the wrong thing, because the car is the reason the ball left.

That last part is what makes this tractable without any new hardware or
any new measurement. A target can leave the frame two ways. It can move,
in which case nobody knows where it went. Or the camera can turn away from
it, in which case **the car knows precisely how far it turned**, because
the gyro has been integrating all along.

So the search is anchored, not blind:

    anchor = the car's heading at the moment of the last sighting

Turning back to the anchor must put the camera where it was when it could
see the target. If the target has not moved, that alone finds it, and it
finds it exactly -- no sweeping, no luck, and no need to know how many
degrees wide the picture is. Only when turning back is not enough, which
means the target moved too, does this widen into a sweep, and even then
the sweep is centred on the one bearing that is known to be interesting
rather than on wherever the car happens to be pointing.

The sweep leans towards the side the target was last seen on and was
heading for. A ball last seen at the right-hand edge, still moving right,
is not equally likely to be in either direction.

Not here, and worth saying so. There is no estimate of how many degrees
off the nose the target is, because that needs the camera's horizontal
field of view, and this camera's is not known -- the two common lenses for
this sensor are 62 and 160 degrees, which is a factor of two and a half,
and a guess in the middle would be wrong for both. That measurement would
let the search go straight to the target's bearing instead of returning to
an orientation; it is a real improvement and it is not pretended to here.
"""

import math

from settings import Spec, group

#: How far round the car is willing to look for something it has lost,
#: in degrees either side of where it last saw it.
#:
#: Not a full circle, deliberately. A car that turns all the way round
#: looking for a ball has stopped doing what it was asked and started
#: doing something that is very hard to watch and judge. Three quarters
#: of a right angle each way covers "it rolled out of shot" and refuses
#: "it is behind me now", which is a different problem.
SWEEP_DEGREES = 70.0

HUNT = group(
    "search",
    enabled=Spec(
        True,
        help="Look around for a target that has gone, instead of stopping "
             "where it was lost."),
    patience_s=Spec(
        6.0, unit="s", bounds=(0.5, 60.0),
        help="How long to keep looking before giving up and stopping."),
    sweep_degrees=Spec(
        SWEEP_DEGREES, unit="deg", bounds=(5.0, 180.0),
        help="How far either side of where it was last seen the car will "
             "look. The sweep opens out to this; it does not start there."),
    dwell_s=Spec(
        0.4, unit="s", bounds=(0.0, 5.0),
        help="How long to hold still on the heading the target was last "
             "seen from before widening the search. The detector needs "
             "several frames to find anything, so a sweep that starts "
             "moving the instant it arrives has not actually looked."),
    opening_rate=Spec(
        45.0, unit="deg/s", bounds=(2.0, 360.0),
        help="How fast the search widens. Slower looks harder close to "
             "where the target was before giving up on that idea."),
    sweep_period_s=Spec(
        2.4, unit="s", bounds=(0.4, 20.0),
        help="How long one swing of the sweep takes. Too fast and the "
             "detector never gets a steady look at anything."),
    effort=Spec(
        0.45, unit="", bounds=(0.05, 1.0),
        help="Hardest the car will turn while searching. Lower than "
             "chasing, because a fast sweep blurs every frame it is "
             "supposed to be searching."),
    gain=Spec(
        0.02, unit="per deg", bounds=(0.001, 0.5),
        help="How hard it turns per degree it is away from where it wants "
             "to be looking."),
    blind_rate=Spec(
        0.30, unit="", bounds=(0.05, 1.0),
        help="How fast to sweep when there is no gyro to close the loop "
             "with, so the car cannot tell how far it has turned."),
)


class Reacquire:
    """A search for something that was being followed a moment ago.

    Owned by the behaviour that lost the target, reset the moment it is
    found again. Holds no opinion about driving: searching is a thing done
    with the camera, and a car that drives towards a target it cannot see
    is doing something nobody asked for.
    """

    def __init__(self, patience_s=None, sweep_degrees=None, dwell_s=None,
                 opening_rate=None, sweep_period_s=None, effort=None,
                 gain=None, blind_rate=None, enabled=None):
        #: None means "read the register", so the car follows the settings
        #: page while a test can pin every number it depends on. A test
        #: that moved the register would change what the car does on its
        #: next tick, which is the one thing the conventions forbid here.
        self._settings = {
            "patience_s": patience_s, "sweep_degrees": sweep_degrees,
            "dwell_s": dwell_s,
            "opening_rate": opening_rate, "sweep_period_s": sweep_period_s,
            "effort": effort, "gain": gain, "blind_rate": blind_rate,
            "enabled": enabled,
        }
        self.searching = False
        self.anchor = 0.0           # heading when the target was last seen
        self.side = 1.0             # which way it went: +1 right, -1 left
        self.elapsed = 0.0
        self.aim = 0.0              # heading currently being turned towards
        self.blind = False          # no gyro, so no closing the loop

    def _get(self, name):
        chosen = self._settings.get(name)
        return getattr(HUNT, name) if chosen is None else chosen

    # -- what the behaviour tells it ---------------------------------------

    def found(self):
        """The target is back. Nothing to remember."""
        self.searching = False
        self.elapsed = 0.0

    def lost(self, last_offset, last_drift, turned, have_gyro=True):
        """Start searching. Call once, at the moment the track dies."""
        if self.searching:
            return
        self.searching = True
        self.elapsed = 0.0
        self.anchor = turned if turned is not None else 0.0
        self.blind = not have_gyro
        self.aim = self.anchor

        # Where it went. Its position carries most of the information --
        # a target lost from the right-hand edge went right -- and its
        # velocity breaks the tie for one lost near the middle, which
        # happens when the detector drops it rather than when it leaves.
        lean = (last_offset or 0.0) + 0.3 * (last_drift or 0.0)
        self.side = 1.0 if lean >= 0 else -1.0

    # -- what the behaviour asks -------------------------------------------

    def steer(self, turned, dt):
        """How hard to turn, or None when it is time to give up.

        None rather than zero, because "stop turning" and "stop searching"
        are different instructions and the caller has to tell its owner
        which one happened.
        """
        if not self.searching:
            return None
        if not self._get("enabled"):
            return None

        self.elapsed += max(0.0, dt or 0.0)
        if self.elapsed > self._get("patience_s"):
            self.searching = False
            return None

        effort = self._get("effort")

        if self.blind or turned is None:
            # No gyro: there is no way to know how far the car has turned,
            # so there is nothing to close a loop around. Sweep open-loop,
            # alternating, and accept that it is a worse search rather than
            # pretending the heading is known. Reversing on the period is
            # the only part of the plan that survives.
            swing = math.cos(math.pi * self.elapsed / self._get("sweep_period_s"))
            return max(-effort, min(effort,
                                    self.side * swing * self._get("blind_rate")))

        # The first thing the car does is come back to the heading it last
        # saw the target from, and then hold there. If the target has not
        # moved, that is the whole search, and it is exact rather than
        # lucky -- it needs nothing from the lens and nothing from chance.
        #
        # The hold matters as much as the heading. Finding something takes
        # the detector several frames, and a sweep that starts widening the
        # instant it arrives has swung past before the first of them came
        # back. Only after the dwell does the clock that opens the search
        # start running.
        looking = max(0.0, self.elapsed - self._get("dwell_s"))
        amplitude = min(self._get("sweep_degrees"),
                        self._get("opening_rate") * looking)
        swing = math.cos(math.pi * looking / self._get("sweep_period_s"))
        self.aim = self.anchor + self.side * amplitude * swing

        error = self.aim - turned
        omega = self._get("gain") * error
        return max(-effort, min(effort, omega))

    def describe(self, label):
        if not self.searching:
            return f"lost the {label}"
        if self.blind:
            return ("looking for the %s, no gyro (%.1fs)"
                    % (label, self.elapsed))
        spread = abs(self.aim - self.anchor)
        if spread < 3.0:
            return ("looking back where the %s was (%.1fs)"
                    % (label, self.elapsed))
        return ("searching %s for the %s, %.0f deg out (%.1fs)"
                % ("right" if self.side > 0 else "left", label,
                   spread, self.elapsed))

    def status(self):
        return {
            "searching": self.searching,
            "elapsed": round(self.elapsed, 1),
            "side": "right" if self.side > 0 else "left",
            "anchor": round(self.anchor, 1),
            "aim": round(self.aim, 1),
            "blind": self.blind,
        }
