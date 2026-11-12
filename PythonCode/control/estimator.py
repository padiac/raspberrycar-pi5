"""What the car believes about a thing it is following, between looks.

The layer that was missing. Before this, `behaviors/follow.py` read the
detector's newest box and fed it straight to a PID:

    detector -> PID -> wheels

which has two faults, and the second is the one that made the car wobble.

The first is that there was no identity. Every frame picked the largest box
of the right label, from scratch, with no memory -- so two balls on the
floor meant the car followed whichever happened to measure bigger this
frame, swapping between them without anything in the code noticing. That is
not tracking, it is repeated detection wearing the word.

The second is that a PID's derivative term differentiates whatever it is
given, and dividing by a 0.05s tick multiplies whatever noise rides on it
by twenty before the gain is applied.

How much that second one mattered is worth stating carefully, because the
first draft of this comment overstated it from an assumed noise level and
the measurement disagreed. Measured on this camera, across seventy
consecutive live frames of a motionless ball, the detector's box wobbles by
0.0005 of half a frame width. Through the old differencing and the old
effective gain of six, that is about 0.086 of turn authority manufactured
out of nothing: real, visible, worth removing, and about a tenth of what
the steering was allowed to command -- not the several times over that was
claimed before anybody measured it.

The larger half of "it wobbles" was very probably the *first* fault, in a
nastier form than swapping between targets. At the confidence gate it was
set to, the detector found this ball in two looks out of forty, so the
behaviour was stop, stop, stop, one full-strength command, stop. An
intermittent controller looks far livelier than a merely noisy one.

Both are fixed, by different parts. `perception/blob.py` stops the
dropouts; this stops the amplification.

So: a constant-velocity Kalman filter per quantity, and an association step
in front of it.

The filter is worth the forty lines over a moving average for one reason
that matters here. An average gives a smoothed *position* and nothing else,
so a derivative still has to be taken by subtraction. This carries velocity
as part of its state and estimates it from the whole history, which means
the follow loop can ask for the rate directly and never differentiate a
noisy signal at all. Smoothing was the lesser half of the problem.

It also keeps running when the detector blinks. `predict()` advances the
belief with no measurement, so a target hidden behind a chair leg for three
frames is still somewhere, and the car keeps turning towards where it will
be rather than stopping dead and starting again.

What this is not: it models one target moving smoothly in the image. It
does not know about occlusion, it does not reason about several targets at
once beyond picking the right one, and it will happily follow a different
ball that passes close to where it expected this one. Those are real
problems with real solutions and none of them are here, which is a
statement about scope rather than an oversight.
"""

from settings import Spec, group

from perception.types import Detection     # noqa: F401 - documents the input

#: Variances are kept as standard deviations in the settings and squared
#: here, because a standard deviation is in the same unit as the thing it
#: describes and a variance is in that unit squared. "The box jitters by
#: about half a percent of the frame" is a sentence somebody can check with
#: their eyes; "the measurement variance is 1.6e-05" is not.
#:
#: Where the two defaults come from, since a number in a filter that nobody
#: measured is the thing this file exists to stop:
#:
#: `jitter` was first guessed at 0.02 and then measured on this camera, on
#: seventy consecutive live frames of a ball that was not moving, by
#: differencing consecutive sightings so that only the wobble is left:
#: 0.0005 for the detector's box and 0.0013 for the ball finder's circle.
#: The default is several times that, because every one of those frames was
#: taken with the car parked. Driving adds motion blur, a changing viewing
#: angle and a changing light, and none of that has been measured yet. It
#: is the number most worth re-measuring once the car is driving again, and
#: `tools/` has the script that did it.
#:
#: `manoeuvre` was chosen by sweeping it against a simulated ball that sits
#: still and then crosses the frame, at the jitter above, and reading three
#: numbers off: how noisy the rate is while the target is still, how far
#: behind it falls once the target moves, and how much quieter the position
#: is than the raw measurement. Lag is negligible anywhere above 0.6. Rate
#: noise grows steadily with it. Position noise is worse than the raw
#: measurement below 1.0. So: 1.0, the point where nothing is yet being
#: given up. Rate noise there is 0.030, which the steering gain of 0.25
#: turns into 0.0075 of turn authority out of a motionless ball, against
#: 0.086 for the old code's differenced measurement.
#: How far past the edge of the picture a belief about where the target is
#: may go: just beyond it, meaning "off that side", and no further.
EDGE = 1.1

#: A box this close to the bottom of the frame (as a fraction of its
#: height) is taken to be cut off by it.
BOTTOM_MARGIN = 0.02

BELIEF = group(
    "tracking",
    jitter=Spec(
        0.004, bounds=(0.0005, 0.5),
        help="How much the detector's idea of where the target is wobbles "
             "when the target is still, as a fraction of half the frame "
             "width. Larger means trust each look less and the picture "
             "more: smoother, but slower to notice a real move."),
    manoeuvre=Spec(
        1.0, bounds=(0.01, 100.0),
        help="How sharply the target can change speed, in half-frame-widths "
             "per second per second. Larger means believe a sudden move "
             "sooner, at the cost of believing jitter sooner too."),
    gate=Spec(
        0.45, bounds=(0.02, 2.0),
        help="How far from where it was predicted a detection may be and "
             "still count as the same target. This is what stops the car "
             "swapping to a second ball that appears across the room."),
    patience=Spec(
        12, bounds=(1, 200),
        help="Looks without a sighting before the target is given up for "
             "good. Until then the belief coasts on its own momentum."),
    stale_after=Spec(
        0.5, unit="s", bounds=(0.05, 10.0),
        help="How long the same sighting may keep being handed over before "
             "it stops counting as current. The control loop ticks faster "
             "than the detector looks, so seeing one sighting twice is "
             "normal; seeing it for half a second means the detector has "
             "stopped looking."),
    confident=Spec(
        0.65, bounds=(0.0, 1.0),
        help="How sure a sighting must look before it is worth starting "
             "to follow. One of two ways to qualify; see sure_size. Only "
             "applies to the first sighting -- after that, being where "
             "the target was predicted to be is far better evidence than "
             "how the thing looks. Measured on this car: a ball in the "
             "middle of the picture scores 0.72 and nothing else in the "
             "room rises above 0.62."),
    sure_size=Spec(
        0.012, bounds=(0.0, 1.0),
        help="How much of the picture a sighting must fill to be worth "
             "starting to follow regardless of its shape. The other way "
             "to qualify, and the one that matters: a ball off to one "
             "side is clipped by the edge of the frame and stops looking "
             "round, which is exactly when a chase has to start. "
             "Measured here: the ball fills 1.9% even when half out of "
             "view, and the room's dark corners never reached 0.7%."),
    doubt=Spec(
        0.30, bounds=(0.01, 2.0),
        help="How uncertain the position may get, in half-frame-widths, "
             "before coasting stops counting as knowing where it is."),
)


class Scalar:
    """One quantity that moves smoothly, estimated from noisy looks.

    A two-state Kalman filter -- value and rate -- written out in longhand
    rather than with matrices. Two-by-two algebra spelled out is shorter
    than the code to build the matrices, has no dependency, and can be read
    by somebody who has not met the matrix form.
    """

    def __init__(self, jitter=None, manoeuvre=None):
        #: Measurement variance: how wrong one look is, squared.
        self.r = (BELIEF.jitter if jitter is None else jitter) ** 2
        #: Process variance: how wrong "it keeps doing what it was doing"
        #: is, squared, per second of acceleration.
        self.q = (BELIEF.manoeuvre if manoeuvre is None else manoeuvre) ** 2

        self.value = None
        self.rate = 0.0
        #: Covariance of (value, rate). Symmetric, so three numbers.
        self.p00 = 0.0
        self.p01 = 0.0
        self.p11 = 0.0

    @property
    def known(self):
        return self.value is not None

    @property
    def doubt(self):
        """Standard deviation of the value, in the value's own unit."""
        return self.p00 ** 0.5 if self.known else float("inf")

    def forget(self):
        self.value = None
        self.rate = 0.0
        self.p00 = self.p01 = self.p11 = 0.0

    def predict(self, dt):
        """Advance the belief by dt seconds with nothing new to go on."""
        if not self.known or dt <= 0:
            return
        self.value += self.rate * dt

        # P = F P F' + Q, with F = [[1, dt], [0, 1]] and the usual
        # constant-acceleration Q. Written out; the ordering matters because
        # p00 depends on the old p01 and p11.
        p00, p01, p11 = self.p00, self.p01, self.p11
        self.p00 = p00 + dt * (2.0 * p01 + dt * p11) + self.q * dt ** 4 / 4.0
        self.p01 = p01 + dt * p11 + self.q * dt ** 3 / 2.0
        self.p11 = p11 + self.q * dt ** 2

    def correct(self, measured):
        """Fold in one look."""
        if measured is None:
            return
        if not self.known:
            # First sighting: take it at face value and admit to knowing
            # nothing about the rate. A large initial rate variance is what
            # lets the next two or three looks establish it quickly.
            self.value = float(measured)
            self.rate = 0.0
            self.p00 = self.r
            self.p01 = 0.0
            self.p11 = self.q
            return

        innovation = measured - self.value
        s = self.p00 + self.r
        k0 = self.p00 / s
        k1 = self.p01 / s

        self.value += k0 * innovation
        self.rate += k1 * innovation

        # P = (I - K H) P with H = [1, 0]. Symmetry survives this because
        # k1 * p00 and k0 * p01 are the same number; see the test.
        p00, p01 = self.p00, self.p01
        self.p00 = (1.0 - k0) * p00
        self.p01 = (1.0 - k0) * p01
        self.p11 = self.p11 - k1 * p01

    def status(self):
        return {
            "value": None if self.value is None else round(self.value, 4),
            "rate": round(self.rate, 4),
            "doubt": None if not self.known else round(self.doubt, 4),
        }


class TargetTrack:
    """One followed object, held together across frames.

    Three quantities, filtered independently: where it is left-to-right,
    where it is up-and-down, and how big it looks. They are independent
    because nothing here couples them -- a ball moving left does not thereby
    get bigger -- and pretending otherwise would mean a six-state filter
    whose cross terms are all zero.

    Offsets are in half-frame-widths: 0 is dead centre, -1 the left edge,
    +1 the right edge. A fraction rather than pixels so that changing the
    preview size does not retune the whole control loop.
    """

    def __init__(self, label, jitter=None, manoeuvre=None,
                 gate=None, patience=None, doubt=None, stale_after=None,
                 confident=None, sure_size=None):
        self.label = label
        self.x = Scalar(jitter, manoeuvre)
        self.y = Scalar(jitter, manoeuvre)
        #: Apparent size varies far more slowly and far less noisily than
        #: position, and nothing steers off it, so it gets the same filter
        #: rather than its own tuning.
        self.size = Scalar(jitter, manoeuvre)
        #: The last sighting touched the bottom of the picture.
        self.below = False

        self._gate = gate
        self._patience = patience
        self._doubt = doubt
        self._stale_after = stale_after
        self._confident = confident
        self._sure_size = sure_size

        #: The exact sighting last folded in, and how long ago that was.
        #:
        #: The control loop ticks faster than the detector looks -- twenty
        #: times a second against twenty-four -- so the same `Detection`
        #: object is handed over on consecutive ticks. Correcting on it
        #: twice would count one look as two pieces of evidence, shrinking
        #: the covariance for nothing and leaving a filter that is certain
        #: of a number it has no new reason to believe. Over a run of
        #: repeats it stops responding to real movement at all.
        #:
        #: Identity, not equality: the pipeline replaces the whole list
        #: when it has looked again, and the frozen `Detection`s inside it
        #: are rebuilt with it, so `is` separates "looked again and saw the
        #: same thing" from "has not looked again".
        self._folded = None
        self._since_new = 0.0

        #: Looks since the last sighting. Zero means seen this frame.
        self.missed = 0
        #: Sightings since the track began, so a one-frame false positive
        #: can be told from something that has been there a while.
        self.seen = 0
        self.confidence = 0.0
        self.box = None

    # -- the three settings, read live unless overridden for a test --------

    @property
    def gate(self):
        return BELIEF.gate if self._gate is None else self._gate

    @property
    def patience(self):
        return BELIEF.patience if self._patience is None else self._patience

    @property
    def doubt_limit(self):
        return BELIEF.doubt if self._doubt is None else self._doubt

    @property
    def confident(self):
        return BELIEF.confident if self._confident is None else self._confident

    @property
    def sure_size(self):
        return BELIEF.sure_size if self._sure_size is None else self._sure_size

    def _convincing(self, detection, frame_size):
        """Worth starting a chase on, with nothing to compare it against.

        Either it looks like the thing, or there is a great deal of it.
        Both tests are weak on their own and they are weak in different
        places, which is the only reason this works: a ball at the edge
        of the picture is not round, and a ball across the room is not
        large, but a ball is never both at once in a way that matters.
        """
        if detection.confidence >= self.confident:
            return True
        width, height = frame_size
        picture = float(width) * float(height)
        if picture <= 0:
            return False
        return (detection.area / picture) >= self.sure_size

    @property
    def stale_after(self):
        return (BELIEF.stale_after if self._stale_after is None
                else self._stale_after)

    # -- what the follower asks --------------------------------------------

    @property
    def alive(self):
        """Still worth following: seen recently enough, and not so uncertain
        that 'where it is' has stopped meaning anything."""
        return (self.x.known and self.missed <= self.patience
                and self.x.doubt <= self.doubt_limit)

    @property
    def offset(self):
        """Where it is, left to right. None when there is no belief."""
        return self.x.value

    @property
    def drift(self):
        """How fast it is crossing the frame, per second.

        This is the number the steering loop uses for damping. It comes out
        of the filter's state rather than from subtracting two measurements,
        which is the whole point of the filter being here.
        """
        return self.x.rate

    @property
    def fresh(self):
        """Seen this look, as opposed to coasting on the prediction.

        Nothing known is not fresh. The counter alone says otherwise --
        `forget()` sets it to zero, and a track that has never seen
        anything has missed nothing -- which read as "seen just now" in
        the telemetry of a car that was looking at an empty room.
        """
        return self.x.known and self.missed == 0

    # -- the loop -----------------------------------------------------------

    def predict(self, dt):
        self.x.predict(dt)
        self.y.predict(dt)
        self.size.predict(dt)
        # A prediction is only worth anything inside the picture. Most of the
        # rate a track carries when it loses its target is the car's own
        # turning, which stops when the step does -- but the filter cannot
        # know that, and carried on extrapolating. Measured on the car: four
        # missed looks took a ball at +0.8 to an offset of -2.7, thirty-four
        # degrees away, and the next step turned there. Off the edge is all
        # that can be said about something that has gone off the edge, so
        # the belief stops there and stops moving.
        for scalar in (self.x, self.y):
            if scalar.known and abs(scalar.value) > EDGE:
                scalar.value = EDGE if scalar.value > 0 else -EDGE
                scalar.rate = 0.0
        if self.size.known and self.size.value < 0.0:
            # Seen on the car as a fill of -0.27. Nothing has negative size.
            self.size.value = 0.0
            self.size.rate = 0.0

    def choose(self, detections, frame_size):
        """Which of these, if any, is the thing being followed.

        Nearest to where it was predicted, not largest. Largest is what the
        old code used and it is the reason two similar balls could swap
        without a word: size is a property of this frame alone, while
        nearness to the prediction is the only evidence available that this
        is the same object as last time.

        With no belief yet there is nothing to be near, so the largest wins
        -- which for a ball on the floor means the closest, and is the right
        first guess.
        """
        width, height = frame_size
        if not width or not height:
            return None

        mine = [d for d in detections
                if d.label.lower() == self.label.lower()]
        if not mine:
            return None

        if not self.x.known:
            # Nothing is being followed yet, so there is no prediction to
            # judge a sighting against and the sighting has to stand on
            # its own. This is the only place the finder's own confidence
            # carries any weight, and the only place it has to: once
            # there is a track, where a thing is beats what it looks
            # like, every time.
            sure = [d for d in mine if self._convincing(d, frame_size)]
            if not sure:
                return None
            return max(sure, key=lambda d: d.area)

        best, best_gap = None, None
        for det in mine:
            gap = abs(self._offset_of(det, width) - self.x.value)
            if best_gap is None or gap < best_gap:
                best, best_gap = det, gap

        # Too far from the prediction to be the same thing. Saying nothing
        # is better than saying the wrong thing: the track coasts, and if
        # this really is where the target went, the prediction's growing
        # uncertainty will let it back in within a frame or two.
        if best_gap > self.gate:
            return None
        return best

    def update(self, detections, frame_size, dt):
        """One tick: predict, then fold in whichever detection is ours.

        Called once per control tick, which is more often than the detector
        produces anything new. Prediction happens every time; correction
        only when there is something that has not been counted yet.
        """
        self.predict(dt)
        self._since_new += max(0.0, dt)

        chosen = self.choose(detections, frame_size)
        if chosen is None:
            self.missed += 1
            self.confidence = 0.0
            self._folded = None
            return None

        if chosen is self._folded:
            # The same sighting again: the loop has ticked, the detector has
            # not looked. Already predicted above, which is the whole of
            # what is owed. Still counts as current until it goes stale --
            # and going stale is how a detector thread that has stopped
            # producing anything is noticed, since from here that looks
            # exactly like one sighting that never gets replaced.
            if self._since_new > self.stale_after:
                self.missed += 1
                self.confidence = 0.0
            return chosen

        width, height = frame_size
        self._folded = chosen
        self._since_new = 0.0
        self.missed = 0
        self.seen += 1
        self.confidence = chosen.confidence
        self.box = chosen.box

        self.x.correct(self._offset_of(chosen, width))
        self.y.correct((chosen.centre_y - height / 2.0) / (height / 2.0))
        # Cut off by the bottom of the picture: the box stops growing at the
        # frame edge while the thing keeps getting closer, so its height
        # says smaller just as it gets nearer. Measured on the car with the
        # ball: 0.39 of the frame at 50cm, already touching the edge, and
        # falling from there. The width is not cut by the bottom edge, and
        # for anything about as wide as it is tall it is the better size.
        # For anything taller than wide, max() leaves the height in charge.
        self.below = chosen.box[3] >= height * (1.0 - BOTTOM_MARGIN)
        tall = chosen.height
        if self.below:
            tall = max(tall, chosen.width)
        self.size.correct(tall / float(height))
        return chosen

    @staticmethod
    def _offset_of(detection, width):
        return (detection.centre_x - width / 2.0) / (width / 2.0)

    def forget(self):
        self.below = False
        self.x.forget()
        self.y.forget()
        self.size.forget()
        self.missed = 0
        self.seen = 0
        self.confidence = 0.0
        self.box = None
        self._folded = None
        self._since_new = 0.0

    def status(self):
        return {
            "label": self.label,
            "alive": self.alive,
            "fresh": self.fresh,
            "missed": self.missed,
            "seen": self.seen,
            "confidence": round(self.confidence, 2),
            "offset": None if self.offset is None else round(self.offset, 3),
            "drift": round(self.drift, 3),
            "size": self.size.status()["value"],
            "doubt": None if not self.x.known else round(self.x.doubt, 3),
        }
