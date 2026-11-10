"""Finding a ball by what it looks like, rather than by what it is.

YOLO knows eighty kinds of thing and is bad at this one. Measured on this
car, with the ball filling a third of the frame height, dead centre, well
lit: it scored 0.287, and across forty consecutive looks at a ball that was
not moving it scored between 0.23 and 0.47. The old confidence gate was
0.40, so the ball was found in two of those forty looks. From outside that
is a car that recognises the ball sometimes and not others for no visible
reason, which is exactly how it was reported.

The cause is not resolution and not lighting. COCO's "sports ball" is
footballs and tennis balls in sports photographs; a black dimpled massage
ball on a wooden floor is outside what the model was ever shown. Lowering
the gate gets it found, but it is still riding a 0.3 score, and a box whose
confidence moves that much has corners that move too -- which is the noise
that the steering loop then differentiates.

So this does the opposite. It knows one thing and knows it exactly:

    threshold -> close the holes -> contours -> is it round?

The ball is near-black on a pale floor, which is the easiest case this
approach has. Measured on the same frame: roundness 0.82 for the ball,
0.10 for the sofa base behind it, which is rejected; 5.5ms a frame against
the detector's 45ms, so it can run on every frame instead of every other
one; and a centre from `minEnclosingCircle`, which is sub-pixel and does
not jitter the way a box does.

The honest limits, because they are not the ones a neural network has:

  - It finds "a dark round thing", not "the ball". A black shoe or a bowl
    on a pale floor will do. The roundness test is the only thing between
    them and being followed.
  - It is a statement about this ball on this floor. On a dark carpet the
    threshold has nothing to separate, and it will find nothing at all
    rather than find the wrong thing, which is the better of the two
    failures but still a failure.
  - A ball that rolls in front of something equally dark merges with it,
    the blob stops being round, and the sighting is dropped. The track in
    `control/estimator.py` coasts through that; here it is simply missing.

Which is why this does not replace the detector. Both run: the detector for
the eighty things it is good at, this for the one thing it is not.
"""

from settings import Spec, group

import cv2
import numpy as np

from .types import Detection

#: Hue in OpenCV runs 0..179, not 0..359, because it has to fit in a byte.
#: Structure, not a choice: it is what `cv2.COLOR_BGR2HSV` produces.
HUE_CEILING = 180

#: Diameter of the morphological kernel, in pixels at the frame size the
#: pipeline hands over. Big enough to close this ball's dimples -- which
#: are holes of pale highlight in an otherwise black disc and would
#: otherwise split it into a constellation -- and small enough not to weld
#: two nearby objects together. Structure: changing it is a code change,
#: and it is sized to a physical property of the ball's surface.
KERNEL = 9

#: The identifier reported for a blob, kept clear of COCO's 0..79 so that
#: the colour a box is drawn in cannot collide with a detector class.
CLASS_ID = 200

#: Smallest patch, per side, that `sample()` will learn a colour from.
#:
#: Two pixels is enough to compute a percentile and not nearly enough to
#: mean anything. A mis-tap on the video produced a window of val 175 to
#: 225 -- which is the floor, not the ball -- and committing that would
#: have left the car chasing the floorboards with no indication that
#: anything had gone wrong. The smallest ball `min_area` admits is about
#: 22 pixels across, so eight after the inset refuses only taps that were
#: never going to work.
SAMPLE_MIN = 8

SHAPE = group(
    "ball",
    enabled=Spec(
        True,
        help="Look for the ball by colour and shape as well as running the "
             "object detector. Cheap enough to leave on: about five "
             "milliseconds a frame."),
    label=Spec(
        "ball",
        help="What to call what this finds. Behaviours follow a label, so "
             "this is the name you point the follower at."),
    hue_centre=Spec(
        0.0, unit="deg/2", bounds=(0.0, 179.0),
        help="Middle of the band of colour to look for, in OpenCV's hue "
             "units of half a degree. Irrelevant when the target is black "
             "or white, which have no hue to speak of."),
    hue_width=Spec(
        90.0, unit="deg/2", bounds=(1.0, 90.0),
        help="How far either side of that to accept. 90 is every hue there "
             "is, which is the right answer for a black ball."),
    sat_min=Spec(
        0.0, bounds=(0.0, 255.0),
        help="Least colourfulness accepted. Raise it to ignore greys when "
             "chasing something genuinely coloured."),
    sat_max=Spec(255.0, bounds=(0.0, 255.0), help="Most colourfulness accepted."),
    val_min=Spec(
        0.0, bounds=(0.0, 255.0),
        help="Darkest brightness accepted."),
    val_max=Spec(
        70.0, bounds=(0.0, 255.0),
        help="Brightest brightness accepted. For a black ball on a pale "
             "floor this is the setting that does all the work: the floor "
             "sits well above it and the ball well below, so the exact "
             "number is not delicate."),
    min_area=Spec(
        400.0, unit="px", bounds=(20.0, 200000.0),
        help="Smaller blobs than this are speckle, not a ball. Also the "
             "floor on how far away it can be seen."),
    roundness=Spec(
        0.72, bounds=(0.1, 1.0),
        help="Blob area divided by the area of the smallest circle round "
             "it. A perfect disc is 1.0. Measured here: this ball 0.82, "
             "the sofa base behind it 0.10. This is the only thing "
             "separating the ball from any other dark object."),
)


def _mask(hsv, centre, width, sat, val):
    """Pixels inside the colour window, hue wrapping included.

    Hue is a circle, so a band centred near zero runs off one end of the
    range and back on at the other. One `inRange` cannot express that --
    it would ask for everything between 170 and 10 and get nothing -- so a
    wrapping band becomes two windows and an or.
    """
    low_s, high_s = sat
    low_v, high_v = val
    low_h = centre - width
    high_h = centre + width

    if width >= HUE_CEILING / 2.0:
        # Every hue. Skip the arithmetic rather than rely on it landing on
        # a full circle exactly.
        return cv2.inRange(hsv, (0, low_s, low_v), (HUE_CEILING - 1, high_s, high_v))

    if low_h >= 0 and high_h <= HUE_CEILING - 1:
        return cv2.inRange(hsv, (low_h, low_s, low_v), (high_h, high_s, high_v))

    first = cv2.inRange(hsv, (max(0.0, low_h), low_s, low_v),
                        (min(HUE_CEILING - 1.0, high_h), high_s, high_v))
    if low_h < 0:
        second = cv2.inRange(hsv, (HUE_CEILING + low_h, low_s, low_v),
                             (HUE_CEILING - 1, high_s, high_v))
    else:
        second = cv2.inRange(hsv, (0, low_s, low_v),
                             (high_h - HUE_CEILING, high_s, high_v))
    return cv2.bitwise_or(first, second)


class BallFinder:
    """Dark -- or coloured -- round things, as `Detection`s.

    Deliberately the same shape as `perception.detector.Detector`:
    `detect(frame)` returning a list, plus `status()` and `configure()`.
    Anything that can use one can use the other, and `Sight` below runs
    both without either knowing.
    """

    label = "ball by colour and shape"

    #: Which names `configure()` will accept, listed rather than derived.
    #: `Group` forwards every attribute to the register, so asking it what
    #: it contains returns a setting called "names" rather than the list --
    #: and a finder that accepted any keyword would turn a detector setting
    #: aimed at the wrong object into a permanent override shadowing the
    #: register.
    SETTABLE = (
        "enabled", "label", "hue_centre", "hue_width", "sat_min", "sat_max",
        "val_min", "val_max", "min_area", "roundness",
    )

    def __init__(self, **overrides):
        #: Tests pass explicit values; the car reads the live register. Kept
        #: per instance rather than as module state so that a test cannot
        #: change what the car is doing, which the conventions forbid and
        #: which on this machine would mean changing it while it drives.
        self._overrides = dict(overrides)
        self.last_mask = None
        self.rejected = 0

    def _get(self, name):
        if name in self._overrides:
            return self._overrides[name]
        return getattr(SHAPE, name)

    @property
    def class_names(self):
        return [self._get("label")]

    def configure(self, **kwargs):
        for name, value in kwargs.items():
            if value is not None:
                self._overrides[name] = value
        return self.status()

    def status(self):
        return {
            "enabled": bool(self._get("enabled")),
            "label": self._get("label"),
            "roundness": round(float(self._get("roundness")), 2),
            "val_max": round(float(self._get("val_max")), 0),
            "rejected": self.rejected,
        }

    def detect(self, frame):
        if frame is None or not self._get("enabled"):
            return []

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = _mask(
            hsv,
            float(self._get("hue_centre")), float(self._get("hue_width")),
            (float(self._get("sat_min")), float(self._get("sat_max"))),
            (float(self._get("val_min")), float(self._get("val_max"))),
        )

        # Close first, then open. Closing fills the ball's dimples; opening
        # removes the speckle that closing joined up out of floor grain.
        # The other order erases the speckle and then fills the dimples,
        # which also works, but costs the second pass the larger image.
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (KERNEL, KERNEL))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        self.last_mask = mask

        min_area = float(self._get("min_area"))
        wanted = float(self._get("roundness"))
        label = self._get("label")

        found = []
        rejected = 0
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            if radius <= 0:
                continue
            roundness = area / (np.pi * radius * radius)
            if roundness < wanted:
                rejected += 1
                continue
            found.append(Detection(
                label=label,
                # Roundness stands in for confidence, and means the same
                # kind of thing: how much this looks like what was asked
                # for. It is also far steadier than the detector's score,
                # which is the point of the whole exercise.
                confidence=float(min(1.0, roundness)),
                box=(int(cx - radius), int(cy - radius),
                     int(cx + radius), int(cy + radius)),
                class_id=CLASS_ID,
            ))

        self.rejected = rejected
        found.sort(key=lambda d: d.area, reverse=True)
        return found

    def sample(self, frame, box):
        """Learn the colour window from a region the user pointed at.

        Takes the middle half of the box, so that the edge -- where the
        object meets whatever is behind it, and where half the pixels
        belong to the background -- contributes nothing. Returns the
        settings it would like rather than writing them, because what
        happens to a measurement is the caller's business and because a
        function that both measures and commits cannot be tested without
        committing.
        """
        if frame is None:
            return None
        x1, y1, x2, y2 = (int(v) for v in box)
        height, width = frame.shape[:2]
        inset_x = (x2 - x1) // 4
        inset_y = (y2 - y1) // 4
        x1, x2 = max(0, x1 + inset_x), min(width, x2 - inset_x)
        y1, y2 = max(0, y1 + inset_y), min(height, y2 - inset_y)
        if x2 - x1 < SAMPLE_MIN or y2 - y1 < SAMPLE_MIN:
            return None

        patch = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
        hue, sat, val = (patch[:, :, i].astype(np.float32) for i in range(3))

        # Percentiles, not min and max. One stray highlight pixel would
        # otherwise widen the window enough to let the floor in, and a
        # dimpled ball is made of stray highlight pixels.
        low_v, high_v = np.percentile(val, (2, 98))
        low_s, high_s = np.percentile(sat, (2, 98))

        if float(np.median(sat)) < 40.0:
            # Barely coloured: black, white or grey. Hue is meaningless
            # noise at this saturation, so asking for a hue band would
            # throw away most of the object for no reason.
            hue_centre, hue_width = 0.0, 90.0
        else:
            # Hue is circular, so an ordinary mean is wrong across the
            # wrap -- the mean of 179 and 1 is 90, the opposite colour.
            # Average the unit vectors instead.
            angles = hue * (2.0 * np.pi / HUE_CEILING)
            mean = np.arctan2(np.sin(angles).mean(), np.cos(angles).mean())
            hue_centre = float(mean % (2.0 * np.pi)) * HUE_CEILING / (2.0 * np.pi)
            hue_width = 15.0

        return {
            "hue_centre": round(hue_centre, 1),
            "hue_width": round(hue_width, 1),
            "sat_min": round(float(max(0.0, low_s - 25.0)), 1),
            "sat_max": round(float(min(255.0, high_s + 25.0)), 1),
            "val_min": round(float(max(0.0, low_v - 25.0)), 1),
            "val_max": round(float(min(255.0, high_v + 25.0)), 1),
        }

    def close(self):
        self.last_mask = None


class Sight:
    """Every way of finding things, behind one `detect`.

    The pipeline asks one object what it can see and does not care how many
    opinions went into the answer. Adding a third finder later -- a face
    detector, an AprilTag reader -- is a line here and nothing anywhere
    else.

    Errors are per-finder. One that throws is reported and skipped for that
    frame rather than taking the others down with it, because a car that
    stops seeing people because the ball finder met an empty frame is a
    worse failure than either alone.
    """

    def __init__(self, *finders, announce=print):
        self.finders = [f for f in finders if f is not None]
        self.announce = announce
        self._complained = set()

    @property
    def class_names(self):
        names = set()
        for finder in self.finders:
            names.update(getattr(finder, "class_names", []) or [])
        return sorted(names)

    def detect(self, frame):
        found = []
        for finder in self.finders:
            try:
                found.extend(finder.detect(frame))
            except Exception as exc:        # noqa: BLE001 - see the docstring
                key = type(finder).__name__
                if key not in self._complained:
                    self._complained.add(key)
                    self.announce(f"[sight] {key} failed, skipping it: {exc}")
        return found

    def configure(self, **kwargs):
        """Pass settings to whichever finder understands them.

        `confidence` and `classes` belong to the detector and the rest to
        the ball finder, but rather than encode that here, each finder is
        offered everything and keeps what it recognises. A finder that is
        asked about a setting it has never heard of ignores it.
        """
        for finder in self.finders:
            configure = getattr(finder, "configure", None)
            if configure is None:
                continue
            accepted = _settable(finder)
            mine = {k: v for k, v in kwargs.items() if k in accepted}
            if mine:
                configure(**mine)
        return self.status()

    def status(self):
        out = {}
        for finder in self.finders:
            status = getattr(finder, "status", None)
            if status is not None:
                out.update(status())
        out["finders"] = [getattr(f, "label", type(f).__name__)
                          for f in self.finders]
        return out

    def close(self):
        for finder in self.finders:
            closer = getattr(finder, "close", None)
            if closer is not None:
                try:
                    closer()
                except Exception as exc:    # noqa: BLE001 - shutting down
                    self.announce(f"[sight] closing {finder}: {exc}")


def _settable(finder):
    """Which keyword arguments this finder's `configure()` accepts.

    A finder may say so itself with a `SETTABLE` tuple; otherwise the
    names are read off the signature. A `**kwargs` signature with no
    `SETTABLE` means "I do not say", and the answer is nothing rather than
    everything: handing a finder a setting meant for its neighbour is how
    an override nobody typed ends up shadowing the register for good.
    """
    import inspect

    declared = getattr(finder, "SETTABLE", None)
    if declared is not None:
        return tuple(declared)
    try:
        signature = inspect.signature(finder.configure)
    except (TypeError, ValueError):
        return ()
    return tuple(
        name for name, param in signature.parameters.items()
        if param.kind in (inspect.Parameter.POSITIONAL_OR_KEYWORD,
                          inspect.Parameter.KEYWORD_ONLY)
    )
