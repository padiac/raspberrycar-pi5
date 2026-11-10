"""Finding a ball by colour and shape, on pictures made up here.

No camera: every frame below is drawn with numpy, so these run on a desk
and cannot fight the car. No test reads the live register either -- every
finder is built with explicit settings -- because a test that changed
`ball.val_max` would change what the car does on its next tick.
"""

import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from perception.blob import BallFinder, Sight, _mask   # noqa: E402

FAILURES = []
WIDTH, HEIGHT = 640, 360

#: A finder set up for the case the car actually has: something very dark
#: on something pale. Spelled out so that nothing here depends on the
#: shipped defaults staying what they are today.
DARK = dict(
    enabled=True, label="ball",
    hue_centre=0.0, hue_width=90.0,
    sat_min=0.0, sat_max=255.0,
    val_min=0.0, val_max=70.0,
    min_area=400.0, roundness=0.72,
)


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


def floor(brightness=200):
    """A pale, empty floor."""
    return np.full((HEIGHT, WIDTH, 3), brightness, dtype=np.uint8)


print("a dark round thing on a pale floor")

frame = floor()
cv2.circle(frame, (300, 200), 60, (20, 20, 20), -1)
found = BallFinder(**DARK).detect(frame)
check("one ball is one sighting", len(found), 1)
close("and it is where it was put, left to right", found[0].centre_x, 300, 3)
close("and top to bottom", found[0].centre_y, 200, 3)
close("and about the size it was drawn", found[0].width / 2.0, 60, 4)
truthy("a filled circle scores near the top of the roundness scale",
       found[0].confidence > 0.85)
check("and it is labelled what the settings asked for",
      found[0].label, "ball")

# The sofa base. Long, dark, and right behind the ball on the real floor,
# so this is the discrimination that matters rather than a contrived one.
frame = floor()
cv2.rectangle(frame, (40, 30), (600, 90), (20, 20, 20), -1)
check("a long dark thing is not a ball, however dark",
      len(BallFinder(**DARK).detect(frame)), 0)

frame = floor()
cv2.rectangle(frame, (40, 30), (600, 90), (20, 20, 20), -1)
cv2.circle(frame, (300, 250), 55, (20, 20, 20), -1)
found = BallFinder(**DARK).detect(frame)
check("with both in frame, only the round one is reported", len(found), 1)
close("and it is the ball, not the furniture", found[0].centre_y, 250, 4)

# Specks of grain in a wooden floor are dark and roughly round. Reporting
# them is how the car ends up chasing a knot in a floorboard.
frame = floor()
for x in range(60, 600, 37):
    cv2.circle(frame, (x, 300), 4, (15, 15, 15), -1)
check("floor speckle is too small to be a ball",
      len(BallFinder(**DARK).detect(frame)), 0)

frame = floor()
check("an empty floor is reported as empty, not as a guess",
      len(BallFinder(**DARK).detect(frame)), 0)

off = dict(DARK)
off["enabled"] = False
frame = floor()
cv2.circle(frame, (300, 200), 60, (20, 20, 20), -1)
check("switched off, it finds nothing rather than quietly running",
      len(BallFinder(**off).detect(frame)), 0)

check("and a frame that never arrived is not an error",
      len(BallFinder(**DARK).detect(None)), 0)

# Two balls: both are reported, and the caller decides. Deciding here
# would hide from `TargetTrack` the very evidence it needs to keep hold
# of the one it was already following.
frame = floor()
cv2.circle(frame, (180, 200), 40, (20, 20, 20), -1)
cv2.circle(frame, (460, 200), 60, (20, 20, 20), -1)
found = BallFinder(**DARK).detect(frame)
check("two balls are two sightings, not a choice made early", len(found), 2)
close("reported biggest first", found[0].centre_x, 460, 4)


print()
print("colour, which is a circle and wraps")

# Red sits at the seam of the hue circle, so a band around it runs off one
# end and back on at the other. One inRange cannot say that, and the
# version that tries asks for everything between 170 and 10 and matches
# nothing at all.
hsv = np.zeros((10, 30, 3), dtype=np.uint8)
hsv[:, :10] = (2, 200, 200)        # just above the seam
hsv[:, 10:20] = (178, 200, 200)    # just below it
hsv[:, 20:] = (90, 200, 200)       # the opposite colour
wrapped = _mask(hsv, 0.0, 10.0, (50, 255), (50, 255))
check("a band at the seam catches the hues just above it",
      int((wrapped[:, :10] > 0).all()), 1)
check("and the ones just below, on the far side of the wrap",
      int((wrapped[:, 10:20] > 0).all()), 1)
check("and not the opposite colour", int((wrapped[:, 20:] > 0).any()), 0)

plain = _mask(hsv, 90.0, 10.0, (50, 255), (50, 255))
check("a band that does not wrap still works the ordinary way",
      int((plain[:, 20:] > 0).all()), 1)

everything = _mask(hsv, 0.0, 90.0, (0, 255), (0, 255))
check("asking for every hue gets every hue",
      int((everything > 0).all()), 1)


print()
print("learning the colour by being shown one")

finder = BallFinder(**DARK)
frame = floor()
cv2.circle(frame, (300, 200), 70, (40, 180, 60), -1)      # a green ball
learned = finder.sample(frame, (230, 130, 370, 270))
truthy("being shown a coloured ball produces a colour window", learned)
truthy("which is a narrow band rather than every hue",
       learned["hue_width"] < 45.0)
# OpenCV hue for this green is about 65 of 180.
close("centred on the colour it was shown", learned["hue_centre"], 65, 12)

frame = floor()
cv2.circle(frame, (300, 200), 70, (20, 20, 20), -1)
learned = finder.sample(frame, (230, 130, 370, 270))
close("being shown a black ball gives up on hue entirely",
      learned["hue_width"], 90.0, 0.1)
truthy("and describes it by brightness instead", learned["val_max"] < 90)

check("a frame that never arrived teaches nothing",
      finder.sample(None, (0, 0, 10, 10)), None)
check("and neither does a box with nothing in it",
      finder.sample(floor(), (10, 10, 12, 12)), None)


print()
print("several ways of looking, behind one")


class Fake:
    label = "fake"

    def __init__(self, result, explode=False, settable=("knob",)):
        self.result = result
        self.explode = explode
        self.SETTABLE = tuple(settable)
        self.given = {}

    def detect(self, frame):
        if self.explode:
            raise RuntimeError("this finder is broken")
        return list(self.result)

    def configure(self, **kwargs):
        self.given.update(kwargs)
        return {}

    def status(self):
        return {"fake": True}


from perception.types import Detection                  # noqa: E402

one = Detection("ball", 0.9, (0, 0, 10, 10), 200)
two = Detection("person", 0.8, (20, 20, 40, 60), 0)

sight = Sight(Fake([one]), Fake([two]))
check("what two finders see is one list", len(sight.detect(None)), 2)

said = []
broken = Sight(Fake([one]), Fake([], explode=True), Fake([two]),
               announce=said.append)
check("a finder that throws does not take the others down with it",
      len(broken.detect(None)), 2)
check("and it is said out loud rather than swallowed", len(said), 1)
broken.detect(None)
check("but not repeated once a frame for ever", len(said), 1)

mine = Fake([one], settable=("knob",))
yours = Fake([two], settable=("dial",))
Sight(mine, yours).configure(knob=7, dial=3, nobodys=1)
check("a setting reaches the finder that declared it", mine.given, {"knob": 7})
check("and its neighbour gets only its own", yours.given, {"dial": 3})

# The real pairing: the ball finder must not be handed the detector's
# confidence gate, which would create an override shadowing the register
# for the rest of the run.
ball_finder = BallFinder(**DARK)
Sight(ball_finder).configure(confidence=0.9, roundness=0.5)
check("the ball finder takes the setting that is its own",
      ball_finder._get("roundness"), 0.5)
check("and refuses one meant for its neighbour",
      "confidence" in ball_finder._overrides, False)

names = Sight(BallFinder(**DARK)).class_names
check("it can say what it is able to find", names, ["ball"])

print()
if FAILURES:
    print(f"{len(FAILURES)} FAILURE(S): {FAILURES}")
    sys.exit(1)
print("all ball finder checks passed")
