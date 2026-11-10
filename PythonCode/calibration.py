"""Numbers measured from this particular car.

Not settings. A setting is a choice and can sensibly have a default; these are
facts about one assembled machine, and a default for them is a guess that
looks like an answer. Which servo angle points down the middle of the chassis
depends on which spline tooth the horn went onto, and no amount of reasoning
recovers it -- somebody has to aim the thing at a wall and watch the number.

So they are measured once and kept. The alternative, which is what this file
replaces, is that the measurement lives in memory and the next reboot quietly
throws it away, leaving the robot confidently acting on a value nobody chose.

Written atomically, because the thing being protected against is losing a
measurement, and a half-written file loses it just as thoroughly as no file.
"""

import json
import os
import threading

PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "calibration.json"
)

#: Zero here means "not yet measured", which for a trim is also the sensible
#: thing to do in the meantime -- assume the horn went on straight, and be
#: wrong by however much it did not.
DEFAULTS = {
    "pan_trim": 0.0,
    #: Whether the head turns the opposite way to the commanded sign. Depends
    #: on how the servo linkage was assembled, so it cannot be assumed.
    "pan_reversed": False,
    #: True when the board's heading counts the opposite way round to the
    #: drivetrain, so that turning right makes it go down. Found the hard
    #: way: the search steers on this, and with the sign crossed it became
    #: positive feedback and turned the car most of the way round while
    #: the ball it was looking for was dead centre in the picture.
    #:
    #: Which way up an accelerometer ended up on the chassis is exactly
    #: the sort of thing that differs between two cars built from the same
    #: parts, which is why it is a measurement and not a constant.
    "heading_reversed": False,
    #: The nearest thing each pan angle can see when the room is empty, keyed
    #: by angle as a string. Not a setting: it is a measurement of what parts
    #: of this car are in the sensor's way, which depends entirely on where
    #: the wiring was routed and changes whenever that does.
    "pan_blind": {},
    #: How wide the chassis is, in centimetres, across the widest part --
    #: which on this car is the mecanum wheels, not the plate they are bolted
    #: to. A measurement, and the one the direction chooser turns on: every
    #: obstacle is grown by half of this plus a margin, and a gap that
    #: survives is one the car fits through. Guessed from the half-diagonal
    #: used for the turning check until somebody puts a ruler across it.
    "car_width_cm": 22.0,
    #: And how long, front to back, over the widest parts. Together with the
    #: width this gives the radius the corners sweep through when the car
    #: turns on the spot, which is the one number the rotation check needs
    #: and which used to be a guess.
    "car_length_cm": 28.0,
    #: Extra room to leave either side, on top of the half width. The car
    #: does not drive where it is pointed to the centimetre.
    "car_margin_cm": 4.0,
    #: Duty cycle below which a commanded turn does not turn the car.
    #: Rotating in place fights the rollers on all four mecanum wheels,
    #: so it costs more than driving straight and has its own floor.
    #: Zero means nobody has measured it, and nothing will be said.
    "turn_floor": 0.0,
    #: Whether the side infrared is actually connected. False means the
    #: pins are sitting at their pull-ups, and a pull-up reads exactly
    #: like a clear road -- so until somebody has watched the numbers
    #: change, the safety layer is not told about them.
    "ir_fitted": False,
}


class Calibration:
    def __init__(self, path=PATH):
        self.path = path
        self.lock = threading.RLock()
        self.values = dict(DEFAULTS)
        self.load()

    def load(self):
        try:
            with open(self.path, encoding="utf-8") as handle:
                stored = json.load(handle)
        except (OSError, ValueError):
            return          # absent or unreadable: the defaults stand
        if isinstance(stored, dict):
            with self.lock:
                for key in DEFAULTS:
                    if key in stored:
                        self.values[key] = stored[key]

    def get(self, key):
        with self.lock:
            return self.values.get(key, DEFAULTS.get(key))

    def set(self, key, value):
        with self.lock:
            self.values[key] = value
            snapshot = dict(self.values)
        temporary = self.path + ".tmp"
        try:
            with open(temporary, "w", encoding="utf-8") as handle:
                json.dump(snapshot, handle, indent=2, sort_keys=True)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, self.path)
        except OSError as exc:
            print(f"[calibration] could not save: {exc}", flush=True)
        return value

    def status(self):
        with self.lock:
            return dict(self.values)
