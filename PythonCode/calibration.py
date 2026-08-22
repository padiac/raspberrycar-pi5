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
