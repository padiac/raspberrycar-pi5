"""Choices that should survive a reboot.

The sibling of calibration.py, and deliberately not the same file. That one
holds measurements -- which way the servo horn went on, where the wiring sits
in the sensor's view -- and the distinction it draws in its own opening lines
is that a measurement has no sensible default, because a default for a
measured fact is a guess wearing the costume of an answer.

These are the other kind. Every one of them is a choice, every one has a
defensible default, and a fresh car with none of this stored still behaves
correctly. What they have in common with calibration is only the failure they
share: a value the human set from the web page lived in memory, and the next
restart quietly put it back, so the car came up driving at a speed nobody
chose and with object detection off after an evening of using it.

Written through the same atomic replace, for the same reason.
"""

import json
import os
import threading

PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "preferences.json"
)

DEFAULTS = {
    #: Ceiling on commanded speed, 0 to 1. The gamepad trigger scales within
    #: it, so this is the top of the range rather than a fixed speed.
    "speed_limit": 0.40,
    #: Forward motion is refused closer than this. The stopping distance is
    #: what sets it, not taste, which is why it is not the knob to turn when
    #: the car feels nannyish.
    "stop_cm": 20.0,
    #: Forward motion is throttled between stop_cm and here.
    #:
    #: This is the knob to turn, and it starts well below the 80cm the code
    #: used to use. In an apartment there is almost always something within
    #: 80cm of the bow, so an 80cm ramp was not an occasional intervention:
    #: it was a permanent one, the car spent its whole life throttled, and
    #: holding the trigger down did nothing. Forty leaves most of a room at
    #: full speed and still gives twenty centimetres of ramp before the stop.
    "slow_cm": 40.0,
    #: Whether the reflex layer is active at all. Stored because turning it
    #: off is a decision a human makes knowingly, and having it come back on
    #: after a restart while somebody is mid-experiment is its own hazard.
    "safety_enabled": True,
    #: Speed allowed when the ultrasonic hears nothing back. One means no
    #: limit: a timeout is taken as open road. The default keeps the car at
    #: a crawl, because a pane of glass returns a timeout and so does a
    #: curtain -- see the comment in control/safety.py.
    "no_echo_ceiling": 0.22,
    #: What a jolt from the accelerometer is allowed to do.
    #:
    #: "off"    -- nothing, it is only ever displayed
    #: "auto"   -- block forward travel briefly, but only when a behaviour is
    #:             driving. A human can see what they hit; an autonomous
    #:             behaviour cannot, and will happily keep pushing.
    #: "always" -- block it for the human too
    "bump_guard": "auto",
    #: Object detection, and the confidence below which a box is ignored.
    "detect_enabled": False,
    "detect_confidence": 0.4,
    #: How often the detector is allowed to look, in frames per second.
    #:
    #: Not free and not about smoothness: the preview is a separate 30fps
    #: stream and stays that way whatever this is. What this buys is how far
    #: the car travels between looks, and what it costs is CPU taken from
    #: the same four cores that encode the video. Measured on this Pi 5:
    #: 10fps leaves the machine at about a quarter loaded, 24fps at roughly
    #: three quarters, with the preview dipping while it ramps and settling
    #: back near 30. No thermal throttling at either.
    "detect_fps": 10.0,
    #: Whether the flight recorder starts with the car.
    "flight_recorder": False,
}

#: Values that have to be inside these bounds to be accepted. A stored file is
#: editable by hand, and a hand-edited speed limit of 50 should not become a
#: car that ignores its own governor.
BOUNDS = {
    "speed_limit": (0.0, 1.0),
    "stop_cm": (0.0, 200.0),
    "slow_cm": (0.0, 400.0),
    "detect_confidence": (0.05, 0.95),
    "detect_fps": (0.0, 60.0),
    "no_echo_ceiling": (0.0, 1.0),
}

CHOICES = {
    "bump_guard": ("off", "auto", "always"),
}


def coerce(key, value):
    """The stored value as the right type and inside its bounds, or None.

    None means "not usable", and every caller treats that as "keep what you
    had". Rejecting is better than clamping for a value that arrived over
    HTTP, because a clamp silently accepts a request nobody meant.
    """
    if key not in DEFAULTS:
        return None
    default = DEFAULTS[key]
    try:
        if isinstance(default, bool):
            if isinstance(value, str):
                value = value.strip().lower() in ("1", "true", "yes", "on")
            return bool(value)
        if isinstance(default, float):
            value = float(value)
            low, high = BOUNDS.get(key, (float("-inf"), float("inf")))
            return value if low <= value <= high else None
        if isinstance(default, str):
            value = str(value).strip().lower()
            allowed = CHOICES.get(key)
            return value if (allowed is None or value in allowed) else None
    except (TypeError, ValueError):
        return None
    return None


class Preferences:
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
        if not isinstance(stored, dict):
            return
        with self.lock:
            for key in DEFAULTS:
                if key not in stored:
                    continue
                value = coerce(key, stored[key])
                if value is None:
                    print("[prefs] ignoring stored %s=%r, out of range"
                          % (key, stored[key]), flush=True)
                    continue
                self.values[key] = value

    def get(self, key):
        with self.lock:
            return self.values.get(key, DEFAULTS.get(key))

    def set(self, key, value):
        """Store one preference. Returns what was stored, or None if refused."""
        value = coerce(key, value)
        if value is None:
            return None
        with self.lock:
            if self.values.get(key) == value:
                return value            # no write, nothing changed
            self.values[key] = value
            self.save()
        return value

    def update(self, changes):
        """Several at once, one write. Returns the keys actually stored."""
        done = []
        with self.lock:
            for key, raw in changes.items():
                value = coerce(key, raw)
                if value is None or self.values.get(key) == value:
                    continue
                self.values[key] = value
                done.append(key)
            if done:
                self.save()
        return done

    def save(self):
        with self.lock:
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
            print(f"[prefs] could not save: {exc}", flush=True)

    def as_dict(self):
        with self.lock:
            return dict(self.values)
