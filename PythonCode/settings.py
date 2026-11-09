"""Every number somebody might want to change, in one register.

There were three places a number could live and no rule saying which, so in
practice it lived wherever it was first written. Forty module constants in
patrol.py alone, a handful in a JSON file of preferences, more in a JSON
file of calibration, and nothing anywhere that could list them.

The obvious fix is a big configuration file, and it is the wrong one. The
most valuable thing in this codebase is not the numbers, it is the
paragraphs beside them explaining why each one is what it is -- why the
infrared is trusted at eight centimetres and not three, why a sweep that
hears nothing scores below a wall it can see. Moving the values into a
central file leaves the explanations orphaned next to code that no longer
mentions them, and within a month the two disagree.

So the values stay where they are explained, and what is centralised is the
*register*: a declaration, beside the explanation, that this number exists,
what it means, what units it is in and what range is sensible. One place
can then list them all, change them, persist the changes, and show them on
a page -- without any of them having been moved away from the reason they
hold that value.

    PATROL = settings.group(
        "patrol",
        cruise=Spec(0.8, help="Forward speed while nothing is in the way."),
    )
    ...
    return Velocity(vx=PATROL.cruise)

Three kinds of number, kept apart on purpose:

    settings       choices a human makes, persisted here
    calibration    measurements of this one machine, persisted separately
    plain constant structure rather than taste -- a protocol field count,
                   the resolution of a histogram. Changing one is a code
                   change and it should look like one.

The distinction between the first two is the one that keeps being worth
drawing: a measurement has no defensible default, because a default for a
measured fact is a guess wearing the costume of an answer.
"""

import json
import os
import threading

PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "settings.json"
)

#: The file this replaces. Read once, then left alone.
LEGACY_PATH = os.path.join(os.path.dirname(PATH), "preferences.json")

#: Old flat names to new dotted ones, so an existing car keeps the settings
#: its owner chose rather than silently reverting to defaults on upgrade.
LEGACY_NAMES = {
    "speed_limit": "drive.speed_limit",
    "stop_cm": "safety.stop_cm",
    "slow_cm": "safety.slow_cm",
    "safety_enabled": "safety.enabled",
    "bump_guard": "safety.bump_guard",
    "no_echo_ceiling": "safety.no_echo_ceiling",
    "detect_enabled": "vision.detect_enabled",
    "detect_confidence": "vision.detect_confidence",
    "detect_fps": "vision.detect_fps",
    "flight_recorder": "log.flight_recorder",
    "floor_fitted": "hardware.floor_fitted",
}


class Spec:
    """One tunable: what it defaults to, what it means, what is allowed."""

    def __init__(self, default, help="", unit="", bounds=None, choices=None):
        self.default = default
        self.help = help
        self.unit = unit
        self.bounds = bounds
        self.choices = choices
        self.name = None            # filled in by group()

    @property
    def type(self):
        if isinstance(self.default, bool):
            return "bool"
        if isinstance(self.default, (int, float)):
            return "number"
        return "text"

    def coerce(self, value):
        """The value as the right type and inside its range, or None.

        None means refused, and every caller treats that as "keep what you
        had". Refusing beats clamping for something arriving over HTTP: a
        clamp quietly accepts a request nobody meant, and the sender is
        never told the number it asked for is not the number in force.
        """
        try:
            if isinstance(self.default, bool):
                if isinstance(value, str):
                    return value.strip().lower() in ("1", "true", "yes", "on")
                return bool(value)
            if isinstance(self.default, (int, float)):
                value = float(value)
                if value != value:          # NaN compares false with itself
                    return None
                if self.bounds is not None:
                    low, high = self.bounds
                    if not low <= value <= high:
                        return None
                return value
            value = str(value).strip()
            if self.choices is not None and value.lower() not in self.choices:
                return None
            return value.lower() if self.choices is not None else value
        except (TypeError, ValueError):
            return None

    def as_dict(self, value):
        return {
            "name": self.name,
            "value": value,
            "default": self.default,
            "help": self.help,
            "unit": self.unit,
            "type": self.type,
            "bounds": list(self.bounds) if self.bounds else None,
            "choices": list(self.choices) if self.choices else None,
        }


class Group:
    """A module's tunables, read as attributes.

    `PATROL.cruise` rather than `settings.get("patrol.cruise")`, so a call
    site costs one character more than the constant it replaces. Attribute
    access is what makes the migration from constants worth doing at all:
    anything longer and the pressure to go back to a module constant for
    "just this one" never goes away.
    """

    def __init__(self, registry, prefix):
        object.__setattr__(self, "_registry", registry)
        object.__setattr__(self, "_prefix", prefix)

    def __getattr__(self, key):
        return self._registry.get("%s.%s" % (self._prefix, key))

    def __setattr__(self, key, value):
        raise AttributeError(
            "settings are changed through settings.set(), so that the change "
            "is validated and written down; assigning to %s.%s would last "
            "until the next restart and tell nobody" % (self._prefix, key))


class Registry:
    def __init__(self, path=PATH):
        self.path = path
        self.lock = threading.RLock()
        self.specs = {}
        self.values = {}
        self._loaded = False

    # -- declaring ---------------------------------------------------------

    def group(self, prefix, **specs):
        """Declare a module's tunables and get an accessor for them."""
        with self.lock:
            for key, spec in specs.items():
                name = "%s.%s" % (prefix, key)
                if name in self.specs:
                    raise ValueError("setting %r declared twice" % name)
                spec.name = name
                self.specs[name] = spec
            self._apply_pending()
        return Group(self, prefix)

    # -- reading and writing ----------------------------------------------

    def get(self, name):
        with self.lock:
            if name in self.values:
                return self.values[name]
            spec = self.specs.get(name)
            if spec is None:
                raise KeyError("no setting called %r" % name)
            return spec.default

    def set(self, name, value):
        """Change one setting and write it down. Returns it, or None."""
        with self.lock:
            spec = self.specs.get(name)
            if spec is None:
                return None
            coerced = spec.coerce(value)
            if coerced is None:
                return None
            if self.get(name) == coerced:
                return coerced          # nothing changed, nothing written
            self.values[name] = coerced
            self.save()
        return coerced

    def update(self, changes):
        """Several at once, one write. Returns the names actually changed."""
        done = []
        with self.lock:
            for name, raw in changes.items():
                spec = self.specs.get(name)
                if spec is None:
                    continue
                coerced = spec.coerce(raw)
                if coerced is None or self.get(name) == coerced:
                    continue
                self.values[name] = coerced
                done.append(name)
            if done:
                self.save()
        return done

    def reset(self, name):
        with self.lock:
            if self.values.pop(name, None) is not None:
                self.save()
            return self.get(name)

    # -- listing -----------------------------------------------------------

    def describe(self):
        """Everything, grouped, for the page that shows them."""
        out = {}
        with self.lock:
            for name, spec in sorted(self.specs.items()):
                prefix = name.split(".", 1)[0]
                out.setdefault(prefix, []).append(spec.as_dict(self.get(name)))
        return out

    def changed(self):
        """Only what differs from the defaults: what to read first when a
        car is behaving unlike the code says it should."""
        with self.lock:
            return {name: value for name, value in sorted(self.values.items())
                    if name in self.specs and value != self.specs[name].default}

    # -- persistence -------------------------------------------------------

    def load(self):
        stored = self._read(self.path)
        if stored is None:
            stored = self._migrate()
        self._pending = dict(stored or {})
        self._loaded = True
        with self.lock:
            self._apply_pending()

    def _read(self, path):
        try:
            with open(path, encoding="utf-8") as handle:
                stored = json.load(handle)
        except (OSError, ValueError):
            return None
        return stored if isinstance(stored, dict) else None

    def _migrate(self):
        """Carry over the file this replaces, once."""
        legacy = self._read(LEGACY_PATH)
        if not legacy:
            return {}
        carried = {}
        for old, new in LEGACY_NAMES.items():
            if old in legacy:
                carried[new] = legacy[old]
        if carried:
            print("[settings] carried %d setting(s) over from preferences.json"
                  % len(carried), flush=True)
        return carried

    def _apply_pending(self):
        """Take stored values for specs that exist now.

        Declaration happens at import time, module by module, and a stored
        file is read before most of them have run -- so values wait here
        until their spec turns up. A value whose spec never appears stays in
        the file untouched, which is what should happen to a setting
        belonging to a module somebody has temporarily removed.
        """
        pending = getattr(self, "_pending", None)
        if not pending:
            return
        for name in list(pending):
            spec = self.specs.get(name)
            if spec is None:
                continue
            coerced = spec.coerce(pending.pop(name))
            if coerced is None:
                print("[settings] ignoring stored %s, outside its range" % name,
                      flush=True)
                continue
            self.values[name] = coerced

    def save(self):
        with self.lock:
            snapshot = dict(self.values)
            snapshot.update(getattr(self, "_pending", {}) or {})
        temporary = self.path + ".tmp"
        try:
            with open(temporary, "w", encoding="utf-8") as handle:
                json.dump(snapshot, handle, indent=2, sort_keys=True)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, self.path)
        except OSError as exc:
            print("[settings] could not save: %s" % exc, flush=True)


#: One per process. Modules declare into it at import time.
settings = Registry()
settings.load()


def group(prefix, **specs):
    return settings.group(prefix, **specs)
