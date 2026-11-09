#!/usr/bin/env python3
"""Checks for the three things added after an evening of chasing ghosts.

Each one here failed in the real car first, which is the only reason any of
them is worth a test:

  * the servo moved on its own, because the ESP32 restarts by itself and the
    firmware centres the head in setup() -- so a reboot has to be noticed and
    the head told again where it was pointing;
  * the link went permanently deaf after forty kilobytes of boot-ROM noise
    arrived with no newline in it, because nothing bounded the buffer;
  * settings the human chose came back as defaults after every restart, and
    the logs that would have explained the restart did not survive it.

Run directly: python3 tests/test_housekeeping.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import hal.link as link_module
from control.safety import SafetyLayer, BUMP_FREEZE_S
from diagnostics.logbook import Logbook
from hal import Velocity
from perception.types import WorldState
from preferences import Preferences, coerce

PASSES = []
FAILS = []


def check(label, condition):
    (PASSES if condition else FAILS).append(label)
    print("  %-6s %s" % ("PASS" if condition else "FAIL", label))


def section(title):
    print("\n" + title)
    print("-" * len(title))


# -- a world with nothing in it, so each test only sets what it is about ---

def clear_world(**kw):
    kw.setdefault("range_cm", 300.0)
    kw.setdefault("range_expected", True)
    return WorldState(**kw)


# =========================================================================
section("a jolt blocks forward travel, and only forward")

layer = SafetyLayer(bump_guard="always")
allowed = layer.filter(Velocity(0.5, 0.0, 0.0), clear_world(bumped=True))
check("hitting something stops forward travel", allowed.vx == 0.0)
check("and says so", any("hit" in r for r in layer.last.interventions))

layer = SafetyLayer(bump_guard="always")
allowed = layer.filter(Velocity(-0.5, 0.0, 0.0), clear_world(bumped=True))
check("reversing off it is still allowed", allowed.vx == -0.5)

layer = SafetyLayer(bump_guard="always")
allowed = layer.filter(Velocity(0.0, 0.0, 0.6), clear_world(bumped=True))
check("turning away from it is still allowed", allowed.omega == 0.6)

# The firmware holds the flag for 400ms; this layer holds its own window, so
# the block has to outlive the flag or it does nothing at 20Hz.
layer = SafetyLayer(bump_guard="always")
layer.filter(Velocity(0.5, 0.0, 0.0), clear_world(bumped=True))
allowed = layer.filter(Velocity(0.5, 0.0, 0.0), clear_world(bumped=False))
check("the block outlives the flag that set it", allowed.vx == 0.0)
check("and is reported as active", layer.bump_active)

layer._bumped_at = time.monotonic() - (BUMP_FREEZE_S + 0.05)
allowed = layer.filter(Velocity(0.5, 0.0, 0.0), clear_world(bumped=False))
check("but lets go after its window", allowed.vx == 0.5)

section("who is driving decides whether a jolt freezes the controls")

layer = SafetyLayer(bump_guard="auto")
layer.human_driving = True
allowed = layer.filter(Velocity(0.5, 0.0, 0.0), clear_world(bumped=True))
check("a human keeps their controls", allowed.vx == 0.5)

layer = SafetyLayer(bump_guard="auto")
layer.human_driving = False
allowed = layer.filter(Velocity(0.5, 0.0, 0.0), clear_world(bumped=True))
check("a behaviour does not", allowed.vx == 0.0)

layer = SafetyLayer(bump_guard="off")
allowed = layer.filter(Velocity(0.5, 0.0, 0.0), clear_world(bumped=True))
check("off means off", allowed.vx == 0.5)

# A jolt must not be able to override the rule this layer is built on, that
# it only ever removes motion.
layer = SafetyLayer(bump_guard="always")
allowed = layer.filter(Velocity(0.0, 0.0, 0.0), clear_world(bumped=True))
check("it never adds motion of its own",
      (allowed.vx, allowed.vy, allowed.omega) == (0.0, 0.0, 0.0))


# =========================================================================
section("preferences survive, and refuse nonsense")

path = "/tmp/car_prefs_test.json"
if os.path.exists(path):
    os.remove(path)
prefs = Preferences(path)
check("a fresh file gives defaults", prefs.get("speed_limit") == 0.40)
check("storing returns what was stored", prefs.set("speed_limit", 0.75) == 0.75)
check("reloading gets it back", Preferences(path).get("speed_limit") == 0.75)

check("a speed above full is refused", prefs.set("speed_limit", 5) is None)
check("and the old value stands", prefs.get("speed_limit") == 0.75)
check("a word where a number goes is refused",
      prefs.set("slow_cm", "soon") is None)
check("an unknown guard is refused", prefs.set("bump_guard", "maybe") is None)
check("a known one is taken, any case", prefs.set("bump_guard", "ALWAYS") == "always")
check("booleans take a checkbox string", coerce("detect_enabled", "true") is True)
check("unknown keys are refused", coerce("fly_to_moon", 1) is None)

# A hand-edited file is the case this guards: a stored value out of range must
# not become the robot's behaviour just because somebody typed it.
with open(path, "w", encoding="utf-8") as handle:
    handle.write('{"speed_limit": 99, "slow_cm": 55}')
reloaded = Preferences(path)
check("a hand-edited bad value falls back to the default",
      reloaded.get("speed_limit") == 0.40)
check("while its good neighbour is still read", reloaded.get("slow_cm") == 55.0)
os.remove(path)


# =========================================================================
section("the logbook cannot outgrow its ceiling")

directory = "/tmp/car_logs_test"
os.system("rm -rf " + directory)
book = Logbook(directory)
book.install()
for i in range(400):
    print("filler line %d, long enough to make the file grow %s" % (i, "x" * 200))
sizes = {f["name"]: f["bytes"] for f in book.files()}
check("it is writing a file", "car.log" in sizes)
check("nothing exceeds the per-file limit",
      all(v <= 2 * 1024 * 1024 + 4096 for v in sizes.values()))

# Rotation is the bound; the sweep is the second line. Give it something the
# handlers did not put there and check it goes, while the live file stays.
stray = os.path.join(directory, "car.log.99")
with open(stray, "w", encoding="utf-8") as handle:
    handle.write("x" * 1024)
os.utime(stray, (0, 0))         # older than any age limit
book.sweep()
check("the sweep removes a stale file", not os.path.exists(stray))
check("and never the file being written",
      os.path.exists(os.path.join(directory, "car.log")))

rows = []
book.record(lambda: {"n": len(rows), "flag": True} if rows.append(1) is None else {})
time.sleep(0.9)
book.stop_recording()
flight = os.path.join(directory, "flight.csv")
check("the flight recorder wrote a file", os.path.exists(flight))
with open(flight, encoding="utf-8") as handle:
    lines = [line for line in handle.read().splitlines() if line]
check("with a header and some rows", len(lines) >= 2 and lines[0].startswith("time,"))

tail = book.tail(3)
check("the tail reads back the last lines", len(tail) == 3)
check("and refuses to leave its directory", book.tail(3, "../calibration.json") == [])

# Standing still should be nearly free: with an unchanging source the only
# rows written are the once-a-second heartbeat.
book2 = Logbook(directory)
book2.record(lambda: {"n": 1})
time.sleep(1.2)
book2.stop_recording()
check("an unchanging robot writes only a heartbeat", book2._flight_rows <= 4)

status = book.status()
check("status reports a ceiling", status["ceiling"] > 0)
check("and what is used", status["bytes"] > 0)

sys.stdout = sys.__stdout__
sys.stderr = sys.__stderr__
os.system("rm -rf " + directory)


# =========================================================================
section("the link bounds what it holds and notices a reboot")

check("there is a cap on the partial buffer", link_module.MAX_PARTIAL <= 4096)
check("and it keeps less than it caps",
      link_module.KEEP_PARTIAL < link_module.MAX_PARTIAL)


class FakeLink:
    """Just enough SerialLink to exercise _handle and _board_booted."""

    def __init__(self):
        self.telemetry = link_module.Telemetry()
        self.board_reboots = 0
        self.garbage = 0
        self.on_reconnect = None
        self._last_hard = 0.0
        self._last_garbage_note = 0.0

    _handle = link_module.SerialLink._handle
    _board_booted = link_module.SerialLink._board_booted


def framed(body):
    return "$%s*%02X" % (body, link_module.checksum(body))


seen = []
fake = FakeLink()
fake._last_hard = -1000.0            # long ago, so a banner is a surprise
fake.on_reconnect = lambda hard: seen.append(hard)
fake._handle(framed("!,bridge ready"))
check("an unexpected banner counts as a reboot", fake.board_reboots == 1)
check("and asks the owner to restore state", seen == [True])

fake = FakeLink()
fake._last_hard = time.monotonic()   # we just reset it ourselves
fake.on_reconnect = lambda hard: None
fake._handle(framed("!,bridge ready"))
check("a banner we asked for is not counted", fake.board_reboots == 0)

fake = FakeLink()
fake.on_reconnect = lambda hard: (_ for _ in ()).throw(RuntimeError("boom"))
fake._last_hard = -1000.0
fake._handle(framed("!,bridge ready"))
check("a hook that throws does not take the reader down", fake.board_reboots == 1)

fake = FakeLink()
fake._handle(framed("!,command timeout, stopped"))
check("an ordinary message is not a reboot", fake.board_reboots == 0)
check("but is kept to show", fake.telemetry.messages[-1].startswith("command timeout"))


# =========================================================================
section("the recorder slows down when nothing is happening")

import diagnostics.logbook as lb

quiet = Logbook("/tmp/car_logs_quiet")
quiet._flight_last = {"n": 1}
quiet._flight_wrote = time.monotonic() - 2.0
quiet._flight_changed = time.monotonic()         # something moved just now
check("a busy car gets a one-second heartbeat",
      quiet._worth_writing({"n": 1}, time.monotonic()))

quiet._flight_wrote = time.monotonic() - 2.0
quiet._flight_changed = time.monotonic() - 120.0  # parked for two minutes
check("a parked car does not",
      not quiet._worth_writing({"n": 1}, time.monotonic()))

quiet._flight_wrote = time.monotonic() - (lb.FLIGHT_IDLE_KEEPALIVE_S + 1)
check("but still gets a pulse eventually",
      quiet._worth_writing({"n": 1}, time.monotonic()))

quiet._flight_changed = time.monotonic() - 120.0
quiet._flight_wrote = time.monotonic()
check("and wakes up the moment something moves",
      quiet._worth_writing({"n": 9}, time.monotonic()))
os.system("rm -rf /tmp/car_logs_quiet")


section("recordings are bounded as a collection, not one at a time")

from perception.clip import ClipRecorder, CLIP_KEEP

shed = "/tmp/car_clips_test"
os.system("rm -rf " + shed)
os.makedirs(shed)
for i in range(CLIP_KEEP + 5):
    with open(os.path.join(shed, "clip-%03d.mp4" % i), "wb") as handle:
        handle.write(b"x" * 2048)
# Constructing it sweeps once, which is the case that matters on a card an
# older version filled up.
recorder = ClipRecorder(None, shed)
left = [c["name"] for c in recorder.clips()]
check("the count is enforced at startup", len(left) == CLIP_KEEP)
check("the oldest went first", "clip-000.mp4" not in left)
check("the newest stayed", "clip-%03d.mp4" % (CLIP_KEEP + 4) in left)

# And again on demand, which is what runs when a recording finishes.
for i in range(200, 206):
    with open(os.path.join(shed, "clip-%03d.mp4" % i), "wb") as handle:
        handle.write(b"x" * 2048)
removed = recorder.prune()
check("a later sweep removes what the new files pushed over",
      len(removed) == 6)
check("and takes them in order, oldest first",
      removed == sorted(removed))
check("leaving exactly the budget", len(recorder.clips()) == CLIP_KEEP)

# The file being written has no index yet and is the one most likely wanted.
recorder.path = os.path.join(shed, "clip-%03d.mp4" % (CLIP_KEEP + 4))
for i in range(100, 150):
    with open(os.path.join(shed, "clip-%03d.mp4" % i), "wb") as handle:
        handle.write(b"x" * 2048)
recorder.prune()
check("a clip still recording is never deleted",
      os.path.exists(recorder.path))
os.system("rm -rf " + shed)


# =========================================================================
print("\n" + "=" * 62)
if FAILS:
    print("FAILED %d of %d:" % (len(FAILS), len(FAILS) + len(PASSES)))
    for name in FAILS:
        print("  - " + name)
    sys.exit(1)
print("all %d housekeeping checks passed" % len(PASSES))
