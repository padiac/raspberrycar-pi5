#!/usr/bin/env python3
"""Patrol for a long stretch and judge honestly whether it went well.

The previous version of this script declared a clean run while the car was
watched driving into a sofa and wedging itself in a corner. Its stuck
detector asked whether the forward reading was a small positive number, and
against something soft the reading is not a number at all -- so it was blind
to precisely the failure that happened. A measurement that cannot see the
thing it is measuring is worse than none, because it comes with a verdict.

So being stuck is no longer defined by distance. It is defined by the room not
changing while the car says it is moving: the sensor is pointed at a room, and
a room that stays exactly the same distance away for seconds on end, while the
wheels are being driven, is a car that is not going anywhere. That works
whether the reading is 4cm, 400cm, or nothing at all.
"""

import json
import sys
import time
import urllib.request

BASE = "http://127.0.0.1:8000"
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0
LIMIT = float(sys.argv[2]) if len(sys.argv) > 2 else 0.30
HEARTBEAT_S = 60.0

#: Commanding motion for this long while the view does not change.
STUCK_S = 5.0
#: How much the reading has to wander to count as changing. The parked car
#: measured 50.5 to 50.6cm over ten seconds, so this is well clear of noise.
STUCK_SPREAD_CM = 6.0
#: A stand-in for "nothing came back", so silence counts as a value that can
#: stay the same rather than as a gap in the record.
SILENT = -1.0


def call(path, body=None):
    if body is None:
        req = urllib.request.Request(BASE + path)
    else:
        req = urllib.request.Request(
            BASE + path, data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=4) as r:
            return json.load(r)
    except Exception as exc:  # noqa: BLE001
        return {"error": str(exc)}


def cm(value):
    if value is None:
        return "  ----"
    if value < 0:
        return " clear"
    return "%6.1f" % value


def kind_of(state):
    if "cornered" in state:
        return "trapped"
    if state.startswith("looking"):
        return "sweep"
    if "most room" in state or "nowhere left" in state:
        return "decision"
    if "turning away" in state:
        return "side"
    if "backing out" in state:
        return "reverse"
    if state.startswith("cruising"):
        return "cruise"
    return "other"


call("/api/speed", {"limit": LIMIT})
call("/api/pan", {"angle": 0})
time.sleep(1.0)
if call("/api/behavior", {"name": "patrol"}).get("error"):
    raise SystemExit("could not start patrol")

print(f"patrolling for {SECONDS / 60:.0f} minutes at {LIMIT:.0%}\n")

began = time.time()
last_state = last_kind = None
next_beat = HEARTBEAT_S
samples = moving = lost = 0
hijacked = False
blind_now = None
blind_events = 0
worst_blind = 0.0
counts = {}
closest = 999.0
distances = []
volts = []
window = []                 # (when, reading, was the car being driven)
stuck_now = None
worst_stuck = 0.0
stuck_events = 0

try:
    while time.time() - began < SECONDS:
        d = call("/api/telemetry")
        now = time.time() - began
        if "error" in d:
            lost += 1
            time.sleep(1.0)
            continue

        # Someone picking up the controller switches the robot to manual, and
        # the rest of this becomes a very long measurement of nothing. Say so
        # at once rather than at the end of it.
        if d.get("behavior") != "patrol":
            print("  %5.0fs  patrol was taken over by %s -- stopping"
                  % (now, d.get("behavior")))
            hijacked = True
            break

        state = d.get("behavior_status", {}).get("state", "?")
        commanded = d["requested"]
        driven = any(abs(commanded[k]) > 0.01 for k in ("vx", "vy", "omega"))
        wheels = list(d["drivetrain"]["wheels"].values())
        samples += 1
        if any(abs(w) > 0.01 for w in wheels):
            moving += 1

        raw = d.get("range_cm")
        reading = SILENT if raw is None or raw < 0 else raw
        if raw is not None and raw > 0:
            distances.append(raw)
            closest = min(closest, raw)
        if (d.get("supply") or {}).get("volts"):
            volts.append(d["supply"]["volts"])

        # -- is the room changing while we drive? --------------------------
        #
        # Only real numbers count. A stretch of nothing-came-back is also a
        # sequence that does not change, and treating that as evidence of not
        # moving means an open room and a sofa look the same -- which is the
        # mistake this whole afternoon has been made of. When the readings are
        # absent the honest answer is that this cannot tell, so it says so
        # separately instead of claiming a verdict.
        window.append((now, reading, driven))
        window[:] = [w for w in window if now - w[0] <= STUCK_S]
        settled = len(window) > 8 and now > STUCK_S
        real = [w[1] for w in window if w[1] > 0]
        if settled and all(w[2] for w in window):
            if len(real) < len(window) * 0.6:
                # Mostly silence. Might be an empty room, might be pressed up
                # against something soft. Not knowable from here.
                if blind_now is None:
                    blind_now = now
                    blind_events += 1
                worst_blind = max(worst_blind, now - blind_now + STUCK_S)
                stuck_now = None
            elif max(real) - min(real) < STUCK_SPREAD_CM:
                blind_now = None
                if stuck_now is None:
                    stuck_now = now
                    stuck_events += 1
                    print("  %5.0fs  STUCK: driving, view unchanged at %s | %s"
                          % (now, cm(raw), state))
                worst_stuck = max(worst_stuck, now - stuck_now + STUCK_S)
            else:
                stuck_now = blind_now = None
        else:
            stuck_now = blind_now = None

        kind = kind_of(state)
        if state != last_state:
            if kind != last_kind:
                counts[kind] = counts.get(kind, 0) + 1
            if kind in ("decision", "side", "reverse", "trapped"):
                print("  %5.0fs  %s cm  %s" % (now, cm(raw), state))
            last_kind, last_state = kind, state

        if now > next_beat:
            next_beat += HEARTBEAT_S
            print("  %5.0fs  --- %d sweeps, %d turns, %d stuck, %.2fV, moving %.0f%%"
                  % (now, counts.get("sweep", 0),
                     counts.get("decision", 0) + counts.get("side", 0),
                     stuck_events, volts[-1] if volts else 0,
                     100 * moving / max(1, samples)))
        time.sleep(0.3)
finally:
    call("/api/behavior", {"name": "idle"})
    call("/api/stop")
    call("/api/speed", {"limit": 0.15})
    call("/api/pan", {"angle": 0})

ran = time.time() - began
print()
print("=" * 64)
print("  ran for            %.1f minutes" % (ran / 60))
print("  moving             %.0f%% of samples" % (100 * moving / max(1, samples)))
print("  swept              %d times" % counts.get("sweep", 0))
print("  chose a way round  %d" % counts.get("decision", 0))
print("  side turns         %d" % counts.get("side", 0))
print("  reversed out       %d" % counts.get("reverse", 0))
print("  cornered escapes   %d" % counts.get("trapped", 0))
print("  closest it came    %.1f cm" % closest)
if distances:
    distances.sort()
    print("  typical clearance  %.0f cm (median)" % distances[len(distances) // 2])
if volts:
    print("  supply             %.2f -> %.2f V" % (volts[0], volts[-1]))
print("  stuck              %d times, longest %.1f s" % (stuck_events, worst_stuck))
print("  driving blind      %d stretches, longest %.1f s (nothing came back;"
      " cannot tell open room from soft obstacle)" % (blind_events, worst_blind))
print("  telemetry lost     %d times" % lost)
print()

problems = []
if hijacked:
    problems.append("something else took control")
if stuck_events:
    problems.append("stuck %d times, worst %.0fs" % (stuck_events, worst_stuck))
if moving / max(1, samples) < 0.5:
    problems.append("stationary over half the time")
if counts.get("sweep", 0) and counts.get("decision", 0) == 0:
    problems.append("looked around but never committed")
if lost > 20:
    problems.append("telemetry dropped repeatedly")
print("  NOT CLEAN: " + "; ".join(problems) if problems
      else "  Clean: kept finding things, kept getting round them, never wedged.")
