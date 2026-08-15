#!/usr/bin/env python3
"""Two questions the car can only answer with you watching it.

    1. How hard do the motors have to be driven before the wheels turn at all?
       Every default above this has been picked without knowing that number.

    2. What is the controller actually asking for, and what happens to it on
       the way to the wheels?

PART 1 DRIVES ALL FOUR WHEELS. Put the car on a stand first.
"""

import json
import time
import urllib.request

BASE = "http://127.0.0.1:8000"


def call(path, body=None):
    if body is None:
        req = urllib.request.Request(BASE + path)
    else:
        req = urllib.request.Request(
            BASE + path, data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"},
        )
    try:
        with urllib.request.urlopen(req, timeout=3) as r:
            return json.load(r)
    except Exception as exc:  # noqa: BLE001
        return {"error": str(exc)}


def wheels():
    d = call("/api/telemetry")
    return [round(v, 2) for v in d.get("drivetrain", {}).get("wheels", {}).values()]


def part1(original_limit):
    print()
    print("=" * 64)
    print("PART 1 -- what does it take to make the wheels turn?")
    print("=" * 64)
    print("Car on a stand, wheels free.")
    print()
    print("Each step drives all four forwards for a second, from very gentle")
    print("upwards. Watch for the first one where the wheels actually TURN,")
    print("rather than the motors just whining. Note that number.")
    print()
    input("Enter when the car is on a stand. Ctrl-C to skip. ")

    # The governor is opened right up for this. The question is about the
    # motors, and leaving a limit in the way would be measuring the software.
    call("/api/speed", {"limit": 1.0})
    print()
    try:
        for duty in [0.05, 0.08, 0.10, 0.12, 0.15, 0.18, 0.22, 0.26, 0.30, 0.35, 0.45]:
            end = time.time() + 1.0
            while time.time() < end:
                call("/api/drive", {"vx": duty, "vy": 0, "omega": 0})
                time.sleep(0.08)
            print(f"  duty {duty:.2f}   wheels {wheels()}")
            call("/api/stop")
            time.sleep(0.6)
    finally:
        call("/api/speed", {"limit": original_limit})
        call("/api/stop")
    print()
    print(f"  (speed limit put back to {original_limit})")


def part2():
    print()
    print("=" * 64)
    print("PART 2 -- what is the controller asking for?")
    print("=" * 64)
    print("Push the LEFT stick fully forward and HOLD it.")
    print("Then, still holding, squeeze R2.")
    print("Watching for 25 seconds.")
    print()

    end = time.time() + 25
    peak = 0.0
    while time.time() < end:
        d = call("/api/telemetry")
        if "error" in d:
            print("  ", d["error"])
            break
        req, allowed = d["requested"], d["allowed"]
        peak = max(peak, abs(req["vx"]), abs(req["vy"]), abs(req["omega"]))
        print("  asked vx %+.3f vy %+.3f w %+.3f | allowed vx %+.3f | wheels %s"
              % (req["vx"], req["vy"], req["omega"], allowed["vx"],
                 [round(v, 2) for v in d["drivetrain"]["wheels"].values()]))
        time.sleep(0.5)

    print()
    print(f"  biggest thing the robot was asked for: {peak:.3f}")
    if peak == 0.0:
        print("  -> nothing from the sticks reached the robot at all, even though")
        print("     the reader is connected. That is a different fault.")
    else:
        print("  -> the controller is getting through. Compare that number with")
        print("     part 1: if it is below where the wheels start turning, the")
        print("     controller is working and the car is simply being asked for")
        print("     less than it takes to move.")


status = call("/api/telemetry")
limit = status.get("speed_limit", 0.15)
print("speed limit:", limit)
print("gamepad    :", status.get("gamepad"))
print("link       :", status.get("drivetrain", {}).get("link"))

try:
    part1(limit)
except KeyboardInterrupt:
    call("/api/speed", {"limit": limit})
    call("/api/stop")
    print("\n  (skipped)")
try:
    part2()
except KeyboardInterrupt:
    print("\n  (skipped)")
call("/api/stop")
print("\ndone, motors stopped")
