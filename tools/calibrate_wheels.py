#!/usr/bin/env python3
"""Work out which motors are wired backwards, and check the wheels are in the
right corners.

    sudo systemctl stop raspberrycar     # it holds the serial port
    python3 calibrate_wheels.py

Two separate things can be wrong, and they need different fixes:

  DIRECTION   a motor's two leads are swapped, so it turns the wrong way.
              Fixed in software, by flipping its entry in DIR[].

  PLACEMENT   a mecanum wheel is in the wrong corner. The rollers sit at 45
              degrees and must form an X across the chassis; get one wrong and
              the car will drive and turn perfectly well but refuse to strafe
              properly. No amount of DIR[] fiddling fixes that -- the wheel has
              to physically move.

Phase 1 finds the directions one wheel at a time, which is why the firmware
needs a raw per-wheel command: the mixer can isolate a diagonal pair at best.
Phase 2 then checks placement, because a strafe test is only meaningful once
the directions are right.

PUT THE CAR ON A STAND. Wheels off the ground throughout.
"""

import sys
import time
from pathlib import Path

# Reach the robot from wherever this is run. It used to assume the working
# directory was PythonCode, which is fine right up until the script moves or
# someone runs it from home -- and the failure is an ImportError about hal,
# which says nothing about the actual problem.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "PythonCode"))
from hal.link import SerialLink  # noqa: E402

NAMES = ["front left", "front right", "rear left", "rear right"]
SPEED = 0.45
SPIN_SECONDS = 2.0


def ask(prompt, options="y/n/r"):
    while True:
        answer = input(f"    {prompt} [{options}] ").strip().lower()
        if answer and answer[0] in options.replace("/", ""):
            return answer[0]
        print("    (y = yes, n = no, r = repeat)")


def spin(link, index, speed, seconds):
    """Drive one wheel, resending so the watchdog stays fed."""
    wheels = [0.0, 0.0, 0.0, 0.0]
    wheels[index] = speed
    body = "W,{:.3f},{:.3f},{:.3f},{:.3f}".format(*wheels)
    end = time.time() + seconds
    while time.time() < end:
        link._write(body)
        time.sleep(0.1)
    link.stop()
    time.sleep(0.4)


def drive(link, vx, vy, omega, seconds):
    end = time.time() + seconds
    while time.time() < end:
        link.send_velocity(vx, vy, omega)
        time.sleep(0.1)
    link.stop()
    time.sleep(0.4)


def main():
    print(__doc__)
    input("Wheels off the ground? Press Enter to start, Ctrl-C to abort. ")

    link = SerialLink()
    time.sleep(2.5)
    if not link.telemetry.fresh:
        print("\nNo telemetry from the ESP32. Is the service still holding the port?")
        print("  sudo systemctl stop raspberrycar")
        return 1

    print("\n=== phase 1: direction, one wheel at a time ===")
    print("Watch the TOP of the wheel. Driving 'forward' means the top of the")
    print("wheel travels towards the FRONT of the car.\n")

    dirs = []
    for i, name in enumerate(NAMES):
        while True:
            print(f"  [{i + 1}/4] {name}")
            spin(link, i, SPEED, SPIN_SECONDS)
            answer = ask("did the top of the wheel go towards the FRONT?")
            if answer == "r":
                continue
            dirs.append(1 if answer == "y" else -1)
            print(f"       -> {'correct' if answer == 'y' else 'REVERSED, will flip'}\n")
            break

    print("=" * 52)
    print("  const int DIR[4] = {%s};   // %s" % (
        ", ".join(f"{d:2d}" for d in dirs), ", ".join(NAMES)))
    print("=" * 52)

    flipped = [NAMES[i] for i, d in enumerate(dirs) if d < 0]
    if flipped:
        print(f"\n  Reversed: {', '.join(flipped)}")
        print("  Put that DIR[] line into firmware/bridge/bridge.ino, reflash, then")
        print("  run this again -- phase 2 is only meaningful once directions are right.")
        print("\n    arduino-cli compile --fqbn esp32:esp32:esp32 ~/projects/RaspberryCar/firmware/bridge")
        print("    arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 ~/projects/RaspberryCar/firmware/bridge")
        link.close()
        return 0

    print("\n  All four already correct.\n")

    print("=== phase 2: wheel placement ===")
    print("Directions are right, so anything wrong now is physical: a mecanum")
    print("wheel in the wrong corner. Put the car DOWN on the floor for these.\n")
    input("  Car on the floor, space around it? Press Enter. ")

    checks = [
        ("forward", 0.4, 0, 0, "drive straight forward, not veering"),
        ("strafe right", 0, 0.45, 0, "slide sideways to the RIGHT, nose staying put"),
        ("strafe left", 0, -0.45, 0, "slide sideways to the LEFT, nose staying put"),
        ("spin", 0, 0, 0.4, "turn on the spot"),
    ]
    problems = []
    for label, vx, vy, omega, expect in checks:
        print(f"  {label}: expect it to {expect}")
        input("    Enter to run... ")
        drive(link, vx, vy, omega, 1.6)
        if ask("    did it?") != "y":
            problems.append(label)
        print()

    print("=" * 52)
    if not problems:
        print("  Everything behaves. Directions and placement are both right.")
    else:
        print(f"  Wrong: {', '.join(problems)}")
        if any("strafe" in p for p in problems) and "forward" not in problems:
            print()
            print("  Forward is fine but strafing is not, which is the signature of a")
            print("  mecanum wheel in the wrong corner. Looking down at the car, the")
            print("  top rollers should form an X:")
            print()
            print("        front")
            print("      \\       /        front left  rollers point  \\")
            print("                       front right rollers point  /")
            print("      /       \\        rear  left  rollers point  /")
            print("        rear             rear right rollers point  \\")
            print()
            print("  Swap whichever wheels do not match. This cannot be fixed in code.")
    print("=" * 52)

    link.close()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\naborted")
