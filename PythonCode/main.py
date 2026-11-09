#!/usr/bin/env python3
"""Start the car.

    python3 main.py

Then open http://<pi-ip>:8000 from a PC or phone on the same network.

Environment:
    CAR_DRIVETRAIN   differential (default) | mecanum
    CAR_STREAM_SIZE  e.g. 1280x720
    CAR_JPEG_QUALITY 1-100, default 90
    CAR_PORT         default 8000
"""

# Before anything else prints: from here on every print lands in a rotating
# file as well as on stdout. journald on this Pi is volatile and the car has
# been rebooting on its own, which destroys exactly the log that would
# explain the reboot.
from diagnostics.logbook import book

book.install()

from web.server import main  # noqa: E402 - must come after the log is up

if __name__ == "__main__":
    main()
