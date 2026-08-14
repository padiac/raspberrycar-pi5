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

from web.server import main

if __name__ == "__main__":
    main()
