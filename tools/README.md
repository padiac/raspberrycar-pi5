# Tools

Scripts you run by hand, when you want an answer about the hardware. Nothing
here runs on its own and nothing in `PythonCode/` imports any of it -- that is
the line: `PythonCode/` is the robot, this is the workshop.

Most of them want the venv:

    cd ~/projects/RaspberryCar && source venv/bin/activate

## Calibration

**`calibrate_wheels.py`** -- which motors are wired backwards, and whether each
mecanum wheel is in the right corner. Two questions that look alike and are
fixed completely differently: a reversed motor is a sign in the firmware's
`DIR[]`, while a wheel in the wrong corner has to be physically moved, since
the rollers sit at 45 degrees and must form an X across the chassis. Get that
wrong and the car drives and turns perfectly while refusing to strafe.

Stop the service first -- it holds the serial port.

    sudo systemctl stop raspberrycar
    python3 tools/calibrate_wheels.py
    sudo systemctl start raspberrycar

**`stall_test.py`** -- how hard the motors have to be driven before the wheels
turn at all. Every speed default above this number was picked without knowing
it. Talks to the running robot over HTTP, so leave the service up. **Car on a
stand**: it drives all four wheels.

## Measurement

**`bench_detect.py`** -- times object detectors on this Pi against a real
photograph. This is where the 58fps for YOLO26n through NCNN came from, and the
reason the project is not using PyTorch for inference.

**`bench_face.py`** -- the same for face detection. Kept for the numbers; that
path was dropped once general detection turned out to be fast enough to make a
special case for faces pointless.

**`focus_check.py`** -- whether a soft picture is out of focus or merely low
resolution. Worth knowing before spending an evening on the second when the
problem is the first: these camera modules have a lens you can turn.

## Setup

**`pair_gamepad.sh`** -- pairs a controller over Bluetooth. `bluetoothctl` is a
shell of its own that takes no script, and the sequence is easy to get wrong.
Hold PS + Create on a DualSense until the light bar flashes fast, then run it.

Trust is the step that matters afterwards: without it the pad needs
re-authorising after every reboot, which on a car means every time it is
switched on.
