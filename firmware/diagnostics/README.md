# Diagnostics

Small sketches that test one thing without the rest of the robot in the way.

The point of keeping them is that they answer questions `bridge.ino` cannot.
The bridge assumes the hardware works and gets on with driving; when something
underneath it is broken, "the car does not move" is all it can tell you. These
narrow that down to a wire.

Flash one, read it, then **put `bridge/` back** -- the car does nothing until
you do.

    cd ~/projects/RaspberryCar
    arduino-cli compile --fqbn esp32:esp32:esp32 firmware/diagnostics/<name>
    arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 firmware/diagnostics/<name>
    # watch it
    arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200
    # and afterwards
    arduino-cli compile --fqbn esp32:esp32:esp32 firmware/bridge
    arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 firmware/bridge

Stop the service first, or it will be holding the port:

    sudo systemctl stop raspberrycar

## Start here

| | |
|---|---|
| `ultrasonic` | Range readings, no libraries. Run this one first on a new board: it proves the toolchain, the upload path and the serial monitor before any motor is involved. |
| `servo` | Sweeps the pan mount. 500-2400us rather than the library default, because SG90 clones vary. Buzzing at an end stop means the pulse is asking for further than the horn can go. |
| `motors` | Spins each of the four motors on its own, so wiring and direction can be checked one at a time. **Wheels off the ground** -- 70% for two seconds each, with no way to stop it from outside. |

## When the ultrasonic goes quiet

These four were written in order during one real fault, and they read as the
argument that found it. The answer turned out to be that ECHO had been wired to
5V. Worth following the same order if it happens again, because each one
narrows what the next has to consider.

| | |
|---|---|
| `echodiag` | Looks at the ECHO line directly rather than through `pulseIn`, which only ever says yes or no. Idle level, response to a trigger, and whether the line is driven or floating -- three readings that tell the causes apart. |
| `echodiag2` | Round two, once round one established that ECHO never rises. Note the thing that wasted time: ECHO sitting at 0V when idle is **normal**, held there by the divider. Only the failure to rise is a fault. |
| `echoscan` | Fires one trigger and watches every plausible input at once, so the pin the wire is actually in does not have to be guessed. 34/35/36/39 are the usual suspects: they are adjacent on the header and two of them are silkscreened VP and VN rather than by number. |
| `trigcheck` | For when fourteen candidate pins have shown no echo and the question is no longer which pin. Toggles TRIG slowly so a multimeter on the sensor's own pin can confirm the signal arrives at all. |

## On the battery

| | |
|---|---|
| `selftest` | One shot, run on battery, read the verdict. Exercises the paths USB testing cannot: on USB the 5V comes from the Pi, on battery it comes from the buck, and that path carries no current until you try. The motors, both TB6612s and the 7.4V rail are untouched by any bench test. |
| `fulltest` | Everything at once, forever: servo, ultrasonic, four wheels. **Wheels off the ground** -- the motor phase starts on its own ten seconds in. Needs no serial monitor: flash it, unplug USB, run it on the battery, and read the phase off the servo, which doubles as a status indicator. |
