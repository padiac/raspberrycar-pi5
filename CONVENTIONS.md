# How this codebase is put together

Rules that earned their place by something going wrong first. Where a rule
can be checked by a machine it is, and `tests/test_architecture.py` and
`tests/test_names.py` fail the build rather than leaving it to a reviewer.

## Layers, and which way they point

```
settings  calibration     choices, and measurements of this machine
diagnostics               logging, the flight recorder
hal                       pins, buses, the camera
perception                sensors to a WorldState
control                   what is allowed; where to point
behaviors                 what to try
robot                     the loop between them
web                       how a human reaches it
```

A module may import its own layer or any layer below it, **never above**.
This is the property that makes hardware swappable: the moment `hal` knows
what the robot is trying to do, the driver and the robot are one thing
wearing two names. Checked automatically.

## Hardware

**Every device implements a contract in `hal/base.py`** — `RangeSensor`,
`ProximitySensor`, `FloorSensor`, `PanMount`, `PowerMonitor`, `FrameSource`,
`Drivetrain` — and **registers a driver**:

```python
@driver("range", "esp32", priority=10, needs_link=True)
def _range_over_serial(context):
    return SerialRangeFinder(context.link)
```

**Nothing outside `hal` may branch on which implementation it got.** Asking
"is this a SerialRangeFinder?" is the bug the layer exists to prevent; ask
the contract instead. Swapping an ultrasonic for a time-of-flight array
should be one new file and no edits anywhere else.

A driver **may return None**. No camera ribbon, no board, a library nobody
installed: all ordinary, all handled, all **said out loud** — a car that
comes up without a rangefinder and mentions nothing looks exactly like a car
whose rangefinder is lying.

Every implementation carries a `label` naming the hardware, so the startup
log shows which driver filled each role. Checked automatically.

## Numbers

Three kinds, three homes. Choosing the wrong one is the commonest mistake
here, so the test is written out:

| Kind | Home | Test |
|---|---|---|
| A **choice** somebody might change | `settings.py`, persisted to `settings.json` | Could two owners reasonably want different values? |
| A **measurement** of this one machine | `calibration.json` | Would you need a ruler, or a wall and a notebook? |
| **Structure** | a module constant | Would changing it be a code change — a protocol field count, a histogram resolution? |

A default for a measured fact is a guess wearing the costume of an answer,
which is why calibration has no defaults worth trusting and settings do.

**Settings are declared beside the code that explains them**, never moved
into a file of their own:

```python
REFLEX = group(
    "safety",
    stop_cm=Spec(STOP_CM, unit="cm", bounds=(0.0, 200.0),
                 help="Forward motion refused closer than this."),
)
```

The most valuable thing in this repository is not the numbers, it is the
paragraphs beside them. A central configuration file orphans those
paragraphs next to code that no longer mentions them, and within a month
the two disagree. What is centralised is the *register* — one page lists
and changes all of them — not the values.

**A value outside its range is refused, not clamped.** A clamp quietly
accepts a request nobody meant and never tells the sender that the number
it asked for is not the number in force.

## Control

**The safety layer only ever removes motion.** It may slow, clamp or refuse;
it may not steer. A layer that injects movement fights the behaviour above
it — both push, neither wins, and the result is oscillation nobody can
debug. Deciding *where to go instead* is intent, and intent belongs to
behaviours.

**A sensor reading has three states and they never collapse into two.**

```
positive   a measurement
negative   nothing came back within range
None       the sensor is not answering
```

Folding the last two together has cost this project more than any other
single mistake. Indoors a timed-out ping is usually glass, a curtain or a
sofa — things that return exactly what an empty room returns.

## Comments

Say **why**, next to the thing. A number with a paragraph explaining what
was measured, what failed before, and what the alternative was, is worth
more than the number. Several of the rules on this page exist because a
past comment recorded the failure that produced them.

When a bug is fixed, the comment records **what the wrong behaviour looked
like from outside** — "a car that ran a minute ago and now will not answer
at all" — because that is what the next person will have in their hands.

## Tests

- One suite per subject; `tests/run_all.py` runs them.
- **Every check names a real-world failure**, not an abstraction. "a side
  that heard nothing does not count as room", not "test_sidestep_3".
- **No test may depend on the live register.** A test that sets a global
  changes the next test, and on this machine it would also change the car.
  Pass explicit overrides: `SafetyLayer(no_echo_ceiling=0.22)`.
- **No test touches hardware.** It would fail on a desk and, worse, fight
  the car while it is running.
- A test that fails is first assumed to be *the test* being wrong. Four of
  the five failures during this refactor were the test's arithmetic, not
  the code's — including a doorway 25cm wide that the car was right to
  refuse.

## Logging

`print()` everywhere; `diagnostics/logbook.py` tees stdout into a rotating
file with timestamps. Nothing needs migrating to a logger and no new print
can forget to be captured.

**Size is bounded by construction**, not by a promise to tidy up later: a
fixed size times a fixed number of backups is a ceiling the writer cannot
exceed even in principle, and the arithmetic is asserted at import. The
periodic sweep is the second line of defence, for stale files and for the
disk filling because of something else.

**Nothing fails silently.** An exception that is swallowed must leave a line
behind saying so.

## Checking it

```bash
python3 PythonCode/tests/run_all.py     # every suite, including these rules
python3 tools/architecture.py PythonCode
python3 tools/undefined.py PythonCode
```

After anything structural, restart the service and read the startup log:
it names the driver that filled each role, and that is the quickest proof
that the car still assembles.
