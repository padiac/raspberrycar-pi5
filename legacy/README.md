# The project this one grew out of

Everything here is from the original 2019 curriculum project, and none of it
runs any more. It is kept rather than deleted because it is the only record of
what the car was before, and because several of the arguments in the current
code only make sense against it.

Nothing in the working robot imports anything from this directory. Deleting the
whole thing would break nothing except the images in the top-level README.

```
python/   the original scripts, as they were left
older/    an even earlier version that shipped inside the original repo
media/    the gifs and photos its README used, and 35 tennis ball frames
site/     its GitHub Pages site
```

## Why keep it

The current design is mostly a series of answers to how this code was
arranged, and the answers read as arbitrary without the question.

`main_obstacle_avoidance.py`, `main_lane_tracking.py`, `main_tennis_tracking.py`
are each a whole program with its own loop, its own camera setup and its own
motor calls. That is the shape that made obstacle avoidance a *mode you chose*:
you got avoidance, or you got lane following, never both, because each one owned
the loop. Splitting that into a safety layer every behaviour passes through was
the single biggest change, and it is hard to see the point of it without seeing
what it replaced.

`main_tennis_tracking.py` could chase exactly one thing, found with a Hough
transform and an HSV colour test tuned to a particular tennis ball under
particular light -- hence 35 photographs of one, in `media/tennis`. The current
follow behaviour is the same control idea with the detection made somebody
else's problem, which is why it works on eighty classes instead of one.

`track.py` and `PID_controller.py` are the ancestors of `control/pid.py`, and
the PID maths in there is essentially unchanged. Some of this code was right.

## What it cannot tell you

It was written for a Raspberry Pi 3 running Raspbian Stretch, with `RPi.GPIO`,
the legacy `picamera`, TensorFlow 1.x, and Python 3.5. Every one of those is
either gone from the platform or incompatible with the Pi 5, which is what the
first weeks of this project were spent discovering. The pin numbers are the
only part that carried over unchanged, and even those now live in the ESP32
firmware rather than here.

So: read it for the shape, not for how to do anything.
