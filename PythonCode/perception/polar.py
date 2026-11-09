"""What has been seen in each direction lately.

One rangefinder on a servo measures one direction at a time, so "how far
away is something" is really "how far away was something, in that
direction, a moment ago". Nothing kept those moments together: the robot
reported the latest reading and the angle it was taken at, and the shape of
the room -- which the car has in fact been measuring all along, one spoke
at a time -- existed nowhere outside the patrol behaviour's private notes.

This keeps them. Every reading is filed under the direction the head was
pointing, old ones expire, and what comes out is the nearest thing a car
with one spinning sensor has to a picture.

It lives here, in perception, rather than in the behaviour that first
needed it, for two reasons. It is a fact about the room and not about any
plan, which is the line this project draws between the two. And a human
driving by hand wants to see it as much as a patrolling car needs it --
more, since the human is the one who cannot see round the corners of their
own chassis.

Readings keep their three states, as everywhere else in this codebase:

    positive  a measurement
    negative  the pulse went out and nothing came back
    None      never offered; nothing is stored

A silence is worth recording. It is not a distance, but "I looked there and
heard nothing" is different from "I have not looked there", and a picture
that cannot tell them apart is the picture that drove this car into a sofa.
"""

#: Directions are rounded to this, in degrees.
#
# The beam is about fifteen degrees wide, so two readings five degrees
# apart are largely the same measurement and filing them separately would
# claim a resolution the sensor does not have. Five is also what the
# direction chooser works in, so nothing has to be resampled.
BUCKET = 5.0

#: How long a reading describes where the car is now.
#
# The car moves while it looks. At a cruise of a few tens of centimetres a
# second, a reading from four seconds ago was taken from somewhere else
# entirely, and drawing it as though it were current is worse than drawing
# nothing -- it is a picture of a room the car has left.
MEMORY_S = 4.0


def bucket(angle):
    return round(float(angle) / BUCKET) * BUCKET


class PolarMemory:
    """Readings by direction, with an age, forgotten when stale."""

    def __init__(self, memory_s=MEMORY_S):
        self.memory_s = memory_s
        self._seen = {}             # angle -> (cm, when)

    def note(self, angle, centimetres, now):
        """File one reading. None means there was nothing to file."""
        if angle is None or centimetres is None:
            return
        self._seen[bucket(angle)] = (float(centimetres), now)

    def forget(self):
        self._seen.clear()

    def readings(self, now):
        """Everything still fresh, nearest direction first.

        Each entry carries its age as well as its value, because how much
        to trust a spoke is mostly a question of how long ago it was drawn
        -- and whoever is looking at it should be able to see that rather
        than be protected from it.
        """
        out = []
        for angle, (centimetres, when) in self._seen.items():
            age = now - when
            if age > self.memory_s:
                continue
            out.append({
                "angle": angle,
                "cm": round(centimetres, 1),
                "age": round(age, 2),
                # Spelled out rather than left to whoever reads the sign,
                # because the one thing everything downstream gets wrong
                # about this sensor is treating a silence as a distance.
                "echo": centimetres >= 0,
            })
        out.sort(key=lambda entry: entry["angle"])
        return out

    def prune(self, now):
        """Drop what has expired. Called occasionally; readings() already
        hides stale entries, so this is only about not growing."""
        for angle in [a for a, (_cm, when) in self._seen.items()
                      if now - when > self.memory_s]:
            del self._seen[angle]

    def status(self, now):
        fresh = self.readings(now)
        return {
            "memory_s": self.memory_s,
            "bucket": BUCKET,
            "readings": fresh,
            "spokes": len(fresh),
        }
