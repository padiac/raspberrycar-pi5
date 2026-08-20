"""Wander around without hitting anything.

This is what the original called `main_obstacle_avoidance.py`, renamed to say
what it actually is. "Obstacle avoidance" was never a goal -- it is a
constraint, and it now lives in the safety layer where it applies to every
behaviour. What is left here is the *intent*: keep exploring.

The division of labour is worth noticing, because it is the whole point of the
refactor. This behaviour just asks to drive forward. When something is in the
way the safety layer clamps that to zero on its own; this code then sees that
it is not making progress and decides where to go instead -- because choosing a
direction is intent, and intent does not belong in the safety layer.

Choosing used to be a coin toss, because there was nothing to choose on. The
ultrasonic sits on a servo, so when the way ahead is blocked the car can stop
and sweep the arc in front of it, ending up with a clearance at each angle
rather than a single number. It then turns towards the roomiest one, and turns
further for an angle further round. That is the whole reason for the pause: the
head has to be pointing at each question in turn.
"""

import random
import time

from hal import Velocity

from .base import Behavior

CRUISE = 0.8
TURN_SPEED = 0.7
#: How long forward progress must be blocked before deciding to do something.
STUCK_AFTER = 0.25

#: Where the head looks, in order, so it sweeps across rather than jumping
#: about. Straight ahead is included even though that is the direction already
#: known to be blocked: it says by how much, which is what tells a doorway
#: apart from a wall.
SCAN_ANGLES = (-90.0, -45.0, 0.0, 45.0, 90.0)
#: Time to hold each angle. The servo has to arrive and the ultrasonic needs a
#: ping or two from the new direction; a reading taken mid-swing describes
#: somewhere the head no longer is.
SCAN_DWELL = 0.4
#: A timed-out ping means nothing within about 4m, which for "where is there
#: room" is simply the best possible answer.
CLEAR_CM = 400.0

#: Turning towards 90 degrees should turn further than towards 45, so the hold
#: scales with how far round the chosen direction is.
TURN_HOLD_MIN = 0.45
TURN_HOLD_MAX = 1.1
#: Below this everywhere, there is nowhere forwards worth going and the car
#: backs out instead of turning on the spot inside a corner.
BOXED_IN_CM = 35.0
REVERSE_SPEED = 0.5
REVERSE_HOLD = 0.7


class PatrolBehavior(Behavior):
    name = "patrol"
    label = "Patrol"
    description = "Drives around on its own, sweeping for room when blocked."
    requires = ("motors",)

    def __init__(self):
        self.until = 0.0            # when the committed manoeuvre ends
        self.move = Velocity.STOP   # what it is while it lasts
        self.blocked_since = None
        self.scan = None            # index into SCAN_ANGLES, and when to read
        self.clearance = {}
        self.state = "idle"

    def start(self, robot):
        self.until = 0.0
        self.move = Velocity.STOP
        self.blocked_since = None
        self.scan = None
        self.clearance = {}
        if robot.pan:
            robot.pan.centre()

    def stop(self, robot):
        # Leave the head where the next behaviour expects it. Everything else
        # assumes forward, and an assumption nobody maintains is how the sensor
        # came to be staring at a wall three metres away in the first place.
        if robot.pan:
            robot.pan.centre()

    def update(self, robot, world):
        now = time.monotonic()

        if self.scan is not None:
            return self._sweep(robot, world, now)

        # Mid-manoeuvre from an earlier tick: see it through.
        if now < self.until:
            return self.move

        # Ask the safety layer what happened to our last request rather than
        # reading the sensors again here -- it already decided, and duplicating
        # the thresholds is how the two end up disagreeing.
        blocked = robot.safety.last.allowed.vx <= 0.0 < robot.safety.last.requested.vx

        if blocked:
            if self.blocked_since is None:
                self.blocked_since = now
            elif now - self.blocked_since > STUCK_AFTER:
                self.blocked_since = None
                return self._start_looking(robot, now)
        else:
            self.blocked_since = None

        self.state = "cruising"
        return Velocity(vx=CRUISE)

    # -- looking for somewhere to go ---------------------------------------

    def _start_looking(self, robot, now):
        """Begin the sweep, or fall back on a guess with no head to sweep."""
        if robot.pan is None:
            return self._commit(
                robot, now, Velocity(omega=self._guess(robot) * TURN_SPEED),
                TURN_HOLD_MIN, "blocked, picking a way round",
            )
        self.clearance = {}
        self.scan = (0, now + SCAN_DWELL)
        robot.pan.look(SCAN_ANGLES[0])
        self.state = "looking around"
        return Velocity.STOP

    def _sweep(self, robot, world, now):
        """One angle of the sweep. Stationary throughout.

        Staying still is not politeness. The safety layer only accepts a range
        reading as forward clearance when the head is forward, so driving
        during a sweep would be driving with no forward clearance at all -- it
        would be refused anyway, and rightly.
        """
        index, due = self.scan
        if now < due:
            return Velocity.STOP

        angle = SCAN_ANGLES[index]
        reading = world.range_cm
        self.clearance[angle] = (
            CLEAR_CM if reading is None or reading <= 0 else min(reading, CLEAR_CM)
        )

        if index + 1 < len(SCAN_ANGLES):
            self.scan = (index + 1, now + SCAN_DWELL)
            robot.pan.look(SCAN_ANGLES[index + 1])
            self.state = f"looking {self._name(SCAN_ANGLES[index + 1])}"
            return Velocity.STOP

        robot.pan.centre()
        self.scan = None
        return self._decide(robot, now)

    def _decide(self, robot, now):
        best = max(self.clearance, key=lambda a: (self.clearance[a], -abs(a)))
        room = self.clearance[best]

        if room < BOXED_IN_CM:
            # Nowhere in front has room. Turning on the spot in a corner just
            # finds the same corner again, so back out and look afresh.
            return self._commit(
                robot, now, Velocity(vx=-REVERSE_SPEED), REVERSE_HOLD,
                f"boxed in ({room:.0f}cm all round), backing out",
            )

        # A tie is real -- a corridor looks the same both ways -- so break it
        # randomly rather than always choosing the same side of it.
        rivals = [a for a in self.clearance if self.clearance[a] > room - 5.0]
        if len(rivals) > 1 and all(abs(a) > 0 for a in rivals):
            best = random.choice(rivals)
            room = self.clearance[best]

        if best == 0.0:
            # The sweep disagrees with the safety layer about straight ahead.
            # Trust the safety layer, which is looking at a fresher reading,
            # and turn away rather than driving at what it just refused.
            best = random.choice((-90.0, 90.0))
            room = self.clearance[best]

        direction = 1.0 if best > 0 else -1.0
        hold = TURN_HOLD_MIN + (TURN_HOLD_MAX - TURN_HOLD_MIN) * (abs(best) / 90.0)
        profile = " ".join(
            f"{int(a):+d}:{self.clearance[a]:.0f}" for a in sorted(self.clearance)
        )
        return self._commit(
            robot, now, Velocity(omega=direction * TURN_SPEED), hold,
            f"most room {self._name(best)} ({room:.0f}cm) | {profile}",
        )

    def _commit(self, robot, now, velocity, seconds, state):
        """Hold a manoeuvre for a while, so the car clears what stopped it
        instead of twitching on the spot as the reading flickers.

        Also says so when the manoeuvre cannot physically happen. A turn is
        asked for as a fraction of full power and then scaled again by the
        speed limit, and below a certain duty the motors buzz without moving
        -- rotating in place fights the rollers on all four wheels, so it
        costs more than driving straight. When that happens the behaviour is
        working perfectly and the car sits still: it decides, commits, waits
        out the hold, finds itself blocked, and decides again, for as long as
        you leave it. Nothing in the logs looks wrong, because nothing is
        wrong except the arithmetic.
        """
        self.move = velocity
        self.until = now + seconds
        floor = robot.calibration.get("turn_floor") if robot.calibration else 0.0
        effective = abs(velocity.omega) * robot.speed_limit
        if floor and velocity.omega and effective < floor:
            state += (f" -- but {effective:.2f} is under this car's {floor:.2f} "
                      f"turning floor, so it will not move; raise the speed")
        self.state = state
        return velocity

    @staticmethod
    def _name(angle):
        if angle == 0:
            return "straight ahead"
        return f"{abs(int(angle))} degrees {'left' if angle < 0 else 'right'}"

    def _guess(self, robot):
        """No head to sweep with: fall back on the side infrared, then a coin."""
        proximity = robot.world.proximity
        if proximity is not None:
            if proximity.left and not proximity.right:
                return 1.0
            if proximity.right and not proximity.left:
                return -1.0
        return random.choice((-1.0, 1.0))

    def status(self):
        return {
            "state": self.state,
            "clearance": {str(int(k)): round(v) for k, v in self.clearance.items()},
        }
