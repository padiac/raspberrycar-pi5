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

# -- looking around while driving ------------------------------------------
#
# The sweep above only happens once the way ahead is already blocked, which
# means that while actually driving the car stares at one direction and knows
# nothing about the rest of the room. That is how it collects chair legs: a
# leg shows up in a sweep as a single ten-degree spike -- 45cm with 84 and 118
# either side of it -- so it is perfectly visible, and completely invisible
# unless the head happens to be pointed at it. The corners of the chassis are
# wider than the beam.
#
# So the head glances left and right as it goes. The safety layer allows this
# because it now accepts a recent forward reading rather than demanding a
# current one; the glance has to be short enough to stay inside that window,
# which is what GLANCE_DWELL and the return to centre are for.

#: How often to look away from straight ahead while cruising.
GLANCE_EVERY_S = 1.8
#: How far to the side, and how long to hold it there. Long enough for the
#: servo to arrive and the sensor to ping, short enough that the forward
#: reading has not gone stale by the time the head is back.
GLANCE_ANGLE = 40.0
GLANCE_DWELL = 0.35
#: Something this close beside the car is worth steering away from, even
#: though nothing is blocking the way ahead. This is intent, not safety: the
#: safety layer only ever removes motion, and turning away is adding some.
SIDE_ALERT_CM = 30.0
#: How hard to lean away from it. Gentle -- the car is still going forwards,
#: and this is a nudge around a table leg rather than a change of plan.
SIDE_AVOID_TURN = 0.35
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
        self.glance = None          # (angle, when to read it)
        self.glance_next = 0.0      # when to look away again
        self.glance_side = 1.0      # alternates, so both sides get looked at
        self.sides = {}             # last thing seen to each side
        self.state = "idle"

    def start(self, robot):
        self.until = 0.0
        self.move = Velocity.STOP
        self.blocked_since = None
        self.scan = None
        self.clearance = {}
        self.glance = None
        self.glance_next = 0.0
        self.sides = {}
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

        return self._cruise(robot, world, now)

    # -- driving along, looking around -------------------------------------

    def _cruise(self, robot, world, now):
        """Drive forward, glancing sideways, leaning away from what is there."""
        if robot.pan is not None:
            self._glance(robot, world, now)

        # Steer away from the nearer side, if either is close enough to matter.
        # Both being close is a gap, not an obstacle, and steering out of a gap
        # is how you hit the other side of it.
        left = self.sides.get(-1, 999.0)
        right = self.sides.get(1, 999.0)
        omega = 0.0
        note = ""
        if min(left, right) < SIDE_ALERT_CM and abs(left - right) > 8.0:
            away = 1.0 if left < right else -1.0
            omega = away * SIDE_AVOID_TURN
            note = " | easing %s off %.0fcm" % (
                "right" if away > 0 else "left", min(left, right))

        aim = "" if not self.glance else " | looking %+.0f" % self.glance[0]
        self.state = "cruising" + aim + note
        return Velocity(vx=CRUISE, omega=omega)

    def _glance(self, robot, world, now):
        """Look aside briefly, then come straight back.

        Coming back matters more than the looking. The safety layer will only
        drive forward on a reading that is recent, so a head left pointing at
        the scenery stops the car -- correctly, but uselessly.
        """
        if self.glance is not None:
            angle, due = self.glance
            if now < due:
                return
            reading = world.range_cm
            self.sides[1 if angle > 0 else -1] = (
                999.0 if reading is None or reading <= 0 else reading)
            robot.pan.centre()
            self.glance = None
            self.glance_next = now + GLANCE_EVERY_S
            return

        if now < self.glance_next or not robot.pan.settled:
            return
        angle = GLANCE_ANGLE * self.glance_side
        self.glance_side *= -1
        robot.pan.look(angle)
        self.glance = (angle, now + GLANCE_DWELL)

    # -- looking for somewhere to go ---------------------------------------

    def _start_looking(self, robot, now):
        """Begin the sweep, or fall back on a guess with no head to sweep."""
        if robot.pan is None:
            return self._commit(
                robot, now, Velocity(omega=self._guess(robot) * TURN_SPEED),
                TURN_HOLD_MIN, "blocked, picking a way round",
            )
        self.clearance = {}
        self.glance = None          # the sweep takes the head from here
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
        roomiest = max(self.clearance.values(), default=0.0)
        if roomiest < BOXED_IN_CM:
            # Nowhere in front has room. Turning on the spot in a corner just
            # finds the same corner again, so back out and look afresh.
            return self._commit(
                robot, now, Velocity(vx=-REVERSE_SPEED), REVERSE_HOLD,
                f"boxed in ({roomiest:.0f}cm all round), backing out",
            )

        # Only somewhere else is a candidate. Straight ahead is where the car
        # is already pointing and has just been refused, so "turn towards 0
        # degrees" is not a manoeuvre -- it was measured anyway, because how
        # blocked the front is tells a doorway from a wall.
        #
        # This used to fall out of a special case: pick the roomiest of all
        # five, and if that came out as straight ahead, turn to a randomly
        # chosen side. Which is how the car was seen turning towards 19cm of
        # room with 400cm available on the other side -- the coin toss did not
        # look at the numbers it was choosing between.
        options = {a: cm for a, cm in self.clearance.items() if a != 0.0}
        if not options:
            return self._commit(
                robot, now, Velocity(omega=self._guess(robot) * TURN_SPEED),
                TURN_HOLD_MIN, "nowhere measured, guessing",
            )

        room = max(options.values())
        # A tie is real -- a corridor looks the same both ways -- so break it
        # randomly rather than always choosing the same side of it.
        rivals = [a for a, cm in options.items() if cm > room - 5.0]
        best = random.choice(rivals)
        room = options[best]

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
            "sides": {("left" if k < 0 else "right"): round(v)
                      for k, v in self.sides.items() if v < 999},
        }
