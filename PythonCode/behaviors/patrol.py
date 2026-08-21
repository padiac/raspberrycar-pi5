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
#: The most room any direction can be worth, so one very long reading does
#: not dominate every comparison.
CLEAR_CM = 400.0

#: What a timed-out ping is worth when choosing where to go.
#:
#: Not the maximum, which is what it used to be scored as. An echo that never
#: came back might be four metres of empty room, or a curtain ten centimetres
#: away -- and indoors it is usually the soft thing. Fabric absorbs the pulse
#: and glass mirrors it away, and both report exactly what an empty room
#: reports. Scoring that as the best direction available meant the sweep
#: actively steered the car towards the only things it cannot see: into a
#: sofa, and into the corner where a curtain met one, with half the room open
#: behind it.
#:
#: So it ranks below anything measured as roomy and above anything measured as
#: close. An unknown is worth trying when the alternative is a wall you can
#: see, and not worth trying when there is real space elsewhere.
UNKNOWN_CM = 70.0

#: Clearance needed all round before turning on the spot.
#:
#: Every other manoeuvre is checked before it happens: the safety layer will
#: not drive forward without a forward reading. Rotating is checked by nothing
#: at all, and it is the one manoeuvre that moves parts of the car sideways --
#: the front corners swing out through space the sensor has never pointed at,
#: and the mecanum wheels stand proud of the chassis. So a turn decided while
#: something sits a few centimetres off the bow connects with it, having been
#: perfectly clear a moment earlier.
#:
#: Roughly the half-diagonal of the chassis. Anything closer than this in any
#: swept direction means backing off first, which costs a second and is the
#: difference between turning and grinding.
TURN_CLEARANCE_CM = 16.0
#: How long to back off for when there is not enough.
MAKE_ROOM_S = 0.8

#: Being blocked this many times inside this window means the ways out are not
#: working. The classic case is a corner: every direction that looks open is a
#: soft surface that is not, so each escape leads straight back.
#:
#: The window has to be short enough that ordinary patrolling does not fill it.
#: Measured in a cluttered room, the car meets something every seven seconds or
#: so, which at twenty-five seconds made "cornered" the normal state of affairs
#: -- it used the escape manoeuvre twice as often as it made an ordinary
#: decision. Twelve seconds means blocked at roughly twice the usual rate,
#: which is a signal rather than a description.
TRAPPED_BLOCKS = 3
TRAPPED_WINDOW_S = 12.0
#: What to do about it. Back out much further than usual, then turn most of the
#: way round -- the one direction that is certainly clear is where the car came
#: from.
TRAPPED_REVERSE_S = 1.6
TRAPPED_TURN_S = 2.2

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
#: And this close is past leaning away from. At a few centimetres the car is
#: already scraping along whatever it is, and continuing forward while curving
#: gently is how it stays there -- observed doing exactly that against a wall
#: for 25 seconds, announcing an avoidance that never happened.
SIDE_CLOSE_CM = 15.0
#: How hard to lean away. Raised to whatever actually turns this car before it
#: is used; see _enough_to_turn.
SIDE_AVOID_TURN = 0.35
SIDE_TURN_HOLD = 0.5
#: Side readings are taken while moving, so they describe somewhere the car
#: used to be. After this long the thing measured is behind it, and steering
#: away from it means steering into whatever is actually there now.
SIDE_MEMORY_S = 3.0
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
        self.unknown = set()        # angles that returned nothing at all
        self.blocks = []            # when recent sweeps happened
        self.avoid = None           # (angle, until) a way out that did not work
        self.follow = None          # second half of a two-part manoeuvre
        self.glance = None          # (angle, when to read it)
        self.glance_next = 0.0      # when to look away again
        self.glance_side = 1.0      # alternates, so both sides get looked at
        self.sides = {}             # side -> (cm, when it was measured)
        self.state = "idle"

    def start(self, robot):
        self.until = 0.0
        self.move = Velocity.STOP
        self.blocked_since = None
        self.scan = None
        self.clearance = {}
        self.unknown = set()
        self.blocks = []
        self.avoid = None
        self.follow = None
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

        # Some manoeuvres come in two parts -- back out of the corner, then
        # turn round -- and the second must not be interrupted by the first
        # tick of ordinary cruising deciding everything is fine again.
        if getattr(self, "follow", None):
            what, _ = self.follow
            self.follow = None
            if what == "decide":
                # Room made, so look again. Deciding on the sweep taken before
                # the reverse is what the reverse was supposed to invalidate:
                # the clearance that was too tight to turn in is still too
                # tight in a map of where the car used to be, so it backs off,
                # re-reads the same numbers, backs off again. Measured doing
                # exactly that for 147 seconds.
                #
                # Sweeping again also counts as another block, which is what
                # eventually escalates this to the escape manoeuvre rather than
                # letting it repeat.
                return self._start_looking(robot, now)
            if what == "turn_back":
                return self._commit(
                    robot, now,
                    Velocity(omega=random.choice((-1.0, 1.0))
                             * self._enough(robot, TURN_SPEED)),
                    TRAPPED_TURN_S, "cornered -- turning round")

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

        # Steer away from the nearer side, if either is close enough to matter
        # and was seen recently enough to still be beside the car. Both being
        # close is a gap, not an obstacle, and steering out of a gap is how you
        # hit the other side of it.
        left = self._side(-1, now)
        right = self._side(1, now)
        nearest = min(left, right)
        omega = 0.0
        note = ""
        if nearest < SIDE_ALERT_CM and abs(left - right) > 8.0:
            away = 1.0 if left < right else -1.0
            side = "right" if away > 0 else "left"
            if nearest < SIDE_CLOSE_CM:
                # Too close to lean away from while still driving at it. Stop
                # going forward and turn, the same as being blocked -- which
                # this is, just not in the direction the safety layer watches.
                self.sides = {}
                return self._commit(
                    robot, now, Velocity(omega=away * self._enough(robot, TURN_SPEED)),
                    SIDE_TURN_HOLD,
                    "%.0fcm to the %s, turning away" % (
                        nearest, "left" if away > 0 else "right"),
                )
            omega = away * self._enough(robot, SIDE_AVOID_TURN)
            note = " | easing %s off %.0fcm" % (side, nearest)

        aim = "" if not self.glance else " | looking %+.0f" % self.glance[0]
        self.state = "cruising" + aim + note
        return Velocity(vx=CRUISE, omega=omega)

    def _side(self, which, now):
        """How far away the last thing seen on that side was, if still relevant."""
        seen = self.sides.get(which)
        if seen is None or now - seen[1] > SIDE_MEMORY_S:
            return 999.0
        return seen[0]

    def _enough(self, robot, magnitude):
        """Raise a turn to something that will actually rotate the car.

        Below a measured duty these motors do not turn the chassis at all --
        rotating in place fights the rollers on all four wheels, and it costs
        more than driving straight. So a turn command under that floor is not
        a gentler turn, it is no turn, and asking for one produces a behaviour
        convinced it manoeuvred and a car that did not move. That is exactly
        what a 0.35 side-avoidance did at every speed limit worth using: 0.12
        of duty against a floor of 0.20.

        Raising it does not spend what the speed limit was protecting. The
        limit is about how fast the car crosses a room; a turn in place
        crosses nothing.

        With no measured floor -- nobody has run the calibration -- this does
        nothing, which is the same behaviour as before it existed.
        """
        floor = robot.calibration.get("turn_floor") if robot.calibration else 0.0
        if not floor:
            return magnitude
        return min(1.0, max(magnitude, floor / max(robot.speed_limit, 1e-6)))

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
                999.0 if reading is None or reading <= 0 else reading, now)
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
                robot, now,
                Velocity(omega=self._guess(robot) * self._enough(robot, TURN_SPEED)),
                TURN_HOLD_MIN, "blocked, picking a way round",
            )
        self.clearance = {}
        self.unknown = set()
        self.blocks = [t for t in self.blocks if now - t < TRAPPED_WINDOW_S]
        self.blocks.append(now)
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
        if reading is None or reading <= 0:
            # Heard nothing. That is not a measurement of open space, it is
            # the absence of a measurement, and indoors the commonest cause
            # is something soft standing right there.
            self.clearance[angle] = UNKNOWN_CM
            self.unknown.add(angle)
        else:
            self.clearance[angle] = min(reading, CLEAR_CM)
            self.unknown.discard(angle)

        if index + 1 < len(SCAN_ANGLES):
            self.scan = (index + 1, now + SCAN_DWELL)
            robot.pan.look(SCAN_ANGLES[index + 1])
            self.state = f"looking {self._name(SCAN_ANGLES[index + 1])}"
            return Velocity.STOP

        robot.pan.centre()
        self.scan = None
        return self._decide(robot, now)

    def _decide(self, robot, now):
        """Choose a way out of wherever the sweep just found the car."""
        profile = " ".join(
            "%+d:%s" % (int(a), "?" if a in self.unknown else "%.0f" % self.clearance[a])
            for a in sorted(self.clearance))

        if len(self.blocks) >= TRAPPED_BLOCKS:
            # Three sweeps inside half a minute means every way out chosen so
            # far has led straight back. Almost always a corner of soft things,
            # each of which reads as open and none of which is. Stop choosing
            # between them: the one direction known to be clear is the one the
            # car arrived from, so reverse a long way and turn most of the way
            # round.
            self.blocks = []
            self.avoid = None
            return self._commit(
                robot, now, Velocity(vx=-REVERSE_SPEED), TRAPPED_REVERSE_S,
                "cornered -- backing right out", follow=("turn_back", now))

        # Nothing checks a rotation before it happens, so check it here. The
        # sweep has just measured every direction the corners are about to
        # swing through, which is the one moment this is knowable.
        tightest = min(self.clearance.values(), default=999.0)
        if tightest < TURN_CLEARANCE_CM:
            return self._commit(
                robot, now, Velocity(vx=-REVERSE_SPEED), MAKE_ROOM_S,
                f"only {tightest:.0f}cm to turn in, backing off first | {profile}",
                follow=("decide", now))

        roomiest = max(self.clearance.values(), default=0.0)
        if roomiest < BOXED_IN_CM:
            return self._commit(
                robot, now, Velocity(vx=-REVERSE_SPEED), REVERSE_HOLD,
                f"boxed in ({roomiest:.0f}cm all round), backing out | {profile}",
            )

        # Only somewhere else is a candidate. Straight ahead is where the car
        # already points and has just been refused; it is measured because how
        # blocked the front is tells a doorway from a wall.
        #
        # Each candidate is scored by the room at it AND beside it. A single
        # clear angle flanked by obstacles is a gap the width of the beam, and
        # the car is wider than the beam -- picking the deepest reading rather
        # than the widest opening is how it kept aiming at slots it could not
        # fit through.
        angles = sorted(self.clearance)
        scores = {}
        for index, angle in enumerate(angles):
            if angle == 0.0:
                continue
            if self.avoid and angle == self.avoid[0] and now < self.avoid[1]:
                continue        # this way out was tried and did not work
            around = [self.clearance[angles[j]]
                      for j in (index - 1, index, index + 1)
                      if 0 <= j < len(angles)]
            scores[angle] = min(around)

        if not scores:
            return self._commit(
                robot, now,
                Velocity(omega=self._guess(robot) * self._enough(robot, TURN_SPEED)),
                TURN_HOLD_MIN, f"nowhere left to try | {profile}",
            )

        best_score = max(scores.values())
        # A tie is real -- a corridor looks the same both ways -- so break it
        # randomly rather than always choosing the same side of it.
        rivals = [a for a, s in scores.items() if s > best_score - 5.0]
        best = random.choice(rivals)
        room = self.clearance[best]

        # Remember it, so if this fails the next decision does not repeat it.
        self.avoid = (best, now + TRAPPED_WINDOW_S)

        direction = 1.0 if best > 0 else -1.0
        hold = TURN_HOLD_MIN + (TURN_HOLD_MAX - TURN_HOLD_MIN) * (abs(best) / 90.0)
        measured = "?" if best in self.unknown else "%.0fcm" % room
        return self._commit(
            robot, now,
            Velocity(omega=direction * self._enough(robot, TURN_SPEED)), hold,
            f"most room {self._name(best)} ({measured}) | {profile}",
        )

    def _commit(self, robot, now, velocity, seconds, state, follow=None):
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
        self.follow = follow
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
            "sides": {("left" if k < 0 else "right"): round(cm)
                      for k, (cm, _) in self.sides.items() if cm < 999},
        }
