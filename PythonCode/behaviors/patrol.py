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

import math
import random
import time

from control import vfh
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
#
# Thirty degrees apart rather than forty-five. The chooser grows every
# obstacle by the width of the car and then looks for gaps, and a gap can
# only be found between two samples that bracket it -- at forty-five degrees
# a doorway at sixty centimetres falls entirely between two readings. Thirty
# is about twice the beam width, which is as fine as the sensor can honestly
# support, and costs one more dwell than before.
SCAN_ANGLES = (-90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0)
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
#: How far a neighbouring angle must see before it will vouch for a no-echo
#: one. A soft thing near enough to swallow the ping is not the width of the
#: beam -- a sofa, a curtain, a coat -- so the angles either side of it
#: usually see it too. Neighbours that both see a long way are evidence the
#: silence is distance rather than absorption.
UNKNOWN_VOUCH_CM = 60.0
#: What a vouched-for silence is worth. Beyond the sensor's range is the most
#: open thing it can report, and scoring it below a wall at arm's length --
#: which is what happened -- steers the car away from the one direction with
#: nothing in it at all.
OPEN_UNKNOWN_CM = 250.0

#: Driving at something and not getting closer to it.
#:
#: The safety layer cannot catch this. It refuses to drive at something close,
#: and this is the case where the reading never becomes close -- the car is
#: pressed against something the beam reads past, or a wheel is on a cable, and
#: the distance ahead simply stops changing. Nothing else notices, because
#: every part of the system is being told what it expects to hear: the wheels
#: are commanded, the reading is plausible, the behaviour is content.
#:
#: Six seconds and six centimetres. The car creeps at two to three centimetres
#: a second when the safety layer is throttling it near an obstacle, so a
#: genuine slow approach still covers more than this; standing still covers
#: none of it.
NO_PROGRESS_S = 6.0
NO_PROGRESS_CM = 6.0

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
#
# Derived now, not guessed. Measured on this car at 15 by 28 centimetres,
# the half-diagonal is 15.9 -- which is what the guess happened to be, so
# nothing about the car changes today. What changes is that the number
# follows the chassis instead of being a comment claiming it does.
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
#: How much of that to actually hold, drawn anew on each escape.
#
# The full figure is about a half turn, which leaves the corner and then sends
# the car back along the ground it just covered. Something between a quarter
# and a half turn leaves just as surely and opens somewhere new, and varying
# it means a corner that catches the car twice does not throw it out along the
# same line both times.
TRAPPED_TURN_SPREAD = (0.5, 0.9)

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

#: What distance to record when a corner infrared fires.
#
# The reading is a bare yes-or-no, so it has to be given a number to sit in
# the same table as the ultrasonic.
#
# Measured on this car: seven to nine centimetres, tuned by turning the
# sensitivity up to just short of where it latches. That point is a ceiling
# rather than a setting. Turning the sensitivity past a point makes the output
# latch on permanently, because the threshold has dropped below the light the
# receiver picks up from its own emitter leaking across the board. The usable
# band is whatever fits between that floor and the echo from a real target,
# and the echo falls off with the square of the distance, so the band is
# narrow and no amount of adjustment widens it. That is a different sensor
# from the one the datasheets describe, and it changes what the thing is for.
# These modules are reflective -- an infrared LED, a photodiode, a comparator
# -- so the range they reach is a property of whatever is being looked at, not
# of the module. A white wall gives one figure and a dark sofa gives a
# fraction of it, because dark surfaces simply do not return the light.
#
# At eight centimetres there is still no avoiding, only stopping -- but the
# stopping is now comfortable rather than marginal. The car covers that in
# about a quarter of a second at cruise and the loop runs at 20Hz, so by
# the time this fires the useful responses are the abrupt ones. It is closer
# in kind to the accelerometer's collision detection than to the ultrasonic --
# a whisker, not a rangefinder, earning its place by being the one thing that
# reports just before contact rather than just after.
#
# Written below SIDE_CLOSE_CM so it lands on the stop-and-turn branch rather
# than the lean-gently-away one, which would be useless at this range.
IR_ASSUMED_CM = 8.0
REVERSE_SPEED = 0.5
REVERSE_HOLD = 0.7

#: Commanded to turn, and not turning. See control/turning.py for why this is
#: worth knowing and why nothing else in the car can tell.
#:
#: The rate has to be below this for the manoeuvre to count as stalled. Not
#: zero: the gyroscope has noise, the car rocks on its suspension, and a
#: figure a hair above zero would make every turn look fine. A real turn at
#: this car's turning speed is well over a hundred degrees a second, so there
#: is a wide gap to sit in.
STALL_RATE = 12.0
#: And it has to have been that way this long. A turn takes a moment to break
#: static friction, and declaring the first tenth of every turn a stall would
#: be worse than not looking.
STALL_AFTER = 0.7

#: Turns are held until the car has actually turned this far, rather than for
#: a fixed time, whenever the IMU is answering. The timed hold stays as the
#: fallback and as the deadline -- a turn that has not finished in twice the
#: time it should have is not going to.
TURN_TIMEOUT_FACTOR = 2.5
#: Degrees per second the hold times were written around, used to convert an
#: old hold into the angle it was meant to produce.
ASSUMED_TURN_RATE = 95.0


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
        self.progress = []          # (when, how far ahead) while cruising
        self.glance = None          # (angle, when to read it)
        self.glance_next = 0.0      # when to look away again
        self.glance_side = 1.0      # alternates, so both sides get looked at
        self.sides = {}             # side -> (cm, when it was measured)
        #: Every angle this car has looked at lately, with when. A sweep is
        #: five seconds of looking in different directions from a car that is
        #: not quite where it was, so the chooser is given the glances taken
        #: while cruising as well -- that is the nearest thing to the single
        #: instantaneous scan the algorithm was written for.
        self.seen = {}              # angle -> (cm, when, was_it_silence)
        #: The direction chosen last time, so the chooser can prefer it. Two
        #: ways out that score within noise of each other used to be settled
        #: by a coin toss on every decision, which is a car twitching between
        #: them rather than taking either.
        self.last_choice = None
        #: (turned-at-the-start, degrees wanted) while a closed-loop turn is
        #: running, or None when the current manoeuvre is timed.
        self.turn_goal = None
        #: When the current manoeuvre started, for the stall check.
        self.move_since = 0.0
        self.stalls = 0
        #: The most recent world, so the parts of the sweep that run two
        #: calls deep can reach it. See _decide.
        self.world_now = None
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
        self.progress = []
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
        self.world_now = world
        self._note_infrared(world, now)

        if self.scan is not None:
            return self._sweep(robot, world, now)

        # Mid-manoeuvre from an earlier tick: see it through -- unless it is
        # already done, or is visibly not happening.
        if now < self.until:
            done, why = self._manoeuvre_over(world, now)
            if not done:
                return self.move
            self.until = 0.0
            self.turn_goal = None
            if why:
                self.state = why

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
                # Far enough round to leave the corner, not so far as to set
                # off back down the line the car came in on. A half turn does
                # escape, and then retraces known ground -- which for a robot
                # with no map is the one thing it can be sure is a waste. The
                # amount is drawn fresh each time so that two escapes from the
                # same corner do not follow the same line out.
                held = TRAPPED_TURN_S * random.uniform(*TRAPPED_TURN_SPREAD)
                return self._commit(
                    robot, now,
                    Velocity(omega=random.choice((-1.0, 1.0))
                             * self._enough(robot, TURN_SPEED)),
                    held, "cornered -- turning away", world=world,
                    degrees=held * ASSUMED_TURN_RATE)

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

    def _manoeuvre_over(self, world, now):
        """Should the committed manoeuvre stop before its time is up?

        Two reasons, and they are the two things the IMU is actually good
        for. Either the turn has gone as far as it was asked to -- which a
        clock cannot know, because how fast this car turns depends on the
        speed limit, the battery and the carpet -- or it is not turning at
        all, which is a wheel on a cable, a castor against a chair leg, or a
        commanded duty below what moves this car. All of those look identical
        from inside the control loop, where the wheels are being told what to
        do and everything downstream is content.

        Returns (stop, note). A note of None means stop quietly.
        """
        if self.turn_goal is not None and world.turn_rate is not None:
            mark, wanted = self.turn_goal
            so_far = world.turned - mark
            if abs(so_far) >= abs(wanted):
                return True, None       # the angle was reached; say nothing

        if (abs(self.move.omega) > 1e-6 and world.turn_rate is not None
                and now - self.move_since > STALL_AFTER
                and abs(world.turn_rate) < STALL_RATE):
            # Commanded to turn and not turning. Treated as a block so that
            # it feeds the same escalation as everything else: three of these
            # inside the window and the car stops trying to turn its way out
            # and reverses instead.
            self.stalls += 1
            self.blocks.append(now)
            self.blocks = [b for b in self.blocks if now - b < TRAPPED_WINDOW_S]
            return True, ("commanded %.2f and turning %.0f deg/s -- stalled "
                          "(%d so far)" % (self.move.omega, world.turn_rate,
                                           self.stalls))
        return False, None

    # -- driving along, looking around -------------------------------------

    def _cruise(self, robot, world, now):
        """Drive forward, glancing sideways, leaning away from what is there."""
        if robot.pan is not None:
            self._glance(robot, world, now)

        if self._going_nowhere(world, now):
            # Not blocked as far as the safety layer is concerned, and not
            # getting anywhere either. Treat it as blocked, because it is.
            self.progress = []
            return self._start_looking(robot, now)

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

    def _going_nowhere(self, world, now):
        """True if the way ahead has stopped getting closer while driving at it.

        Only real readings count. A stretch of nothing-came-back does not
        change either, and calling that stuck would mean an open room and a
        sofa are the same thing -- which is the mistake this sensor invites at
        every level it is used.
        """
        ahead = world.forward_cm
        if ahead is None or ahead <= 0:
            self.progress = []
            return False
        self.progress.append((now, ahead))
        # Keep more history than the window being judged. Trimming to exactly
        # NO_PROGRESS_S and then asking whether the samples span NO_PROGRESS_S
        # is a question that can never be answered yes: after the trim the
        # oldest sample is at most that old, and floating point makes "at most"
        # into "just under". Written that way it looked right and never fired
        # once, through three minutes of a car with its wheels turning and a
        # cable round the axle.
        self.progress = [p for p in self.progress if now - p[0] <= NO_PROGRESS_S * 2]
        recent = [p for p in self.progress if now - p[0] <= NO_PROGRESS_S]
        if len(recent) < 10 or now - recent[0][0] < NO_PROGRESS_S * 0.9:
            return False
        values = [p[1] for p in recent]
        return max(values) - min(values) < NO_PROGRESS_CM

    def _note_infrared(self, world, now):
        """Let the corner sensors write into the same table as the ultrasonic.

        This is the whole of the integration, and it is deliberately small.
        Rather than a second set of rules that has to be kept in step with the
        first, the infrared writes into `sides` -- the one place this behaviour
        keeps what it believes is beside the car -- and every rule already
        built on that table starts working from it for free: leaning away, the
        stop-and-turn when something is close, the memory that keeps an
        obstacle real for a few seconds after it leaves view.

        The two sensors are complementary in exactly the way that matters
        here. The ultrasonic is long, narrow, quantitative and slow: it glances
        aside once every GLANCE_EVERY_S and the car covers real ground blind in
        between. The infrared is short, wide, binary and continuous. Pouring
        the second into the table the first fills is what closes that gap.
        """
        prox = world.proximity
        if prox is None:
            return
        if prox.left:
            self.sides[-1] = (IR_ASSUMED_CM, now)
        if prox.right:
            self.sides[1] = (IR_ASSUMED_CM, now)

    def _resolve_unknowns(self):
        """Decide what each no-echo angle was, using the ones beside it.

        Silence from an ultrasonic has two causes that look identical: open
        space past the four metres it reaches, and something soft close enough
        to swallow the ping. Treating every silence as a soft thing was the
        fix for a car that drove into sofas, and it brought its own fault --
        the emptiest direction in the room scores below a wall an arm's length
        away, so the car turns away from the only way that was actually open.

        The angles either side settle it. A sofa is wider than the beam and
        shows up in its neighbours; four metres of empty floor does not. So a
        silence flanked by long readings is taken as distance, and one with
        anything close beside it stays suspect.

        Used for choosing a direction and nowhere else. The checks that ask
        whether there is room to turn keep the pessimistic figure, because
        being wrong there means turning into something rather than declining
        to drive somewhere.
        """
        angles = sorted(self.clearance)
        resolved = dict(self.clearance)
        for index, angle in enumerate(angles):
            if angle not in self.unknown:
                continue
            beside = [self.clearance[angles[j]]
                      for j in (index - 1, index + 1)
                      if 0 <= j < len(angles) and angles[j] not in self.unknown]
            # Nothing measured anywhere means the whole sweep heard silence,
            # which indoors is a room with no wall within range in any
            # direction -- or a sensor that has stopped answering, and that is
            # caught separately by the rule about going deaf.
            if not beside or min(beside) >= UNKNOWN_VOUCH_CM:
                resolved[angle] = OPEN_UNKNOWN_CM
            else:
                # Stated rather than left as it was found, so this returns a
                # figure for every angle whatever the caller stored.
                resolved[angle] = UNKNOWN_CM
        return resolved

    def _side(self, which, now):
        """How far away the last thing seen on that side was, if still relevant."""
        seen = self.sides.get(which)
        if seen is None or now - seen[1] > SIDE_MEMORY_S:
            return 999.0
        return seen[0]

    def _enough(self, robot, magnitude):
        """See Robot.enough_to_turn. Manual driving needs the same rule now."""
        return robot.enough_to_turn(magnitude)

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
        if world.range_blind:
            # Looking at the car itself. Scoring that as an obstacle is what
            # condemned a whole side of the chassis every sweep.
            reading = None
        if reading is None or reading <= 0:
            # Heard nothing. That is not a measurement of open space, it is
            # the absence of a measurement, and indoors the commonest cause
            # is something soft standing right there.
            self.clearance[angle] = UNKNOWN_CM
            self.unknown.add(angle)
        else:
            self.clearance[angle] = min(reading, CLEAR_CM)
            self.unknown.discard(angle)
        self._remember(angle, self.clearance[angle], now, angle in self.unknown)

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
        # Kept from the last tick by update(), because the sweep machinery
        # reaches here through two layers that never needed a world before
        # and threading one through all of them to reach one call is worse
        # than remembering it.
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
        if tightest < self._turn_clearance(robot):
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
        samples = self._samples(now)
        forbid = ()
        if self.avoid and now < self.avoid[1]:
            # A way out that was tried and led straight back. Rule out the
            # whole arc the car would be aimed into, not the one exact angle
            # -- at five-degree resolution "the same direction" is a span.
            forbid = tuple(a for a in vfh.fine_angles((-90.0, 90.0))
                           if abs(a - self.avoid[0]) <= 20.0)

        best, trace = vfh.choose(
            samples, self._radius(robot), previous=self.last_choice,
            forbid=forbid)

        if best is None:
            # Every direction is inside something once the car is grown to
            # its own width. Backing out is the only move left that does not
            # involve swinging a corner through an obstacle.
            self.last_choice = None
            return self._commit(
                robot, now, Velocity(vx=-REVERSE_SPEED), REVERSE_HOLD,
                f"no gap wide enough -- {trace} | {profile}",
            )

        self.last_choice = best
        # Remember it, so if this fails the next decision does not repeat it.
        self.avoid = (best, now + TRAPPED_WINDOW_S)

        if abs(best) < 1e-6:
            # The chooser can pick straight ahead when the thing that blocked
            # the car has gone -- somebody stepped out of the way, or the
            # reading was a one-off. Driving is then the right answer and
            # turning zero degrees for a second is not.
            self.blocked_since = None
            return self._commit(
                robot, now, Velocity(vx=CRUISE), TURN_HOLD_MIN,
                f"clear ahead after all -- {trace} | {profile}",
            )

        direction = 1.0 if best > 0 else -1.0
        hold = TURN_HOLD_MIN + (TURN_HOLD_MAX - TURN_HOLD_MIN) * (abs(best) / 90.0)
        # The angle is the whole point of choosing one: turn to it, rather
        # than turning for however long that angle used to take.
        return self._commit(
            robot, now,
            Velocity(omega=direction * self._enough(robot, TURN_SPEED)), hold,
            f"going {self._name(best)} -- {trace} | {profile}",
            world=self.world_now, degrees=best,
        )

    # -- what the chooser is given ----------------------------------------

    def _turn_clearance(self, robot):
        """Radius the corners sweep through when turning on the spot.

        Half the diagonal of the chassis, which is where the furthest corner
        is from the centre of rotation, and therefore exactly how much room
        a rotation needs in every direction it has measured. No margin on
        top: unlike driving at a gap, this is checked against a sweep taken
        a moment ago from this spot, so the measurement and the manoeuvre
        describe the same place.
        """
        if robot.calibration is None:
            return TURN_CLEARANCE_CM
        width = robot.calibration.get("car_width_cm") or 0.0
        length = robot.calibration.get("car_length_cm") or 0.0
        if width <= 0.0 or length <= 0.0:
            return TURN_CLEARANCE_CM
        return math.hypot(width / 2.0, length / 2.0)

    def _radius(self, robot):
        """Half the car plus a margin: what every obstacle is grown by."""
        if robot.calibration is None:
            return 15.0
        width = robot.calibration.get("car_width_cm") or 22.0
        margin = robot.calibration.get("car_margin_cm") or 0.0
        return width / 2.0 + margin

    def _remember(self, angle, centimetres, when, silent):
        self.seen[angle] = (centimetres, when, silent)

    def _samples(self, now):
        """Every direction worth telling the chooser about.

        This sweep first, then anything glanced at recently that the sweep
        did not cover. Old readings are dropped rather than aged gracefully:
        a measurement taken while the car was somewhere else is not a worse
        measurement of here, it is a measurement of somewhere else.
        """
        resolved = self._resolve_unknowns()
        samples = dict(resolved)
        for angle, (centimetres, when, silent) in self.seen.items():
            if angle in samples:
                continue
            if now - when > SIDE_MEMORY_S:
                continue
            samples[angle] = OPEN_UNKNOWN_CM if silent else centimetres
        return samples

    def _commit(self, robot, now, velocity, seconds, state, follow=None,
                world=None, degrees=None):
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
        self.move_since = now
        self.turn_goal = None
        if degrees is not None and world is not None and world.turn_rate is not None:
            # Closed loop: hold until the car has turned this far, with the
            # clock kept on as a deadline. A turn that has not finished in
            # well over the time it should have taken is not going to, and
            # hanging on a goal that will never arrive is worse than a turn
            # of the wrong size.
            self.turn_goal = (world.turned, degrees)
            seconds = seconds * TURN_TIMEOUT_FACTOR
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
