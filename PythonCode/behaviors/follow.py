"""Follow whatever object you name.

The original `main_tennis_tracking.py` could chase exactly one thing: a
yellow-ish circle, found with a Hough transform and an HSV colour test. Because
detection is now a separate concern, the same control logic works on any of the
detector's 80 classes -- steer from how far off centre the target sits, drive
from how big it looks.

On a mecanum chassis this also gains an option the original could not express:
`orbit` strafes sideways while keeping the target centred, circling it instead
of only approaching head-on.

Two sensors, two questions. The camera says *which way* the target is, which
is what a camera is good at. The ultrasonic says *how far*, which is what a
rangefinder is good at and what a camera is worst at. Judging distance from
how big something looks works only while the whole of it is inside the frame,
and this camera sits 25cm off the floor -- follow a person and it sees legs,
which stop growing once they reach the top and bottom edges. The box then
says "still far away" at exactly the moment the car is about to arrive.

The apparent-size estimate is kept as a fallback rather than deleted, because
an ultrasonic does not hear everything. A small soft ball can return no echo
at all, and a follow behaviour that stops working when one sensor goes quiet
is worse than one that gets vaguer.

## What changed, and what it looked like from outside

This used to read the newest detection every tick and hand it straight to a
PID. Three complaints, all the same bug wearing different clothes:

*"It wobbles left and right, like it is being very lively."* Two causes,
and the measurement put them in the opposite order to the guess. The small
one: the PID's D term differentiated the raw box position, and dividing a
measured 0.0005 of jitter by a 0.05s tick and the old effective gain of six
manufactured about 0.086 of turn authority out of a ball that was not
moving. The large one: at the confidence gate it was set to, the detector
found that ball in two looks out of forty, so the loop was mostly stopped
and occasionally issuing a command at full strength. Intermittent looks
livelier than noisy.

*"It goes forward in fits and starts."* Forward motion was switched off
outright whenever the target sat more than `ahead_deadband` off centre, so
the car turned, started, drifted past the edge of the band, stopped, turned
again.

*"With two balls, which one does it follow?"* Whichever measured larger
**this frame**, decided afresh every tick. It could swap between two similar
targets without anything noticing, and nothing in the code could have told
you which one it was on.

All three are now one answer: a `TargetTrack` sits between the detector and
the controllers. It keeps an identity, it carries velocity as part of its
state so the D term never has to differentiate anything, and it coasts on
its own prediction through the frames where the detector blinks.
"""

from settings import Spec, group

import time

import cv2

from control import Aim, PID, TargetTrack
from hal import Velocity

from .base import Behavior
from .reacquire import Reacquire

#: How far to sit from whatever is being followed, in centimetres. A real
#: measurement in a unit you can hold a ruler up to, rather than a fraction of
#: a frame that means something different for every target and every lens.
#: Kept well clear of the safety layer's stop distance: closer than that and
#: forward motion is refused, and the car would sit shuffling at a target it
#: has been told to approach and is not allowed to reach.
STANDOFF_CM = 45.0
DISTANCE_DEADBAND_CM = 8.0

#: Fraction of frame height the target should fill. Only used when there is no
#: distance reading to be had.
TARGET_FILL = 0.45
FILL_DEADBAND = 0.06
CENTRE_DEADBAND = 0.05

#: The rangefinder looks where the car points, not where the camera is looking,
#: so its reading is only about the target while the target is roughly ahead.
#: Off to one side, turn to face it first and do not pretend to know how far
#: away it is -- which also keeps the head forward, where the safety layer
#: needs it to allow any forward motion at all.
#:
#: This now gates *which estimate of distance is believed*, not whether the
#: car may move at all; see `APPROACH_FALLOFF`.
AHEAD_DEADBAND = 0.25

#: How sharply approach speed falls off as the target moves off centre.
#:
#: The old code made this a cliff: inside the band, full speed; outside it,
#: nothing. A cliff in a control law is a thing the car will sit on, because
#: the moment it starts moving it also starts turning, and turning is what
#: carries the target back across the edge. Hence the stop-start.
#:
#: A slope instead. At dead centre nothing is lost; at a quarter off centre
#: a quarter of the speed is; at the frame edge the car is only turning.
#: Travelling mostly-forwards while still swinging round wastes very little
#: -- the component of the motion that is not towards the target goes as the
#: cosine, which is 0.97 at fifteen degrees -- so there was never much to
#: buy by refusing outright.
APPROACH_FALLOFF = 1.0

#: How far off centre is worth a step, in degrees.
#:
#: Not a tuning knob. It is the scatter on the smallest move this chassis
#: can make, and a loop whose deadband is narrower than that scatter will
#: hunt at any gain, because it is correcting errors it cannot make and
#: then correcting its failure to make them.
#:
#: Measured today:
#:
#:     one control tick, 17.7Hz at 30 deg/s        1.7 deg
#:     coast after a braked step, median           3.7 deg
#:     smallest stop-to-stop move                 ~5.4 deg
#:     that coast across sixteen runs       -0.9 .. 8.3 deg
#:
#: So the scatter is about plus or minus 4.6 degrees and the deadband has
#: to clear it. At 7 degrees -- 56% of this camera's 12.5 degree half
#: field -- an overshoot lands inside the band and the car stops, which
#: is the whole point: overshoot only matters if you then correct it.
#:
#: The job this serves is keeping the ball in the middle of the picture,
#: not in the middle of the picture to within a degree. Somebody with the
#: remote control does the first perfectly well and has never attempted
#: the second. If 7 degrees looks loose on the video, that is what a
#: controller that is not fighting its own actuator looks like.
AIM_DEADBAND_DEG = 7.0

#: How far ahead of a moving target to aim, in seconds.
#:
#: A step takes a tenth of a second and the look after it another fifth,
#: so a target that is moving will not be where it was by the time the car
#: gets there. The velocity comes from the estimator, which carries it as
#: state -- the reason it is there at all.
LOOK_AHEAD_S = 0.3

MAX_FORWARD = 0.6
MAX_TURN = 0.8
MAX_STRAFE = 0.5

#: Looks without the target before giving up, rather than driving on blind.
#: Larger than it used to be because the track now coasts through the gap on
#: a prediction instead of simply holding still, so the frames are useful
#: rather than merely tolerated.
LOST_PATIENCE = 12

CHASE = group(
    'follow',
    standoff_cm=Spec(
        STANDOFF_CM, unit='cm', bounds=(5.0, 300.0),
        help='How far behind the target it tries to sit when a range reading is available.'),
    distance_deadband_cm=Spec(
        DISTANCE_DEADBAND_CM, unit='cm', bounds=(0.0, 100.0),
        help='How far off that it tolerates before correcting.'),
    target_fill=Spec(
        TARGET_FILL, unit='', bounds=(0.05, 1.0),
        help='How much of the frame the target should fill when no range reading exists.'),
    fill_deadband=Spec(
        FILL_DEADBAND, unit='', bounds=(0.0, 0.5),
        help='How far off that fraction it tolerates.'),
    centre_deadband=Spec(
        CENTRE_DEADBAND, unit='', bounds=(0.0, 0.5),
        help='How far off centre the target may drift before it turns.'),
    ahead_deadband=Spec(
        AHEAD_DEADBAND, unit='', bounds=(0.0, 1.0),
        help='How far off centre the target may be and still have the '
             'rangefinder believed about it. Beyond this the head is '
             'pointing at something else, so apparent size is used instead.'),
    approach_falloff=Spec(
        APPROACH_FALLOFF, unit='', bounds=(0.0, 3.0),
        help='How much approach speed is given up for being off centre. '
             'Zero drives at full speed whatever the bearing; one gives up '
             'all of it at the frame edge; higher turns more before moving.'),
    aim_deadband_deg=Spec(
        AIM_DEADBAND_DEG, unit='deg', bounds=(0.2, 45.0),
        help='How far off centre the target must be before the car turns '
             'at all. Below roughly one step there is nothing it can do '
             'about the difference, and trying is how it hunts.'),
    look_ahead_s=Spec(
        LOOK_AHEAD_S, unit='s', bounds=(0.0, 2.0),
        help='How far ahead of a moving target to aim, to allow for the '
             'step and the look after it. Zero aims at where it was.'),
    max_forward=Spec(
        MAX_FORWARD, unit='', bounds=(0.0, 1.0),
        help='Fastest it will chase.'),
    max_turn=Spec(
        MAX_TURN, unit='', bounds=(0.0, 1.0),
        help='Hardest it will turn while chasing.'),
    max_strafe=Spec(
        MAX_STRAFE, unit='', bounds=(0.0, 1.0),
        help='Hardest it will slide sideways while chasing.'),
    lost_patience=Spec(
        LOST_PATIENCE, unit='frames', bounds=(1.0, 200.0),
        help='Looks without a sighting before it gives up. Until then it '
             'keeps turning towards where the target was going.'),
)


class RangeGate:
    """Refuse a distance that jumps, until it has kept jumping for a while.

    Measured against a wall while driving at 10cm/s: the ultrasonic every
    so often hears the far side of the room instead, and reads 75-86cm for
    up to four consecutive samples while the wall is 45cm away. The
    distance controller's D term turned that into a lurch.

    Nothing real in front of this car moves 15cm in a twentieth of a
    second, so a jump that large is held off. If it persists it is real --
    the ball was picked up, or something stepped in -- and is accepted.
    Only the follower is filtered; the safety layer keeps the raw reading,
    which is the conservative choice for it.
    """

    JUMP_CM = 15.0
    PERSIST_S = 0.6

    def __init__(self):
        self.reset()

    def reset(self):
        self.value = None
        self._odd_for = 0.0

    def update(self, reading, dt):
        if reading is None:
            self.reset()
            return None
        if self.value is None or abs(reading - self.value) <= self.JUMP_CM:
            self.value = reading
            self._odd_for = 0.0
            return reading
        self._odd_for += max(0.0, dt or 0.0)
        if self._odd_for >= self.PERSIST_S:
            self.value = reading
            self._odd_for = 0.0
            return reading
        return self.value


#: Only drive forwards or backwards with the target this near the middle.
#:
#: Distance comes from apparent size whenever the rangefinder cannot be
#: believed, and a ball near the edge of this narrow picture is clipped by
#: it -- so it looks smaller, so it looks further away, so the car drives at
#: it. On the car that is exactly what happened: at offsets of 0.8 it drove
#: forward at "fill 0.29" and pushed the ball out of sight. At 45cm the ball
#: spans about a third of the half-width, so past 0.6 it is starting to be
#: cut off. Turn first; the distance can wait one step.
DRIVE_WITHIN = 0.6


class FollowBehavior(Behavior):
    name = "follow"
    label = "Follow object"
    description = "Locks onto a chosen object, keeps it centred and at distance."
    requires = ("motors", "camera", "detector")
    wants_detection = True
    accepts_target = True

    def __init__(self, target="person"):
        self.target = target
        self.orbit = 0.0            # -1 .. 1, sideways circling (mecanum only)
        #: None means "whatever the settings page says". Only a deliberate
        #: configure() pins it, so dragging the slider works, and a value
        #: somebody set for this run is not silently overwritten by one
        #: they changed for the next.
        self._standoff_override = None

        #: Built in start(), where the robot -- and so the calibration --
        #: is in reach. A behaviour is constructed long before it is given
        #: a car.
        self.aim = None
        self.half_fov = None
        #: Gain is per centimetre: 35cm too far asks for 0.7, which the clamp
        #: below brings to CHASE.max_forward.
        self.distance_pid = PID(kp=0.02, kd=0.005, setpoint=CHASE.standoff_cm)
        self.forward_pid = PID(kp=2.0, kd=0.2, setpoint=CHASE.target_fill)
        self.range_gate = RangeGate()

        self.track = self._new_track()
        #: Where to look when the target has gone. Owned here rather than
        #: by the track, because losing something is a fact and deciding
        #: to go and look for it is intent, and this layer is the one that
        #: is allowed to have intentions.
        self.search = Reacquire()
        #: Heading at the last real sighting; see update().
        self._seen_at = 0.0
        self.state = "waiting"

    @property
    def standoff_cm(self):
        return (CHASE.standoff_cm if self._standoff_override is None
                else self._standoff_override)

    def _new_track(self):
        """One patience, not two.

        `TargetTrack` has its own default for how long to coast, for the
        benefit of anything else that uses one. Here the number the owner
        can see on the settings page wins, so there is exactly one knob
        rather than two that disagree.
        """
        return TargetTrack(self.target, patience=CHASE.lost_patience)

    def configure(self, **kwargs):
        if kwargs.get("target"):
            self.target = str(kwargs["target"]).strip().lower()
            self._reset()
        if "standoff_cm" in kwargs:
            try:
                self._standoff_override = max(
                    25.0, min(200.0, float(kwargs["standoff_cm"])))
            except (TypeError, ValueError):
                pass
        if "orbit" in kwargs:
            try:
                self.orbit = max(-1.0, min(1.0, float(kwargs["orbit"])))
            except (TypeError, ValueError):
                pass
        return self.status()

    def _reset(self):
        self.track = self._new_track()
        self.search.found()
        self._seen_at = 0.0
        if self.aim is not None:
            self.aim.forget()
        self.distance_pid.reset()
        self.forward_pid.reset()
        self.range_gate.reset()

    def _measure(self, robot):
        """Read what was measured about this car's camera and chassis."""
        calibration = getattr(robot, "calibration", None)
        self.half_fov = calibration.get("camera_half_fov_deg") if calibration else None
        rate = calibration.get("turn_rate_deg_s") if calibration else None
        coast = calibration.get("turn_coast_deg") if calibration else None
        self.aim = Aim(rate, coast_deg=coast or 0.0)

    def _step_for(self, offset):
        """How far to turn for a target this far across the picture.

        Returns (degrees, worth_doing).

        With the camera measured, the error is an angle and the question
        is whether that angle is bigger than the smallest turn this car
        can make. Without it the error cannot be put in degrees at all --
        the two common lenses for this sensor differ by a factor of two
        and a half, and this car's is neither, so there is no honest
        default -- and the judgement has to happen where it can, in the
        picture, with the car creeping in minimum steps.

        The first version of this compared a creep step against a deadband
        in degrees. The step was smaller than the deadband, so no step was
        ever requested and an uncalibrated car sat perfectly still while
        believing it was following something.
        """
        if self.half_fov:
            degrees = offset * self.half_fov
            return degrees, abs(degrees) >= CHASE.aim_deadband_deg

        # A step big enough to be worth taking, judged in the picture.
        nudge = (self.aim.smallest_deg * 1.6) if self.aim else 2.5
        return (nudge if offset > 0 else -nudge,
                abs(offset) > CHASE.centre_deadband)

    @property
    def box(self):
        """The last box seen, for the overlay. Read off the track so there
        is one place that knows what is being followed."""
        return self.track.box

    def start(self, robot):
        self._measure(robot)
        self._reset()
        self.state = "waiting"

    def stop(self, robot):
        self.track.forget()

    def update(self, robot, world):
        width, height = world.frame_size
        if not width or not height:
            self.track.forget()
            self.state = "no camera"
            return Velocity.STOP

        if self.aim is None:
            # update() without start(), which happens in tests and would
            # otherwise mean no steering at all rather than a loud failure.
            self._measure(robot)

        # Live setpoints, so the page can tune the loop while it runs.
        self.distance_pid.setpoint = self.standoff_cm
        self.forward_pid.setpoint = CHASE.target_fill

        self.track.update(world.detections, world.frame_size, world.dt)
        if self.track.fresh:
            # The heading the target was last actually seen from, which is
            # not the heading at which the track is declared dead: it coasts
            # for up to lost_patience ticks first, and the car keeps turning
            # throughout. Anchoring the search on the later of the two sends
            # it back to where the target already was not.
            self._seen_at = world.turned

        if not self.track.alive:
            # Either never seen, or coasted for longer than it is worth.
            if (self.track.seen == 0 and not self.search.searching
                    and not self.search.gave_up):
                # Told to follow something and no sign of it. The search
                # was written for targets that had been lost and was
                # never offered one that had not yet been found, so a car
                # started with the ball at the edge of the picture -- a
                # sliver too clipped to be acquired, which is exactly the
                # case worth turning towards -- sat still indefinitely.
                #
                # No bias either way: there is no last sighting to lean
                # on, so it opens out from where it is standing.
                #
                # Once, not for ever. Without the gave_up guard the end
                # of one search is immediately the start of the next --
                # the target has still never been seen, so the condition
                # above is still true -- and the car turns until somebody
                # switches it off. `found()` clears the flag, so a target
                # that turns up later is chased normally.
                self.search.lost(0.0, 0.0, world.turned,
                                 world.turn_rate is not None)
            elif self.track.seen > 0:
                # It was there a moment ago, and the commonest reason it is
                # not there now is that this car turned away from it. Start
                # a search anchored on the heading it was last seen from,
                # before forgetting the belief that says which way it went.
                #
                # `turn_rate` is None when the gyro is not answering, which
                # is not the same as not turning -- the search has to know
                # the difference, because without a heading it cannot close
                # the loop and has to sweep blind instead.
                self.search.lost(self.track.offset, self.track.drift,
                                 self._seen_at, world.turn_rate is not None)
            # Forgetting rather than keeping a stale belief means the next
            # sighting starts a clean track instead of being gated against
            # a prediction made a long time ago from somewhere else.
            self.track.forget()

            turning = self.search.steer(world.turned, world.dt)
            if turning is None:
                self.state = (f"lost the {self.target}"
                              if self.search.gave_up
                              else f"no {self.target} in view")
                return Velocity.STOP
            self.state = (self.search.describe(self.target)
                          if self.track.seen > 0 or self.search.searching
                          else f"no {self.target} in view")
            # Camera only. Driving towards something nobody can currently
            # see is a different and much worse idea than looking for it.
            return Velocity(omega=turning)

        self.search.found()
        offset = self.track.offset
        drift = self.track.drift

        # -- turn: step towards it, then stop and look ---------------------
        #
        # Not a continuous command any more. Measured on this car with the
        # ball in front of it: the smallest turn it can make is 1.7
        # degrees, it turns at 46 degrees a second once moving, and the
        # camera sees 16 degrees either side of the nose. A ball a quarter
        # of the way to the edge is four degrees off, which the car covers
        # in under a tenth of a second -- less time than the camera, the
        # detector and this loop take to notice. Feedback that arrives
        # describing more rotation than the error it was correcting makes
        # a loop that oscillates at any gain, which is why nothing done to
        # the gains helped and why this was never a tuning problem.
        now = time.monotonic()
        if self.aim.ready(now) and self.track.fresh:
            # Aim where it will be, not where it was: a step and the look
            # after it take half a second, and the velocity is already
            # carried by the estimator.
            ahead = offset + drift * CHASE.look_ahead_s
            degrees, worth_doing = self._step_for(ahead)
            if worth_doing:
                self.aim.request(degrees, world.turned, now)
        omega = self.aim.omega(now, world.turned)
        if omega:
            # The aim commands the measured floor, below which these
            # motors do not rotate the chassis at all -- and then the
            # governor scales whatever a behaviour asks for by the speed
            # limit, which at 0.4 turns that floor into 0.08 and the car
            # into an ornament. The trace of this showed the step counter
            # climbing from 1 to 9 over nine seconds with the gyro
            # motionless at -2.6 degrees.
            #
            # Robot.enough_to_turn does the arithmetic that undoes it,
            # and exists for this: patrol calls it, manual driving calls
            # it, and this never has.
            lifted = robot.enough_to_turn(abs(omega))
            omega = lifted if omega > 0 else -lifted
        omega = max(-CHASE.max_turn, min(CHASE.max_turn, omega))

        # -- drive: hold a real distance, and fall back to apparent size ----
        ahead = abs(offset) < CHASE.ahead_deadband
        measured = self.range_gate.update(
            world.range_cm
            if (world.range_cm is not None and world.range_cm > 0
                and world.range_forward and ahead)
            else None,
            world.dt)

        vx = 0.0
        if measured is not None:
            # PID error is (setpoint - measured), which goes negative when the
            # target is further away than wanted -- and further away is exactly
            # when to drive forward. Flip it, as the turn above does.
            if abs(measured - self.standoff_cm) > CHASE.distance_deadband_cm:
                vx = -self.distance_pid.update(measured, world.dt)
            how = f"{measured:.0f}cm"
        else:
            # No usable echo, or the target is off to one side where the
            # rangefinder is pointed at something else entirely. The camera's
            # estimate is worse, and worse is not nothing -- which is a change
            # from the old behaviour, where being off to one side meant not
            # moving at all and produced the stop-start.
            fill = self.track.size.value
            if fill is None:
                how = "no size yet"
            else:
                if abs(CHASE.target_fill - fill) > CHASE.fill_deadband:
                    vx = self.forward_pid.update(
                        fill, world.dt, rate=self.track.size.rate)
                how = ("fill %.2f (%s)"
                       % (fill, "off to one side" if not ahead else "no echo"))

        # Approach along the line of sight rather than across it. A slope,
        # not the cliff this used to be.
        facing = max(0.0, 1.0 - CHASE.approach_falloff * abs(offset))
        vx *= facing

        if not self.track.fresh:
            # Coasting on a prediction. Turning towards where the target was
            # going is a reasonable bet and costs nothing if it is wrong;
            # driving at something nobody can currently see is neither.
            vx = 0.0

        if abs(offset) > DRIVE_WITHIN:
            vx = 0.0

        if vx:
            # The same fault the turn had: the governor scales this by the
            # speed limit, and at 0.4 everything the distance controller
            # asks for inside 25cm of error arrives at the wheels under the
            # measured 0.20 floor. Measured: the car does not move at 0.15
            # in either direction. The facing falloff above still shapes
            # speed over the floor; under it there is no slower, only still.
            lifted = robot.enough_to_drive(abs(vx))
            vx = lifted if vx > 0 else -lifted

        vx = max(-CHASE.max_forward, min(CHASE.max_forward, vx))
        vy = self.orbit * CHASE.max_strafe

        self.state = (
            "%s %.2f | off %+.2f | %s%s"
            % (self.target, self.track.confidence, offset, how,
               "" if self.track.fresh else " | coasting %d" % self.track.missed)
        )
        return Velocity(vx=vx, vy=vy, omega=omega)

    def overlay(self, frame):
        """Draw what is believed as well as what was seen.

        Three marks, because the difference between them is the whole point
        of the estimator and is otherwise invisible: the red box is the raw
        sighting, the green crosshair is the filtered position the steering
        actually uses, and the yellow line is where the car is pointing. A
        green cross that sits still while the red box shivers around it is
        the fix working, visible at a glance on the video.
        """
        height, width = frame.shape[:2]
        cv2.line(frame, (width // 2, 0), (width // 2, height), (255, 255, 0), 1)

        box = self.track.box
        if box is not None and self.track.fresh:
            x1, y1, x2, y2 = (int(v) for v in box)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

        if self.track.alive and self.track.offset is not None:
            believed_x = int((self.track.offset + 1.0) * width / 2.0)
            believed_y = height // 2
            if self.track.y.known:
                believed_y = int((self.track.y.value + 1.0) * height / 2.0)
            colour = (0, 220, 0) if self.track.fresh else (0, 180, 220)
            cv2.line(frame, (believed_x, believed_y - 14),
                     (believed_x, believed_y + 14), colour, 2)
            cv2.line(frame, (believed_x - 14, believed_y),
                     (believed_x + 14, believed_y), colour, 2)
            # Where it will be in a third of a second, if it keeps going.
            lead = int(self.track.drift * width / 2.0 * 0.33)
            if abs(lead) > 3:
                cv2.arrowedLine(frame, (believed_x, believed_y),
                                (believed_x + lead, believed_y), colour, 2,
                                tipLength=0.3)
        return frame

    def status(self):
        return {
            "target": self.target,
            "orbit": self.orbit,
            "standoff_cm": self.standoff_cm,
            "state": self.state,
            "track": self.track.status(),
            "search": self.search.status(),
            "aim": self.aim.status(time.monotonic()) if self.aim else None,
        }
