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

import cv2

from control import PID, TargetTrack
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

#: Steering gains. Proportional to where the target is, derivative on how
#: fast it is crossing the frame.
#:
#: The derivative gain reads very differently now and the old number would
#: be wrong. It used to multiply a frame-to-frame difference already divided
#: by dt, so its effective size was kd/dt -- 0.3/0.05, an effective gain of
#: six against a proportional gain of 1.6. Here it multiplies a velocity
#: estimated from the whole history, in half-frame-widths per second, so it
#: means what it says: a target crossing the frame at one half-width a
#: second adds this much steering to meet it.
TURN_KP = 1.6
TURN_KD = 0.25

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
    turn_kp=Spec(
        TURN_KP, unit='', bounds=(0.0, 10.0),
        help='Steering response to where the target is.'),
    turn_kd=Spec(
        TURN_KD, unit='', bounds=(0.0, 5.0),
        help='Steering response to how fast the target is crossing the '
             'frame. Damps the swing on the way in and leads a moving '
             'target. Raise it if the car overshoots and comes back; lower '
             'it if the car twitches.'),
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
        self.standoff_cm = CHASE.standoff_cm

        #: Steering gains live on the PID but are read from the register at
        #: every tick, so turning the knob on the page takes effect without
        #: a restart -- which is the only way tuning a control loop by feel
        #: is bearable.
        self.turn_pid = PID(kp=CHASE.turn_kp, kd=CHASE.turn_kd, setpoint=0.0)
        #: Gain is per centimetre: 35cm too far asks for 0.7, which the clamp
        #: below brings to CHASE.max_forward.
        self.distance_pid = PID(kp=0.02, kd=0.005, setpoint=CHASE.standoff_cm)
        self.forward_pid = PID(kp=2.0, kd=0.2, setpoint=CHASE.target_fill)

        self.track = self._new_track()
        #: Where to look when the target has gone. Owned here rather than
        #: by the track, because losing something is a fact and deciding
        #: to go and look for it is intent, and this layer is the one that
        #: is allowed to have intentions.
        self.search = Reacquire()
        #: Heading at the last real sighting; see update().
        self._seen_at = 0.0
        self.state = "waiting"

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
                self.standoff_cm = max(25.0, min(200.0, float(kwargs["standoff_cm"])))
                self.distance_pid.setpoint = self.standoff_cm
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
        self.turn_pid.reset()
        self.distance_pid.reset()
        self.forward_pid.reset()

    @property
    def box(self):
        """The last box seen, for the overlay. Read off the track so there
        is one place that knows what is being followed."""
        return self.track.box

    def start(self, robot):
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

        # Live gains, so the page can tune the loop while it runs.
        self.turn_pid.kp = CHASE.turn_kp
        self.turn_pid.kd = CHASE.turn_kd

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
            if self.track.seen > 0:
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
                self.state = (f"no {self.target} in view"
                              if not self.search.searching
                              else f"lost the {self.target}")
                return Velocity.STOP
            self.state = self.search.describe(self.target)
            # Camera only. Driving towards something nobody can currently
            # see is a different and much worse idea than looking for it.
            return Velocity(omega=turning)

        self.search.found()
        offset = self.track.offset
        drift = self.track.drift

        # -- turn: drive the horizontal offset to zero ----------------------
        omega = 0.0
        if abs(offset) > CHASE.centre_deadband:
            # PID error is (setpoint - measured) = -offset, so a target to the
            # right gives a negative output; flip it back to "turn right".
            # The rate comes from the track's own state rather than from
            # subtracting two sightings -- see PID.update().
            omega = -self.turn_pid.update(offset, world.dt, rate=drift)
            omega = max(-CHASE.max_turn, min(CHASE.max_turn, omega))

        # -- drive: hold a real distance, and fall back to apparent size ----
        ahead = abs(offset) < CHASE.ahead_deadband
        measured = (
            world.range_cm
            if (world.range_cm is not None and world.range_cm > 0
                and world.range_forward and ahead)
            else None
        )

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
        }
