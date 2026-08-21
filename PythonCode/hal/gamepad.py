"""A game controller plugged into the robot rather than into the browser.

The page has always polled the browser's Gamepad API, and on this car that
cannot work: the API is restricted to secure contexts, and the robot is served
over plain HTTP on a LAN address that can never hold a certificate. So that
route is closed rather than broken.

Reading the pad here is the better answer anyway. It takes the phone out of
the loop entirely -- no browser, no wifi round trip, no page needing to stay in
the foreground -- and a controller talking straight to the machine it drives is
one fewer thing between the stick and the wheels.

Nothing new is invented above this. A stick is another source of manual
driving, exactly like the on-screen pad, so it goes in through the same
`robot.manual()` door: same behaviour, same speed governor, same safety layer.
What the buttons do is likewise an operation that already existed, named in
`actions`, so this file knows how to read a controller and nothing about what
a robot is.

    left stick        drive: up/down is vx, left/right strafes (mecanum only)
    right stick X     turn
    R2 / L2           faster / slower, underneath the speed the slider allows
    D-pad left/right  swing the ultrasonic head 45 degrees
    D-pad down        head back to centre
    Circle            stop
    Triangle          back to manual driving, whatever it was doing
    Options           next behaviour
    Cross             start or stop recording a path
    rumble            how close the obstacle ahead is
"""

import threading
import time

import evdev
from evdev import ecodes

from .drivetrain import Velocity

#: Sticks rest a little off centre and jitter there. Below this, no movement.
DEADZONE = 0.12
#: Resend rate while a stick is held. The manual behaviour times out if it
#: stops hearing commands, which is what makes a dropped controller stop the
#: car rather than leave it running.
SEND_HZ = 20.0
#: How often to look for a controller that is not there yet.
RESCAN_S = 2.0
#: Substring matches, lowercased. Pads name themselves variously -- a DualSense
#: comes up as "Wireless Controller" under some kernels and by name under
#: others -- so this is deliberately loose, and backed up by checking the
#: device actually has the axes and buttons we need.
NAMES = ("dualsense", "dualshock", "wireless controller", "gamepad",
         "controller", "xbox", "x-box", "joystick")

#: Fully on the brake trigger. With neither trigger touched the sticks get
#: their full range, because anything else makes the pad quietly weaker
#: than the buttons on the phone -- which it was: a 0.6 here multiplied by
#: a speed limit of 0.15 came to 0.09, under what the motors need to turn
#: at all, so the pad connected, reported fine, and did nothing.
CRAWL = 0.2
#: Each nudge of the D-pad, in degrees.
PAN_STEP = 45.0

#: The same direction pressed again inside this is the switch bouncing, not a
#: person. Nobody presses a d-pad twice in a third of a second, and the "has
#: the value changed" test does not catch it: a bounce goes -1, 0, -1, and
#: every one of those is a change.
REPEAT_GUARD_S = 0.3

#: Rumble is re-sent rather than held, because an effect with a duration
#: outlives the reason for it if the car stops. This is how often.
RUMBLE_PERIOD_S = 0.4
#: Ignore changes smaller than this, so a distance wobbling by a centimetre
#: does not re-upload an effect twenty times a second.
RUMBLE_EPSILON = 0.12


#: What the buttons do, and what to call them.
#:
#: A table rather than a chain of elif, because it is also the answer to "what
#: do the buttons do" -- the page reads this and renders it, so the help cannot
#: drift from the behaviour the way a written list does. Adding a binding here
#: adds it to both.
#:
#: One code per button, and it has to be the compass name.
#:
#: The letter names in linux/input-event-codes.h are not the aliases they look
#: like. BTN_X is BTN_NORTH and BTN_Y is BTN_WEST -- so listing NORTH and Y
#: together, as a defensive pair covering one button under two drivers, claims
#: two different buttons instead. Written that way, Triangle and Square both
#: matched the first row that mentioned either, first-match-wins gave both to
#: "back to manual", and Square's replay never once fired. It looked like the
#: recording had never worked; recording was fine and nothing was asking to
#: play it.
BUTTONS = [
    (("BTN_EAST",), "stop", "Circle", "Stop everything"),
    (("BTN_NORTH",), "manual", "Triangle", "Back to driving by hand"),
    (("BTN_WEST",), "play_path", "Square", "Replay the recorded path"),
    (("BTN_SOUTH",), "toggle_record", "Cross", "Start or stop recording a path"),
    (("BTN_START",), "cycle_behavior", "Options", "Next behaviour"),
    (("BTN_SELECT",), "cycle_behavior", "Create", "Next behaviour"),
    (("BTN_TL",), "slower", "L1", "Speed limit down"),
    (("BTN_TR",), "faster", "R1", "Speed limit up"),
]

#: The d-pad, which reports as two axes and behaves as four buttons. Same
#: shape as BUTTONS so the page can render them together, with the value the
#: axis takes when that direction is pressed, and an argument for the action
#: where it needs one.
HATS = [
    ("ABS_HAT0X", -1, "pan_by", -PAN_STEP, "D-pad left", "Swing the head 45 degrees left"),
    ("ABS_HAT0X", 1, "pan_by", PAN_STEP, "D-pad right", "Swing the head 45 degrees right"),
    ("ABS_HAT0Y", 1, "pan_centre", None, "D-pad down", "Head back to centre"),
    ("ABS_HAT0Y", -1, "toggle_clip", None, "D-pad up", "Start or stop recording video"),
]

#: What is left: continuous, so not a lookup, but still in need of explaining
#: somewhere that cannot go stale.
AXES = [
    ("Left stick", "Drive: forward and back, and sideways on a mecanum chassis"),
    ("Right stick, left/right", "Turn"),
    ("L2", "Crawl -- squeezes the speed down to a fifth"),
    ("R2", "Lift the speed limit, for as long as it is held"),
    ("Rumble", "How close the obstacle ahead is"),
]


def describe():
    """The whole mapping, for anyone who has to explain it to a person."""
    return {
        "buttons": ([{"button": name, "does": what}
                     for _, _, name, what in BUTTONS]
                    + [{"button": name, "does": what}
                       for _, _, _, _, name, what in HATS]),
        "axes": [{"button": name, "does": what} for name, what in AXES],
    }


def _codes(names):
    return {getattr(ecodes, n) for n in names if hasattr(ecodes, n)}


def _check_bindings():
    """No button may mean two things.

    Cheap, and it is the check that would have caught Square and Triangle
    sharing a code -- a fault with no symptom except a button that quietly did
    the wrong job.
    """
    seen = {}
    for names, action, label, _ in BUTTONS:
        for code in _codes(names):
            if code in seen:
                raise ValueError(
                    f"{label} ({action}) and {seen[code]} both claim code {code}")
            seen[code] = label
    return seen


_check_bindings()


def _looks_like_a_pad(device):
    caps = device.capabilities()
    axes = {code for code, _ in caps.get(ecodes.EV_ABS, [])}
    if not {ecodes.ABS_X, ecodes.ABS_Y} <= axes:
        return False
    # Touchpads and accelerometers report ABS_X/Y too. A pad has face buttons.
    keys = set(caps.get(ecodes.EV_KEY, []))
    return bool(keys & {ecodes.BTN_SOUTH, ecodes.BTN_EAST, ecodes.BTN_A, ecodes.BTN_B})


class Gamepad:
    """Reads a controller, drives through callbacks, and buzzes.

    `on_velocity(velocity)` is called at SEND_HZ while anything is off centre,
    once more with a stop when everything returns to centre, and never
    otherwise -- so a resting controller does not fight the web page for
    control of the car.

    `actions` names what the buttons do. Anything absent is simply not bound,
    which is what lets this file stay ignorant of what it is driving.
    """

    def __init__(self, on_velocity, actions=None, holonomic=True):
        self.on_velocity = on_velocity
        self.actions = dict(actions or {})
        #: bool or a callable, since the chassis can be swapped at runtime and
        #: a value captured at construction would go stale.
        self._holonomic = holonomic
        self.device = None
        self.name = None
        self.running = True

        self._axes = {}
        self._scale = {}
        self._hat = {}
        self._hat_at = {}
        self._alert = 0.0
        self._rumble_sent = -1.0
        self._rumble_at = 0.0
        self._effect_id = None
        self.rumble_ok = False

        threading.Thread(target=self._loop, daemon=True).start()

    # -- finding one -------------------------------------------------------

    def _find(self):
        for path in evdev.list_devices():
            try:
                device = evdev.InputDevice(path)
            except OSError:
                continue
            if any(n in device.name.lower() for n in NAMES) and _looks_like_a_pad(device):
                return device
            device.close()
        return None

    def _learn(self, device):
        """Record each axis range, rather than assuming one.

        The same pad reports 0..255 through one driver and -32768..32767
        through another, so the numbers only mean anything relative to the
        range the kernel declares for them.
        """
        self._scale = {}
        for code, info in device.capabilities().get(ecodes.EV_ABS, []):
            self._scale[code] = (info.min, info.max)
        self._axes = {}
        self._hat = {}
        self._hat_at = {}
        self.rumble_ok = ecodes.FF_RUMBLE in device.capabilities().get(ecodes.EV_FF, [])
        self._effect_id = None
        self._rumble_sent = -1.0

    # -- reading it --------------------------------------------------------

    def _stick(self, code):
        """A centred axis as -1..1, dead in the middle, squared for feel."""
        raw = self._axes.get(code)
        if raw is None or code not in self._scale:
            return 0.0
        low, high = self._scale[code]
        span = (high - low) / 2.0
        if span <= 0:
            return 0.0
        value = (raw - (low + span)) / span
        if abs(value) < DEADZONE:
            return 0.0
        # Square the response, keeping the sign. Full deflection is still full
        # speed, but the first half of the travel now spans a quarter of the
        # range, which is where the driving actually happens.
        return value * abs(value)

    def _trigger(self, code):
        """A trigger as 0..1.

        It rests at one end rather than in the middle, so the stick maths above
        would read an untouched trigger as permanently held hard over.
        """
        raw = self._axes.get(code)
        if raw is None or code not in self._scale:
            return 0.0
        low, high = self._scale[code]
        if high <= low:
            return 0.0
        return max(0.0, min(1.0, (raw - low) / (high - low)))

    @property
    def holonomic(self):
        return self._holonomic() if callable(self._holonomic) else bool(self._holonomic)

    def _velocity(self):
        # Left stick drives, right stick turns. evdev counts Y downwards.
        vx = -self._stick(ecodes.ABS_Y)
        vy = self._stick(ecodes.ABS_X) if self.holonomic else 0.0
        omega = self._stick(ecodes.ABS_RX)

        # L2 attenuates, and only attenuates: sticks alone give exactly what
        # the on-screen pad gives, so picking up the controller never makes
        # the car slower than putting it down.
        brake = self._trigger(ecodes.ABS_Z)
        throttle = 1.0 - brake * (1.0 - CRAWL)
        return Velocity(vx=vx, vy=vy, omega=omega).scaled(throttle)

    # -- buttons -----------------------------------------------------------

    def _do(self, name, *args):
        """Run an action here and now. For quick ones only -- see _do_async."""
        action = self.actions.get(name)
        if action is None:
            return
        try:
            action(*args)
        except Exception as exc:  # noqa: BLE001 - a bound action must not kill the reader
            print(f"[gamepad] {name}: {exc}", flush=True)

    def _do_async(self, name, *args):
        """Run an action without making the controller wait for it.

        Some of them are slow. Stopping a recording waits for ffmpeg to write
        the index, which can take seconds, and done on this thread it stops the
        reader dead: events pile up in the kernel buffer, the release of the
        button that started it arrives late along with whatever was pressed
        next, and they are all processed in a rush afterwards. The car stops
        answering the sticks for the duration, too.

        Buttons are the only things routed through here. The triggers are read
        every tick and must stay in order, so they still go through _do.
        """
        if self.actions.get(name) is None:
            return
        threading.Thread(
            target=self._do, args=(name,) + args, daemon=True).start()

    def _button(self, code):
        """Dispatch a press. One code, one action, checked at import time."""
        for names, action, _, _ in BUTTONS:
            if code in _codes(names):
                self._do_async(action)
                return

    def _hat_moved(self, code, value):
        """The D-pad reports as an axis, but it is a set of buttons.

        Firing on the transition rather than on the value is the difference
        between one nudge per press and a nudge every tick it is held down,
        which on a servo means it slams straight to the end stop.
        """
        if self._hat.get(code) == value:
            return
        self._hat[code] = value
        if value == 0:
            return

        # A bounce is -1, 0, -1 in a few milliseconds, and every one of those is
        # a change, so the test above waves it straight through. Two toggles of
        # a video recording from one press of a button is exactly what that
        # looks like from the outside.
        now = time.monotonic()
        if now - self._hat_at.get((code, value), 0.0) < REPEAT_GUARD_S:
            return
        self._hat_at[(code, value)] = now

        for axis, when, action, argument, _, _ in HATS:
            if code == getattr(ecodes, axis, None) and value == when:
                if argument is None:
                    self._do_async(action)
                else:
                    self._do_async(action, argument)
                return

    # -- rumble ------------------------------------------------------------

    def alert(self, level):
        """How hard to buzz, 0 to 1. Called from the control loop.

        The car is usually further away than you can judge from, and the useful
        thing about a distance reading is knowing about it before you can see
        that it matters.
        """
        self._alert = max(0.0, min(1.0, float(level)))

    def _pump_rumble(self):
        if not self.rumble_ok or self.device is None:
            return
        now = time.monotonic()
        changed = abs(self._alert - self._rumble_sent) > RUMBLE_EPSILON
        if not changed and (self._alert <= 0.0 or now < self._rumble_at):
            return
        try:
            if self._effect_id is not None and (changed or self._alert <= 0.0):
                self.device.erase_effect(self._effect_id)
                self._effect_id = None
            if self._alert <= 0.0:
                self._rumble_sent = 0.0
                return
            if self._effect_id is None:
                from evdev import ff

                strong = int(0xFFFF * self._alert)
                weak = int(0xFFFF * self._alert * 0.6)
                effect = ff.Effect(
                    ecodes.FF_RUMBLE, -1, 0,
                    ff.Trigger(0, 0),
                    # A little longer than the resend period, so consecutive
                    # pulses overlap into one continuous buzz rather than
                    # stuttering at the seams.
                    ff.Replay(int(RUMBLE_PERIOD_S * 1000) + 120, 0),
                    ff.EffectType(ff_rumble_effect=ff.Rumble(strong, weak)),
                )
                self._effect_id = self.device.upload_effect(effect)
                self._rumble_sent = self._alert
            self.device.write(ecodes.EV_FF, self._effect_id, 1)
            self._rumble_at = now + RUMBLE_PERIOD_S
        except (OSError, ValueError, AttributeError) as exc:
            # Force feedback is a nicety. Losing it must not cost the driving.
            print(f"[gamepad] rumble unavailable: {exc}", flush=True)
            self.rumble_ok = False
            self._effect_id = None

    # -- the loop ----------------------------------------------------------

    def _loop(self):
        period = 1.0 / SEND_HZ
        moving = False
        next_scan = 0.0

        while self.running:
            if self.device is None:
                now = time.monotonic()
                if now < next_scan:
                    time.sleep(0.2)
                    continue
                next_scan = now + RESCAN_S
                device = self._find()
                if device is None:
                    continue
                self.device = device
                self.name = device.name
                self._learn(device)
                extra = " with rumble" if self.rumble_ok else ""
                print(f"[gamepad] {device.name} on {device.path}{extra}", flush=True)

            try:
                # A timeout rather than a blocking read, so the resend happens
                # on time whether or not anything moved. A held stick generates
                # no events at all, and the manual behaviour would time out
                # underneath it.
                for event in self._drain(period):
                    if event.type == ecodes.EV_ABS:
                        if event.code in (ecodes.ABS_HAT0X, ecodes.ABS_HAT0Y):
                            self._hat_moved(event.code, event.value)
                        else:
                            self._axes[event.code] = event.value
                    elif event.type == ecodes.EV_KEY and event.value == 1:
                        self._button(event.code)
            except OSError as exc:
                print(f"[gamepad] {self.name} disconnected: {exc}", flush=True)
                self._close_device()
                # A controller that vanishes mid-corner must not leave the car
                # driving. Say stop once on the way out, rather than waiting for
                # the timeout to work it out.
                if moving:
                    self._do("stop")
                self._do("boost", 0.0)
                moving = False
                continue

            # R2 lifts the limit rather than scaling the velocity. Scaling
            # both here and at the governor is what made the pad weaker the
            # harder it was configured; this way the slider is the resting
            # speed and the trigger reaches past it, which is what a trigger
            # is for. The person holding the pad can see the car.
            self._do("boost", self._trigger(ecodes.ABS_RZ))

            velocity = self._velocity()
            if velocity.moving:
                moving = True
                self.on_velocity(velocity)
            elif moving:
                moving = False
                self.on_velocity(Velocity.STOP)

            self._pump_rumble()

    def _drain(self, timeout):
        """Every event waiting, having waited up to `timeout` for the first."""
        import select

        events = []
        ready, _, _ = select.select([self.device.fd], [], [], timeout)
        if ready:
            while True:
                event = self.device.read_one()
                if event is None:
                    break
                events.append(event)
        return events

    def _close_device(self):
        try:
            if self.device:
                self.device.close()
        except OSError:
            pass
        self.device = None
        self._effect_id = None
        self.rumble_ok = False

    def close(self):
        self.running = False
        self._close_device()

    def status(self):
        return {
            "connected": self.device is not None,
            "name": self.name if self.device is not None else None,
            "rumble": self.rumble_ok,
        }
