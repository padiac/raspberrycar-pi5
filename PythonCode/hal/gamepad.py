"""A game controller plugged into the robot rather than into the browser.

The page has always polled the browser's Gamepad API, and on this car that
cannot work: the API is restricted to secure contexts, and the robot is served
over plain HTTP on a LAN address. There is no certificate to be had for
192.168.0.22, so that route is closed rather than broken.

Reading the pad here is the better answer anyway. It takes the phone out of
the loop entirely -- no browser, no wifi round trip, no page needing to stay
in the foreground -- and a controller talking straight to the machine it
drives is one fewer thing between the stick and the wheels.

Nothing new is invented above this. A stick is another source of manual
driving, exactly like the on-screen pad, so it goes in through the same
`robot.manual()` door: same behaviour, same speed governor, same safety layer.
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
#: others -- so this is deliberately loose and backed up by checking the device
#: actually has the axes we need.
NAMES = ("dualsense", "dualshock", "wireless controller", "gamepad",
         "controller", "xbox", "x-box", "joystick")


def _looks_like_a_pad(device):
    caps = device.capabilities()
    axes = {code for code, _ in caps.get(ecodes.EV_ABS, [])}
    if not {ecodes.ABS_X, ecodes.ABS_Y} <= axes:
        return False
    # Touchpads and accelerometers also report ABS_X/Y. A pad has buttons.
    keys = set(caps.get(ecodes.EV_KEY, []))
    return bool(keys & {ecodes.BTN_SOUTH, ecodes.BTN_EAST, ecodes.BTN_A, ecodes.BTN_B})


class Gamepad:
    """Reads a controller and calls back with a Velocity.

    `on_velocity(velocity)` is called at SEND_HZ while anything is off centre,
    once more with a stop when everything returns to centre, and never
    otherwise -- so a resting controller does not fight the web page for
    control of the car.
    """

    def __init__(self, on_velocity, on_stop=None, holonomic=True):
        self.on_velocity = on_velocity
        self.on_stop = on_stop
        self.holonomic = holonomic
        self.device = None
        self.name = None
        self.error = None
        self.running = True
        self._axes = {}
        self._scale = {}
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

    def _learn_axes(self, device):
        """Record each axis's range, rather than assuming one.

        The same pad reports 0..255 through one driver and -32768..32767
        through another, so the numbers only mean something relative to the
        range the kernel declares for them.
        """
        self._scale = {}
        for code, info in device.capabilities().get(ecodes.EV_ABS, []):
            span = (info.max - info.min) / 2.0
            if span > 0:
                self._scale[code] = (info.min + span, span)
        self._axes = {}

    def _value(self, code):
        raw = self._axes.get(code)
        if raw is None or code not in self._scale:
            return 0.0
        centre, span = self._scale[code]
        value = (raw - centre) / span
        if abs(value) < DEADZONE:
            return 0.0
        # Square the response, keeping the sign. Full deflection still means
        # full speed, but the first half of the stick's travel is spread over
        # a quarter of it, which is where the useful driving happens.
        return value * abs(value)

    # -- reading it --------------------------------------------------------

    def _velocity(self):
        # Left stick drives, right stick turns. evdev counts Y downwards.
        vx = -self._value(ecodes.ABS_Y)
        vy = self._value(ecodes.ABS_X) if self.holonomic else 0.0
        omega = self._value(ecodes.ABS_RX)
        return Velocity(vx=vx, vy=vy, omega=omega)

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
                self.error = None
                self._learn_axes(device)
                print(f"[gamepad] {device.name} on {device.path}", flush=True)

            try:
                # A timeout rather than a blocking read, so the resend happens
                # on time whether or not the stick has moved. A held stick
                # generates no events at all, and the manual behaviour would
                # time out underneath it.
                for event in self._drain(period):
                    if event.type == ecodes.EV_ABS:
                        self._axes[event.code] = event.value
                    elif event.type == ecodes.EV_KEY and event.value == 1:
                        if event.code in (ecodes.BTN_EAST, ecodes.BTN_B):
                            if self.on_stop:
                                self.on_stop()
                            moving = False
            except OSError as exc:
                print(f"[gamepad] {self.name} disconnected: {exc}", flush=True)
                self._close_device()
                # A controller that vanishes mid-corner must not leave the car
                # driving. Say stop once, on the way out.
                if moving and self.on_stop:
                    self.on_stop()
                moving = False
                continue

            velocity = self._velocity()
            if velocity.moving:
                moving = True
                self.on_velocity(velocity)
            elif moving:
                moving = False
                self.on_velocity(Velocity.STOP)

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

    def close(self):
        self.running = False
        self._close_device()

    def status(self):
        return {
            "connected": self.device is not None,
            "name": self.name if self.device is not None else None,
        }
