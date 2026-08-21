"""What the Pi thinks of its own power supply.

No wiring. The Pi 5 has a PMIC that measures its input rail and a set of
sticky flags recording every time that rail has misbehaved, and `vcgencmd`
hands both over for the asking. A battery does not usually fail cleanly -- the
rail sags first, for a while, and only then does everything stop -- so this is
enough warning to matter, and it costs a subprocess every couple of seconds.

It does not see the 7.4V pack driving the motors. That one is on the other
side of a buck converter and needs a divider and two resistors before anything
here can know about it. This covers the supply the Pi itself runs on, which is
the one that takes the whole robot down with it.

The throttle flags are the interesting half, because the low sixteen bits say
"right now" and the high sixteen say "at some point since boot". A brownout
that lasted 200ms leaves nothing in the live bits and a permanent mark in the
sticky ones, which is precisely the evidence you want and cannot otherwise
get: rebooting clears them, so a fault that causes a reboot erases its own
record unless something was watching.
"""

import re
import subprocess
import threading
import time

#: Below this the Pi is unhappy but still running. Its own undervoltage trip
#: is at 4.63V, so this leaves room to say something first.
SAG_VOLTS = 4.90
#: How often to ask. vcgencmd is a subprocess, and the answer changes slowly.
PERIOD_S = 2.0

#: Meaning of the bits in get_throttled. The low half is now, the high half is
#: ever-since-boot.
FLAGS = {
    0: "undervoltage",
    1: "arm frequency capped",
    2: "throttled",
    3: "soft temperature limit",
}


def _vcgencmd(*args):
    return subprocess.run(
        ["vcgencmd", *args], capture_output=True, text=True, timeout=3,
    ).stdout.strip()


class Supply:
    """Polls the PMIC on its own thread and remembers the worst of it."""

    def __init__(self):
        self.volts = None
        self.throttled = 0
        self.temperature = None
        self.available = False
        #: Sticky in our own right: the flags survive until the process
        #: restarts, so a sag during a drive is still visible afterwards.
        self.seen = set()
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.running:
            try:
                self._sample()
                self.available = True
            except Exception as exc:  # noqa: BLE001 - never worth stopping for
                if self.available:
                    print(f"[supply] {exc}", flush=True)
                self.available = False
            time.sleep(PERIOD_S)

    def _sample(self):
        text = _vcgencmd("pmic_read_adc", "EXT5V_V")
        match = re.search(r"=([0-9.]+)V", text)
        self.volts = float(match.group(1)) if match else None

        text = _vcgencmd("get_throttled")
        match = re.search(r"=0x([0-9a-fA-F]+)", text)
        self.throttled = int(match.group(1), 16) if match else 0
        for bit, name in FLAGS.items():
            if self.throttled & (1 << bit) or self.throttled & (1 << (bit + 16)):
                self.seen.add(name)

        text = _vcgencmd("measure_temp")
        match = re.search(r"=([0-9.]+)", text)
        self.temperature = float(match.group(1)) if match else None

    @property
    def sagging(self):
        return self.volts is not None and self.volts < SAG_VOLTS

    @property
    def unhappy(self):
        """Something is wrong now, as opposed to having been wrong once."""
        return self.sagging or bool(self.throttled & 0xF)

    def close(self):
        self.running = False

    def status(self):
        return {
            "available": self.available,
            "volts": None if self.volts is None else round(self.volts, 2),
            "temperature": self.temperature,
            "sagging": self.sagging,
            "unhappy": self.unhappy,
            "now": sorted(n for b, n in FLAGS.items() if self.throttled & (1 << b)),
            "ever": sorted(self.seen),
        }
