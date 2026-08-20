"""Checks the wire protocol without an ESP32 attached.

Uses a stand-in for the serial port, so the framing, the checksum and the
telemetry parsing can be exercised on their own. What this cannot test is
whether the firmware agrees -- that needs the board -- but it does catch the
things that are easy to get wrong on one side and only notice on the other.
"""

import sys
import time
import types

import sys
from pathlib import Path

# Find the robot from wherever this is run. Relying on the working directory
# instead -- which is what "sys.path.insert(0, '.')" amounts to -- works until
# somebody runs the file from anywhere else, and then fails with an ImportError
# about `hal` that says nothing about the actual problem.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from hal.link import SerialLink, checksum, frame

FAILURES = []


def check(name, got, want):
    ok = got == want
    print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    if not ok:
        print(f"        got {got!r}  want {want!r}")
        FAILURES.append(name)


class FakePort:
    """Stands in for serial.Serial: records writes, replays canned reads."""

    def __init__(self, *a, **kw):
        self.written = []
        self.to_read = b""
        self.closed = False

    def write(self, data):
        self.written.append(data)
        return len(data)

    def read(self, n=1):
        chunk, self.to_read = self.to_read[:n], self.to_read[n:]
        return chunk

    def reset_input_buffer(self):
        pass

    def close(self):
        self.closed = True

    def feed(self, text):
        self.to_read += text.encode()


def build_link():
    """A SerialLink wired to a FakePort instead of a real device."""
    port = FakePort()
    fake_serial = types.SimpleNamespace(Serial=lambda *a, **kw: port)

    link = SerialLink.__new__(SerialLink)      # bypass __init__ and its threads
    link._serial_mod = fake_serial
    link.port_name = "fake"
    link.baud = 115200
    from hal.link import Telemetry
    link.telemetry = Telemetry()
    import threading
    link.lock = threading.Lock()
    link.running = True
    link.connected = True
    link.port = port
    link._last_sent = 0.0
    link._last_velocity = (0.0, 0.0, 0.0)
    return link, port


print("checksum and framing")
check("checksum of 'V,1.000,0.000,0.000'",
      f"{checksum('V,1.000,0.000,0.000'):02X}",
      f"{checksum('V,1.000,0.000,0.000'):02X}")          # stable, self-consistent
check("frame wraps with $ and *", frame("H").startswith(b"$H*"), True)
check("frame ends with newline", frame("H").endswith(b"\n"), True)
# The checksum must cover the body only, not the $ or the *.
body = "T,12.3"
expected = 0
for ch in body.encode():
    expected ^= ch
check("checksum covers the body only", checksum(body), expected)

print("\nsending")
link, port = build_link()
link.send_velocity(0.5, -0.25, 1.0)
sent = port.written[-1].decode()
check("velocity line shape", sent.startswith("$V,0.500,-0.250,1.000*"), True)
check("velocity line ends with newline", sent.endswith("\n"), True)

link.send_servo(135)
check("servo line", port.written[-1].decode().startswith("$S,135*"), True)

link.stop()
check("stop line", port.written[-1].decode().startswith("$E*"), True)

print("\nreceiving telemetry")
link, port = build_link()
good = "T,23.4,1.000,0.333,0.333,1.000,0"
link._handle(f"${good}*{checksum(good):02X}")
tel = link.telemetry
check("range parsed", tel.range_cm, 23.4)
check("wheels parsed", tel.wheels, (1.0, 0.333, 0.333, 1.0))
check("stopped flag parsed", tel.stopped, False)
check("marked fresh", tel.fresh, True)

# -1 means the pulse timed out with nothing inside about 4m. That is a
# measurement, and it says the road is clear -- so it is carried through as it
# stands rather than folded into None.
#
# This check used to assert the opposite, and asserting it is what let the bug
# live: with -1 turned into None here, "nothing is in front of me" became
# indistinguishable from "the board has stopped talking", and the safety layer
# above -- which is right to refuse to drive forward blind -- refused to drive
# forward in an empty room. Reported as "it will not go forwards, everything
# else is fine".
noecho = "T,-1.0,0.000,0.000,0.000,0.000,1"
link._handle(f"${noecho}*{checksum(noecho):02X}")
check("no echo stays -1, not None", link.telemetry.range_cm, -1.0)
check("  and the link still counts as fresh", link.telemetry.fresh, True)

# None is reserved for the one thing it can honestly mean.
fresh_link, _ = build_link()
check("nothing received yet reads as None", fresh_link.telemetry.range_cm, None)

print("\ncorrupt input is ignored")
link, port = build_link()
before = link.telemetry.updated
link._handle("$T,99.9,0,0,0,0,0*00")          # wrong checksum
check("bad checksum dropped", link.telemetry.updated, before)
link._handle("garbage without a dollar")
check("unframed line dropped", link.telemetry.updated, before)
link._handle("$T,notanumber,0,0,0,0,0*" + f"{checksum('T,notanumber,0,0,0,0,0'):02X}")
check("unparseable numbers dropped", link.telemetry.updated, before)

print("\nstaleness")
link, port = build_link()
ok = "T,10.0,0,0,0,0,1"
link._handle(f"${ok}*{checksum(ok):02X}")
check("fresh right after an update", link.telemetry.fresh, True)
link.telemetry.updated = time.monotonic() - 5
check("stale after silence", link.telemetry.fresh, False)

print("\nround trip against the firmware's own format")
# Exactly the line bridge.ino builds, to catch a mismatch in field order.
firmware_line = "$T,23.4,1.000,0.333,0.333,1.000,0*"
firmware_body = firmware_line[1:-1]
link, port = build_link()
link._handle(firmware_line + f"{checksum(firmware_body):02X}")
check("firmware telemetry understood", link.telemetry.range_cm, 23.4)
check("  and its wheel order", link.telemetry.wheels, (1.0, 0.333, 0.333, 1.0))

print()
if FAILURES:
    print(f"{len(FAILURES)} FAILURE(S): {FAILURES}")
    sys.exit(1)
print("all link checks passed")
