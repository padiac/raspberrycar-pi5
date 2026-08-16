"""The serial link to the ESP32.

The ESP32 owns the motors and the ultrasonic because it can hold microsecond
timing, which Linux cannot -- a scheduling hiccup is enough to ruin an echo
measurement. This module is the Pi's end of that conversation.

Lines are text with an XOR checksum, deliberately readable: at 20Hz the traffic
is a few hundred bytes a second, so packing it into a binary struct would buy
nothing and cost the ability to watch the exchange in a terminal when something
misbehaves. Struct packing also invites alignment and endianness bugs between a
32-bit MCU and 64-bit ARM Linux.

Reading runs on its own thread so a silent or unplugged board can never block
the control loop.
"""

import glob
import os
import threading
import time

BAUD = 115200
#: How often to ping when nothing else is being sent.
#:
#: This exists only so the link's health can be judged while the robot is
#: sitting still. It deliberately does NOT keep the motors alive: the firmware
#: ignores heartbeats when deciding whether it is still being driven. That
#: separation matters because this ping runs on its own thread, so if the
#: control loop hung while this thread lived on, a heartbeat that counted as
#: "still driving" would let the car keep going under a robot that had stopped
#: thinking. Velocity commands come from the control loop at 20Hz, which is
#: well inside the firmware's 500ms window, so nothing else is needed.
HEARTBEAT_S = 0.4
#: Consider the board gone if no telemetry arrives for this long.
STALE_S = 1.0
#: Silence for this long means the link is broken even though the OS
#: still has a perfectly good file descriptor open. Generous next to the
#: 20Hz the board reports at, and next to the two seconds it takes to
#: reboot, so an ordinary reset is ridden out rather than reacted to.
SILENCE_S = 4.0


def checksum(body):
    value = 0
    for ch in body.encode():
        value ^= ch
    return value


def frame(body):
    return f"${body}*{checksum(body):02X}\n".encode()


def find_port():
    """First plausible USB serial device. The ESP32's CP210x shows up here."""
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        found = sorted(glob.glob(pattern))
        if found:
            return found[0]
    return None


class Telemetry:
    """Latest state reported by the firmware."""

    def __init__(self):
        self.range_cm = None
        self.wheels = (0.0, 0.0, 0.0, 0.0)
        self.stopped = True
        self.updated = 0.0
        self.messages = []

    @property
    def fresh(self):
        return self.updated and (time.monotonic() - self.updated) < STALE_S


class SerialLink:
    """Talks to the ESP32. Reconnects on its own if the cable is pulled."""

    def __init__(self, port=None, baud=BAUD):
        import serial  # pyserial

        self._serial_mod = serial
        self._opened_at = 0.0
        self.port_name = port or os.environ.get("CAR_SERIAL") or find_port()
        if not self.port_name:
            raise RuntimeError("no USB serial device found (is the ESP32 plugged in?)")

        self.baud = baud
        self.telemetry = Telemetry()
        self.lock = threading.Lock()
        self.running = True
        self.connected = False
        self.port = None

        self._last_sent = 0.0
        self._last_velocity = (0.0, 0.0, 0.0)

        self._open()
        threading.Thread(target=self._read_loop, daemon=True).start()
        threading.Thread(target=self._heartbeat_loop, daemon=True).start()

    # -- connection --------------------------------------------------------

    def _open(self):
        try:
            self.port = self._serial_mod.Serial(self.port_name, self.baud, timeout=0.2)
            # Opening the port resets the ESP32 via DTR/RTS; give it a moment
            # to boot before expecting it to answer.
            time.sleep(2.0)
            self.port.reset_input_buffer()
            self._opened_at = time.monotonic()
            self.connected = True
            print(f"[link] {self.port_name} open", flush=True)
        except Exception as exc:  # noqa: BLE001
            self.connected = False
            print(f"[link] cannot open {self.port_name}: {exc}", flush=True)

    def _read_loop(self):
        buffer = b""
        while self.running:
            if not self.connected:
                time.sleep(1.0)
                self._open()
                buffer = b""
                continue
            try:
                chunk = self.port.read(256)
            except Exception:  # noqa: BLE001 - cable pulled
                self.connected = False
                continue

            # A board that has stopped talking is a broken link, and it does not
            # look like one from here: the descriptor stays valid, reads return
            # empty, and nothing ever raises. Reconnecting only on an exception
            # meant that pulling the board's power while this was running left a
            # healthy-looking port waiting forever for a frame that was never
            # coming -- the car simply stopped responding, with no error
            # anywhere to say why.
            if self._silent_for_too_long():
                print(f"[link] {self.port_name} silent, reopening", flush=True)
                self._reopen()
                buffer = b""
                continue

            if not chunk:
                continue
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                self._handle(line.decode("ascii", "replace").strip())

    def _silent_for_too_long(self):
        """True if nothing has been parsed for a while.

        Measured from the last good frame, or from when the port was opened if
        there has never been one -- a board that was already dead when the port
        opened has to be caught too, not just one that dies later.
        """
        since = self.telemetry.updated or self._opened_at
        return since and (time.monotonic() - since) > SILENCE_S

    def _reopen(self):
        try:
            self.port.close()
        except Exception:  # noqa: BLE001
            pass
        self.connected = False

    def _handle(self, line):
        if not line.startswith("$") or "*" not in line:
            return
        body, _, given = line[1:].partition("*")
        try:
            if checksum(body) != int(given, 16):
                return                      # garbled, drop it
        except ValueError:
            return

        parts = body.split(",")
        if parts[0] == "T" and len(parts) >= 7:
            try:
                range_cm = float(parts[1])
                wheels = tuple(float(p) for p in parts[2:6])
                stopped = parts[6] == "1"
            except ValueError:
                return
            tel = self.telemetry
            tel.range_cm = None if range_cm < 0 else range_cm
            tel.wheels = wheels
            tel.stopped = stopped
            tel.updated = time.monotonic()
        elif parts[0] == "!":
            message = ",".join(parts[1:])
            print(f"[esp32] {message}", flush=True)
            self.telemetry.messages.append(message)
            del self.telemetry.messages[:-10]

    # -- sending -----------------------------------------------------------

    def _write(self, body):
        if not self.connected:
            return False
        try:
            with self.lock:
                self.port.write(frame(body))
            return True
        except Exception:  # noqa: BLE001
            self.connected = False
            return False

    def send_velocity(self, vx, vy, omega):
        self._last_velocity = (vx, vy, omega)
        self._last_sent = time.monotonic()
        return self._write(f"V,{vx:.3f},{vy:.3f},{omega:.3f}")

    def send_servo(self, angle):
        return self._write(f"S,{int(angle)}")

    def stop(self):
        self._last_velocity = (0.0, 0.0, 0.0)
        return self._write("E")

    def _heartbeat_loop(self):
        while self.running:
            time.sleep(HEARTBEAT_S)
            # Only needed when nothing else is talking; a moving robot is
            # already sending velocities faster than this.
            if time.monotonic() - self._last_sent > HEARTBEAT_S:
                self._write("H")
                self._last_sent = time.monotonic()

    def status(self):
        tel = self.telemetry
        return {
            "port": self.port_name,
            "connected": self.connected and tel.fresh,
            "range_cm": None if tel.range_cm is None else round(tel.range_cm, 1),
            "wheels": [round(w, 3) for w in tel.wheels],
            "messages": list(tel.messages[-3:]),
        }

    def close(self):
        self.running = False
        try:
            self.stop()
            self.port.close()
        except Exception:  # noqa: BLE001
            pass
