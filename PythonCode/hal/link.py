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
#: A reopen that does not stick within this long is taken to mean the board
#: itself is wedged rather than the USB plumbing, and the next attempt is
#: allowed to reset it. Reopening used to always reset, which fixed a
#: bootloader wedge at the cost of restarting the executor -- and so
#: re-centring the camera servo -- every time the USB bridge hiccuped. A
#: head that twitches on its own once an hour is that, not a ghost.
ESCALATE_S = 20.0

#: Most bytes this will hold waiting for a newline before deciding they are
#: not a line at all.
#:
#: Found the hard way: a log line read "0 bytes waiting, 42374 partial". Forty
#: kilobytes had accumulated without a single newline in it -- the ESP32 boot
#: ROM talks at 74880 baud, and read at 115200 its chatter is bytes that
#: happen never to be 0x0A. Nothing trimmed that buffer, so it grew for as
#: long as the process ran, and every byte of real telemetry arriving after
#: it was appended to a string the parser would never find a line in. A link
#: that is permanently deaf and a process that leaks memory, from the same
#: missing bound.
#:
#: A frame is about sixty bytes, so a kilobyte is already more slack than any
#: real line needs.
MAX_PARTIAL = 1024
#: What to keep when discarding: enough to not slice a real frame in half if
#: one happens to be arriving at that moment.
KEEP_PARTIAL = 128
#: A banner arriving within this long of a reset we asked for is that reset
#: answering, not the board restarting on its own.
EXPECTED_BANNER_S = 6.0


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
        #: None until a frame arrives, then whatever the board said --
        #: including a negative, which means nothing is within range.
        self.range_cm = None
        #: None until a frame carries them, so "no infrared fitted" and
        #: "infrared sees nothing" stay different answers.
        self.proximity = None
        #: Where the board thinks it is pointing, in degrees, relative to
        #: wherever it was last zeroed. Integrated from a gyroscope, so it
        #: drifts: good for a manoeuvre, useless as a compass.
        self.heading = None
        #: A jolt the accelerometer noticed. Held by the firmware for long
        #: enough to survive being polled at 20Hz.
        self.bumped = False
        self.wheels = (0.0, 0.0, 0.0, 0.0)
        self.stopped = True
        self.updated = 0.0
        self.messages = []

    @property
    def fresh(self):
        return self.updated and (time.monotonic() - self.updated) < STALE_S


class SerialLink:
    """Talks to the ESP32. Reconnects on its own if the cable is pulled."""

    #: Default here as well as in __init__, because the test harness
    #: builds one without running __init__ to avoid starting threads
    #: against a device that is not there.
    heading_reversed = False

    def __init__(self, port=None, baud=BAUD, heading_reversed=False):
        import serial  # pyserial

        #: Whether this car's gyro counts the opposite way to its wheels;
        #: see calibration.py. Applied here, at the one place a heading
        #: enters the program, rather than by each of the four things that
        #: read one -- three of which only ever take its absolute value
        #: and so would never have noticed it was wrong.
        self.heading_reversed = bool(heading_reversed)

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
        #: Whether the next open may reset the board. The first one may: at
        #: startup the board could genuinely be sitting in its bootloader.
        self._next_hard = True
        self._opened_once = False
        self._last_reopen = 0.0
        #: Called with hard=True/False after any reopen, and after the board
        #: has restarted by itself. Set by the owner.
        self.on_reconnect = None
        #: Times the board has announced itself without being asked to. The
        #: whole reason this counter exists: the servo twitching once an hour
        #: with nothing driving it was the firmware booting and re-centring
        #: the head, and from the outside that is indistinguishable from a
        #: ghost. A number on the dashboard makes it a fact.
        self.board_reboots = 0
        #: Bytes thrown away as not-a-line.
        self.garbage = 0
        self._last_hard = 0.0
        self._last_garbage_note = 0.0

        self._open()
        threading.Thread(target=self._read_loop, daemon=True).start()
        threading.Thread(target=self._heartbeat_loop, daemon=True).start()

    # -- connection --------------------------------------------------------

    def _open(self, hard=None):
        if hard is None:
            hard = self._next_hard
        self._next_hard = True
        try:
            self.port = self._serial_mod.Serial(self.port_name, self.baud, timeout=0.2)

            # Put the board into its firmware, not its bootloader.
            #
            # On these USB-serial boards RTS is wired to EN and DTR to GPIO0,
            # and whatever levels the driver leaves them at when the port is
            # opened decide which of the two the chip comes up in. Get it wrong
            # and the port opens perfectly, the chip sits in the ROM waiting to
            # be flashed, and not one byte is ever sent -- which looks exactly
            # like a dead board or a bad cable, and has cost this project an
            # evening at least once.
            #
            # So say it explicitly: GPIO0 high, meaning run rather than
            # download, and then pulse EN to reset into that choice.
            #
            # Only when asked, though. A reset is the heavy answer and it has
            # side effects: the firmware comes up centring the servo and with
            # no velocity, so a board that was answering perfectly well a
            # moment ago gets its head yanked back to the middle. A plain
            # reopen is tried first and the reset kept for when that fails.
            if hard:
                try:
                    self.port.dtr = False
                    self.port.rts = True
                    time.sleep(0.12)
                    self.port.rts = False
                    time.sleep(0.35)
                    self.port.reset_input_buffer()
                except (OSError, IOError):
                    pass
                # Resetting the ESP32 costs it a boot; give it a moment before
                # expecting it to answer.
                self._last_hard = time.monotonic()
                time.sleep(2.0)
            else:
                # Do not touch DTR or RTS at all: on these boards they are the
                # reset and boot-mode lines, and the point here is to leave the
                # running firmware alone.
                time.sleep(0.3)
            self.port.reset_input_buffer()
            self._opened_at = time.monotonic()
            self.connected = True
            print("[link] %s open%s" % (self.port_name, " (board reset)" if hard else ""), flush=True)
            if self._opened_once and self.on_reconnect is not None:
                # Whatever state the far end had is now in doubt, and only the
                # layers above know what it is meant to be.
                try:
                    self.on_reconnect(hard)
                except Exception as exc:  # noqa: BLE001
                    print(f"[link] reconnect hook failed: {exc}", flush=True)
            self._opened_once = True
        except Exception as exc:  # noqa: BLE001
            self.connected = False
            print(f"[link] cannot open {self.port_name}: {exc}", flush=True)

    def _read_loop(self):
        buffer = b""
        # Instrumentation for a fault that has three possible homes and no
        # way to tell them apart from the outside: silence can mean the board
        # stopped sending, the USB bridge stopped delivering, or this thread
        # stopped asking. The first two leave this loop spinning on empty
        # reads; the third leaves a hole in its own timeline. So keep the
        # worst gap between iterations, and report it alongside the silence.
        spins = 0
        worst_gap = 0.0
        last_iter = time.monotonic()
        while self.running:
            now = time.monotonic()
            gap = now - last_iter
            last_iter = now
            if gap > worst_gap:
                worst_gap = gap
            spins += 1
            if not self.connected:
                time.sleep(1.0)
                self._open()
                buffer = b""
                spins, worst_gap, last_iter = 0, 0.0, time.monotonic()
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
                try:
                    waiting = self.port.in_waiting
                except Exception:  # noqa: BLE001
                    waiting = -1
                if self.telemetry.updated:
                    age = "%.1fs" % (time.monotonic() - self.telemetry.updated)
                else:
                    # Subtracting from zero gave the age of the process, which
                    # printed as a plausible and entirely fictional number.
                    age = "ever"
                print(
                    "[link] %s silent for %s, reopening"
                    " (reader spun %d times, worst pause %.2fs,"
                    " %d bytes waiting, %d partial)"
                    % (self.port_name, age, spins, worst_gap, waiting, len(buffer)),
                    flush=True,
                )
                self._reopen()
                buffer = b""
                spins, worst_gap, last_iter = 0, 0.0, time.monotonic()
                continue

            if not chunk:
                continue
            buffer += chunk
            if len(buffer) > MAX_PARTIAL:
                # Not a line. Keep the tail in case a real frame is arriving
                # inside it and drop the rest, which is the only choice that
                # both bounds the memory and lets the link recover.
                self.garbage += len(buffer) - KEEP_PARTIAL
                buffer = buffer[-KEEP_PARTIAL:]
                now = time.monotonic()
                if now - self._last_garbage_note > 5.0:
                    self._last_garbage_note = now
                    print("[link] discarded %d bytes of non-line noise"
                          % self.garbage, flush=True)
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                self._handle(line.decode("ascii", "replace").strip())

    def _silent_for_too_long(self):
        """True if nothing has been parsed for a while.

        Measured from the last good frame or from when the port was opened,
        whichever is later. Both halves matter, and taking only one of them is
        a bug this has had twice.

        The last frame alone misses a board that was already dead when the port
        opened: there is no earlier frame to age, so nothing ever fires.

        The open time alone is worse, and only goes wrong once a board has
        worked. Opening the port resets the chip, so it needs a second or two
        of quiet before its first frame. Age that from a frame received before
        the reset and the link is declared silent the instant it opens, which
        resets the board again, forever -- a car that ran a minute ago and now
        will not answer at all, with the log cheerfully reporting a reconnect
        every few seconds.
        """
        since = max(self.telemetry.updated or 0.0, self._opened_at or 0.0)
        return since and (time.monotonic() - since) > SILENCE_S

    def _note_stray(self, line):
        """Pass on what the board says outside the protocol, a little of it.

        When the ESP32 panics it prints why, and a backtrace, on this same
        port before restarting -- the only account of the crash there is.
        Dropping every line that was not a frame threw exactly that away.
        """
        if not line or not any(c.isalpha() for c in line):
            return
        now = time.monotonic()
        if now - getattr(self, "_stray_window", 0.0) > 10.0:
            self._stray_window, self._stray_count = now, 0
        self._stray_count += 1
        if self._stray_count <= 40:
            print("[esp32 says] %s" % line[:200], flush=True)

    def _reopen(self):
        # A reopen that did not buy more than a few seconds of telemetry did
        # not fix anything, so the next one is allowed to reset the board.
        now = time.monotonic()
        self._next_hard = (now - self._last_reopen) < ESCALATE_S
        self._last_reopen = now
        # Under the write lock. Closing while another thread was mid-write
        # let that write land on whatever reused the file descriptor next --
        # it once came back as the body of an HTTP response, a velocity frame
        # sent to a web browser instead of the motors.
        with self.lock:
            self.connected = False
            try:
                self.port.close()
            except Exception:  # noqa: BLE001
                pass

    def _handle(self, line):
        if not line.startswith("$") or "*" not in line:
            self._note_stray(line)
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
            # The board's -1 is kept as -1. It means the pulse timed out with
            # nothing inside about 4m, which is a measurement, and the answer
            # is that the road is clear. Folding it into None here -- which is
            # what this line used to do -- makes it indistinguishable from the
            # board having gone silent, and the safety layer above treats that
            # as a fault and refuses to drive forward. The car then declines to
            # move in precisely the situation where there is nothing in front
            # of it at all.
            #
            # None still means something here: no frame has ever been parsed.
            tel.range_cm = range_cm
            tel.wheels = wheels
            tel.stopped = stopped
            if len(parts) >= 9:
                tel.proximity = (parts[7] == "1", parts[8] == "1")
            if len(parts) >= 12:
                if parts[9] == "1":
                    raw = float(parts[10])
                    tel.heading = -raw if self.heading_reversed else raw
                else:
                    # Not a zero: the board is saying it has no heading to
                    # give, which is a different thing from not turning.
                    tel.heading = None
                tel.bumped = parts[11] == "1"
            tel.updated = time.monotonic()
        elif parts[0] == "!":
            message = ",".join(parts[1:])
            print(f"[esp32] {message}", flush=True)
            self.telemetry.messages.append(message)
            del self.telemetry.messages[:-10]
            if message.startswith("bridge ready"):
                self._board_booted()

    def _board_booted(self):
        """The firmware has just run its setup(). Deal with the consequences.

        This is the case the reconnect logic missed, and it is the common one.
        A board that restarts by itself never breaks the serial port: the
        descriptor stays open, telemetry resumes within a couple of seconds,
        the silence watchdog never fires, and the only trace is one line of
        greeting. Meanwhile setup() has centred the camera servo and cleared
        the velocity, so the state this side believes is now wrong -- and
        wrong in the specific way that matters, because a head reported at
        -45 while actually pointing at 0 files every subsequent range reading
        under the wrong angle and turns the scan table into fiction.

        Velocity needs nothing: the control loop sends one every tick, and a
        firmware that hears nothing stops by itself. The servo has no
        feedback and cannot be asked, so it has to be told again.
        """
        expected = (time.monotonic() - self._last_hard) < EXPECTED_BANNER_S
        if not expected:
            self.board_reboots += 1
            print("[link] board restarted on its own (%d since launch)"
                  % self.board_reboots, flush=True)
        if self.on_reconnect is not None:
            try:
                self.on_reconnect(True)
            except Exception as exc:    # noqa: BLE001
                print(f"[link] reconnect hook failed: {exc}", flush=True)

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

    def zero_heading(self):
        """Call this heading zero. The car has to be still; nothing here
        can check that, so it is asked for rather than done on a timer."""
        return self._write("Z")

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
            "proximity": tel.proximity,
            "heading": tel.heading,
            "bumped": tel.bumped,
            "reboots": self.board_reboots,
            "garbage": self.garbage,
            "messages": list(tel.messages[-3:]),
        }

    def close(self):
        self.running = False
        try:
            self.stop()
            self.port.close()
        except Exception:  # noqa: BLE001
            pass
