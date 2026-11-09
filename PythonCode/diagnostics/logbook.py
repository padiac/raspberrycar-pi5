"""Logging that cannot fill the card.

Three things are wanted from a robot's logs and they pull against each other:
keep enough history to explain a fault that happens once an hour, keep enough
detail to explain one that lasts 200ms, and never run the SD card out of
space. The card is 29GB with under 10GB free, and a car that bricks its own
rootfs to record why it bumped a chair leg has not helped anybody.

The resolution is to stop treating "do not fill the disk" as a chore to be
done periodically. A sweep that runs every hour leaves an hour in which the
disk can fill, and the failure it is meant to prevent happens inside that
hour or not at all. So the bound here is structural: every file is written
through a handler that rotates at a fixed size and keeps a fixed number of
backups, and those two numbers multiply out to a ceiling the writer cannot
exceed even in principle. BUDGET below is that arithmetic, asserted at import
so that raising a limit without redoing the sum is a crash rather than a
surprise three weeks later.

The periodic sweep still exists, as the second line rather than the first. It
catches what the arithmetic cannot: files left by an older version with
different limits, a flight recording somebody started and forgot, and the
case the ceiling cannot help with at all -- the disk filling up because of
something else entirely, which is when a robot most needs to not be adding
to it.

Two streams, because they answer different questions.

car.log is the narrative: every line the program prints, timestamped, which
means the existing print() calls all over this codebase keep working and land
in the file without being rewritten into logger calls. systemd already
collects stdout, but journald here is volatile -- it keeps nothing across a
reboot, and this car has been rebooting on its own, so the log that would
explain the reboot is destroyed by it. A file on disk survives.

flight.csv is the flight recorder: machine-readable state at a few hertz, for
the faults that are over before a human can look. It is deliberately not a
dump of everything at 20Hz, which is tens of megabytes an hour of mostly
identical rows. It samples, and writes a row only when something changed or a
second has passed, so standing still is nearly free while the interesting
seconds are recorded densely.
"""

import csv
import io
import logging
import logging.handlers
import os
import shutil
import sys
import threading
import time

#: Where the logs go: a sibling of PythonCode, not inside it.
DEFAULT_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "logs",
)

#: The narrative log. Six files of two megabytes.
NARRATIVE_BYTES = 2 * 1024 * 1024
NARRATIVE_KEEP = 6

#: The flight recorder. Fewer, larger files: a CSV is only useful in runs.
FLIGHT_BYTES = 4 * 1024 * 1024
FLIGHT_KEEP = 5

#: What the two above can occupy between them, by construction. A handler
#: rotates when the file it is writing passes its limit, so each stream can
#: hold its limit times one more than its backup count.
BUDGET = (NARRATIVE_BYTES * (NARRATIVE_KEEP + 1)
          + FLIGHT_BYTES * (FLIGHT_KEEP + 1))
assert BUDGET <= 48 * 1024 * 1024, "log ceiling has grown past 48MB"

#: Anything in the log directory older than this is swept, whatever it is.
MAX_AGE_DAYS = 14

#: Below this much free space the sweep stops being polite: it deletes the
#: oldest rotated files until either only the live files are left or the
#: floor is cleared. No log is worth more than a working rootfs.
FREE_FLOOR_MB = 800

#: How often the sweep runs. Rotation is what keeps the size bounded, so this
#: is not load-bearing and does not need to be frequent.
SWEEP_S = 1800.0

#: How often the flight recorder looks at the robot.
FLIGHT_HZ = 5.0

#: A row is written even when nothing has changed, this often, so that a
#: recording has a heartbeat and a gap in it means something.
FLIGHT_KEEPALIVE_S = 1.0

#: ...but a car parked for an hour does not need thirty-six hundred identical
#: rows to prove it was parked, and at that rate a day of standing still is
#: enough to push a whole patrol run out of the window. So once nothing has
#: changed for a while the heartbeat slows to this, and speeds back up the
#: instant anything moves. Density where it is useful, a pulse where it is
#: not.
FLIGHT_IDLE_KEEPALIVE_S = 15.0

#: How long without a change counts as parked.
FLIGHT_SETTLE_S = 30.0

#: Numeric fields closer together than this count as unchanged.
FLIGHT_EPSILON = 0.02


class _Rotating(logging.handlers.RotatingFileHandler):
    """RotatingFileHandler, used for its rotation and not its formatting.

    Everything here writes to the stream directly and asks for a rollover
    afterwards, because the lines being written are already formatted -- by
    the print that produced them, or by the CSV writer -- and passing them
    through a Formatter would only mean finding a format that does nothing.
    """

    def over_limit(self):
        try:
            return self.stream.tell() >= self.maxBytes
        except Exception:       # noqa: BLE001
            return False

    def put(self, text):
        """Write already-formatted text, rotating if that filled the file."""
        try:
            self.stream.write(text)
            self.stream.flush()
            if self.over_limit():
                self.doRollover()
                return True
        except Exception:       # noqa: BLE001 - logging is never fatal
            pass
        return False


class _Tee:
    """A stdout that also writes timestamped lines to a rotating file.

    Wrapping the stream rather than rewriting several hundred print() calls
    is a deliberate trade. The cost is that the file gets whatever shape the
    prints already have; the gain is that nothing has to be migrated, no
    print added later can forget to be logged, and output from inside a
    library -- which cannot be made to use our logger -- is captured too.
    """

    def __init__(self, stream, handler):
        self._stream = stream
        self._handler = handler
        self._partial = ""
        self._lock = threading.Lock()

    def write(self, text):
        self._stream.write(text)
        with self._lock:
            self._partial += text
            while "\n" in self._partial:
                line, self._partial = self._partial.split("\n", 1)
                line = line.rstrip()
                if line:
                    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
                    self._handler.put("%s %s\n" % (stamp, line))
        return len(text)

    def flush(self):
        try:
            self._stream.flush()
        except Exception:       # noqa: BLE001
            pass

    def isatty(self):
        return getattr(self._stream, "isatty", lambda: False)()

    @property
    def encoding(self):
        return getattr(self._stream, "encoding", "utf-8")


class Logbook:
    """The log directory, its two streams, and the sweep that guards it."""

    def __init__(self, directory=DEFAULT_DIR):
        self.directory = directory
        self.installed = False
        self.sweeps = 0
        self.deleted = 0
        self._narrative = None
        self._flight = None
        self._flight_handler = None
        self._flight_fields = None
        self._flight_last = None
        self._flight_wrote = 0.0
        self._flight_changed = 0.0
        self._flight_rows = 0
        self._source = None
        self._stop = threading.Event()
        os.makedirs(self.directory, exist_ok=True)

    # -- narrative ---------------------------------------------------------

    def install(self):
        """Start capturing stdout and stderr. Safe to call twice."""
        if self.installed:
            return self
        self._narrative = _Rotating(
            os.path.join(self.directory, "car.log"),
            maxBytes=NARRATIVE_BYTES,
            backupCount=NARRATIVE_KEEP,
            encoding="utf-8",
        )
        sys.stdout = _Tee(sys.stdout, self._narrative)
        # stderr shares the file: a traceback belongs next to the lines that
        # led to it, not in a second file nobody thinks to open.
        sys.stderr = _Tee(sys.stderr, self._narrative)
        self.installed = True
        print("[log] %s, ceiling %.0fMB, sweep every %.0fmin"
              % (self.directory, BUDGET / 1048576.0, SWEEP_S / 60.0))
        threading.Thread(target=self._sweep_loop, daemon=True).start()
        return self

    # -- flight recorder ---------------------------------------------------

    def record(self, source):
        """Start the flight recorder, sampling source() for a flat dict."""
        if self._flight is not None:
            return False
        self._source = source
        self._flight_handler = _Rotating(
            os.path.join(self.directory, "flight.csv"),
            maxBytes=FLIGHT_BYTES,
            backupCount=FLIGHT_KEEP,
            encoding="utf-8",
        )
        self._flight_last = None
        self._flight_fields = None
        self._stop.clear()
        self._flight = threading.Thread(target=self._flight_loop, daemon=True)
        self._flight.start()
        print("[log] flight recorder on, %.0fHz" % FLIGHT_HZ)
        return True

    def stop_recording(self):
        if self._flight is None:
            return False
        thread, self._flight = self._flight, None
        self._stop.set()
        thread.join(timeout=2.0)
        handler, self._flight_handler = self._flight_handler, None
        try:
            handler.close()
        except Exception:       # noqa: BLE001
            pass
        print("[log] flight recorder off, %d rows" % self._flight_rows)
        return True

    @property
    def recording(self):
        return self._flight is not None

    def _flight_loop(self):
        period = 1.0 / FLIGHT_HZ
        while self._flight is not None and not self._stop.wait(period):
            try:
                row = self._source()
            except Exception as exc:    # noqa: BLE001
                print("[log] flight source failed: %s" % exc)
                return
            if not isinstance(row, dict):
                return
            now = time.monotonic()
            if not self._worth_writing(row, now):
                continue
            self._flight_last = dict(row)
            self._flight_wrote = now
            self._write_row(row)

    def _worth_writing(self, row, now):
        """True if this row says something the last one did not.

        The point is not to save bytes for their own sake; it is that a file
        of identical rows is unreadable. Dropping the repeats means a human
        scrolling the CSV sees one line per thing that actually happened.
        """
        if self._flight_last is None:
            self._flight_changed = now
            return True
        for key, value in row.items():
            was = self._flight_last.get(key)
            changed = False
            if isinstance(value, bool) or isinstance(was, bool):
                changed = value != was
            elif isinstance(value, (int, float)) and isinstance(was, (int, float)):
                changed = abs(value - was) > FLIGHT_EPSILON
            else:
                changed = value != was
            if changed:
                self._flight_changed = now
                return True
        quiet = now - (self._flight_changed or now)
        keepalive = (FLIGHT_KEEPALIVE_S if quiet < FLIGHT_SETTLE_S
                     else FLIGHT_IDLE_KEEPALIVE_S)
        return now - self._flight_wrote >= keepalive

    def _write_row(self, row):
        handler = self._flight_handler
        if handler is None:
            return
        if self._flight_fields is None:
            self._flight_fields = ["time"] + sorted(row)
            try:
                fresh = handler.stream.tell() == 0
            except Exception:   # noqa: BLE001
                fresh = False
            if fresh:
                handler.put(",".join(self._flight_fields) + "\n")
        buffer = io.StringIO()
        writer = csv.writer(buffer, lineterminator="\n")
        stamp = "%s.%03d" % (time.strftime("%H:%M:%S"), (time.time() % 1) * 1000)
        writer.writerow([stamp] + [row.get(f, "") for f in self._flight_fields[1:]])
        if handler.put(buffer.getvalue()):
            self._flight_fields = None      # header again in the new file
        self._flight_rows += 1

    # -- the sweep ---------------------------------------------------------

    def _sweep_loop(self):
        while True:
            time.sleep(SWEEP_S)
            try:
                self.sweep()
            except Exception as exc:    # noqa: BLE001
                print("[log] sweep failed: %s" % exc)

    def _live(self):
        """Files the handlers are writing right now: never delete these."""
        live = {"car.log"}
        if self._flight_handler is not None:
            live.add("flight.csv")
        return live

    def files(self):
        out = []
        try:
            names = sorted(os.listdir(self.directory))
        except OSError:
            return out
        for name in names:
            path = os.path.join(self.directory, name)
            if not os.path.isfile(path):
                continue
            try:
                stat = os.stat(path)
            except OSError:
                continue
            out.append({"name": name, "bytes": stat.st_size, "mtime": stat.st_mtime})
        return out

    def sweep(self):
        """Second line of defence. Rotation is the first."""
        self.sweeps += 1
        live = self._live()
        spare = [f for f in self.files() if f["name"] not in live]
        spare.sort(key=lambda f: f["mtime"])
        now = time.time()
        removed = 0

        # 1. Age. Catches files an older version of this code left behind
        #    under names nothing rotates any more.
        for entry in list(spare):
            if now - entry["mtime"] > MAX_AGE_DAYS * 86400:
                removed += self._remove(entry["name"])
                spare.remove(entry)

        # 2. Budget. Belt to the arithmetic's braces: if the directory is over
        #    the ceiling then something is in here the handlers did not put
        #    there, and the oldest of it goes first.
        total = sum(f["bytes"] for f in self.files())
        while total > BUDGET and spare:
            entry = spare.pop(0)
            if self._remove(entry["name"]):
                removed += 1
                total -= entry["bytes"]

        # 3. Free space, which the ceiling cannot help with: the disk can be
        #    filling because of something else entirely, and that is exactly
        #    when a robot should stop contributing.
        free_mb = self._free_mb()
        while free_mb < FREE_FLOOR_MB and spare:
            entry = spare.pop(0)
            removed += self._remove(entry["name"])
            free_mb = self._free_mb()

        if removed:
            print("[log] sweep removed %d file(s), %.0fMB free" % (removed, free_mb))
        return removed

    def _free_mb(self):
        try:
            return shutil.disk_usage(self.directory).free / 1048576.0
        except OSError:
            return float("inf")

    def _remove(self, name):
        try:
            os.remove(os.path.join(self.directory, name))
            self.deleted += 1
            return 1
        except OSError:
            return 0

    # -- reading it back ---------------------------------------------------

    def tail(self, lines=200, name="car.log"):
        """Last few lines of a log file, for the web page.

        Reads from the end, so tailing a two megabyte file does not cost two
        megabytes, and refuses anything that is not a plain filename, so that
        a URL cannot walk out of the directory.
        """
        if os.path.basename(name) != name or name.startswith("."):
            return []
        path = os.path.join(self.directory, name)
        try:
            size = os.path.getsize(path)
            window = min(size, max(4096, lines * 160))
            with open(path, "rb") as handle:
                handle.seek(size - window)
                blob = handle.read().decode("utf-8", "replace")
        except OSError:
            return []
        out = blob.splitlines()
        if window < size and out:
            out = out[1:]       # the first line is probably half a line
        return out[-lines:]

    def status(self):
        files = self.files()
        return {
            "directory": self.directory,
            "installed": self.installed,
            "recording": self.recording,
            "rows": self._flight_rows,
            "bytes": sum(f["bytes"] for f in files),
            "ceiling": BUDGET,
            "free_mb": round(self._free_mb()),
            "sweeps": self.sweeps,
            "deleted": self.deleted,
            "files": files,
        }


#: One per process: the web layer and the robot both need to reach it.
book = Logbook()
