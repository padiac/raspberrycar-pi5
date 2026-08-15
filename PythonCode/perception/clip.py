"""Record what the camera sees to a file you can take away.

The MJPEG stream is fine to watch and useless to keep: it is a sequence of
whole JPEGs with no compression between them, so a minute of it is a hundred
megabytes and nothing will play it back as a video. This turns the same frames
into H.264 in an MP4, which is perhaps a twentieth the size and opens
anywhere.

ffmpeg does the encoding, fed the JPEGs the stream already produces. That is
deliberate: those frames are made anyway for the browser, they carry the
detection boxes and whatever the behaviour drew on top, and reusing them means
recording costs no extra capture, no extra encode of the raw frame, and
nothing at all when it is switched off. What you get is exactly what you were
watching.

The Pi 5 has no hardware H.264 encoder -- it was dropped from the 5 -- so this
is libx264 on the CPU. At 640x360 and 15fps that is minor next to the object
detector, but it is not free, and it is the reason the clip is smaller than
the preview rather than the same size.
"""

import os
import shutil
import subprocess
import threading
import time
from datetime import datetime

#: Long edge of the recording. The preview is 1280x720; a clip does not need
#: to be, and halving it quarters the work and the file.
WIDTH = 640
FPS = 15
#: Constant Rate Factor: lower is better and bigger. 26 is a visible step down
#: from the preview but holds up on a phone, which is where these get watched.
CRF = 26
#: Two hard stops, so a forgotten recording cannot fill the card. ffmpeg
#: applies both itself and finishes the file properly when it hits either.
MAX_BYTES = 150 * 1024 * 1024
MAX_SECONDS = 900
#: Refuse to start with less than this free. A full root filesystem on a Pi is
#: not a lost video, it is a robot that will not boot.
MIN_FREE_BYTES = 800 * 1024 * 1024

#: Below this, what was produced is not a video. Players differ on whether
#: they show a frozen frame or simply refuse, so it is deleted and said so
#: rather than handed over as a file that will not open.
MIN_FRAMES = 12
#: How long to give ffmpeg to finish after its input closes. It has to write
#: the index at the end, and a clip killed before that is unplayable.
FINALISE_TIMEOUT_S = 8.0


def _ffmpeg_available():
    return shutil.which("ffmpeg") is not None


class ClipRecorder:
    """Records the stream to an MP4 on demand."""

    def __init__(self, frames, directory, camera_off=None):
        self.frames = frames
        #: Callable answering whether the camera is currently off.
        self.camera_off = camera_off
        self.directory = directory
        os.makedirs(self.directory, exist_ok=True)
        self.available = _ffmpeg_available()
        self.process = None
        self.path = None
        self.started_at = None
        self.frames_written = 0
        self.error = None
        self._thread = None
        self._lock = threading.RLock()

    # -- starting and stopping ---------------------------------------------

    def start(self):
        with self._lock:
            if self.process is not None:
                return False, "already recording"
            if not self.available:
                return False, "ffmpeg is not installed"
            if self.camera_off is not None and self.camera_off():
                # ffmpeg with no input produces no file at all: the UI would say
                # it recorded and the list would stay empty, with nothing
                # anywhere saying why.
                return False, "the camera is off"
            free = shutil.disk_usage(self.directory).free
            if free < MIN_FREE_BYTES:
                return False, f"only {free // (1024 * 1024)}MB free, refusing to record"

            name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".mp4"
            path = os.path.join(self.directory, name)
            try:
                self.process = subprocess.Popen(
                    self._command(path),
                    stdin=subprocess.PIPE,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                )
            except OSError as exc:
                self.error = str(exc)
                self.process = None
                return False, f"could not start ffmpeg: {exc}"

            self.path = path
            self.started_at = time.monotonic()
            self.frames_written = 0
            self.error = None
            self._thread = threading.Thread(target=self._pump, daemon=True)
            self._thread.start()
            return True, name

    def _command(self, path):
        return [
            "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
            # The frames arrive as they arrive; let their real arrival times be
            # the timestamps, or a dropped frame silently speeds the clip up.
            "-use_wallclock_as_timestamps", "1",
            "-f", "image2pipe", "-vcodec", "mjpeg", "-i", "-",
            # A silent audio track. It costs a couple of kilobytes a minute and
            # it is the difference between a file that opens everywhere and one
            # that some players, phone galleries and messaging apps refuse
            # outright -- a video-only MP4 is unusual enough that plenty of
            # software treats it as broken.
            "-f", "lavfi", "-i", "anullsrc=channel_layout=mono:sample_rate=44100",
            "-vf", f"scale={WIDTH}:-2",
            "-r", str(FPS),
            "-c:v", "libx264", "-preset", "veryfast", "-crf", str(CRF),
            # Without this the file plays in ffmpeg and in nothing else --
            # phones and browsers want 4:2:0.
            "-pix_fmt", "yuv420p",
            # Puts the index at the front, so it starts playing before it has
            # finished downloading.
            "-c:a", "aac", "-b:a", "16k",
            # The silence is infinite; stop when the pictures do.
            "-shortest",
            "-movflags", "+faststart",
            "-fs", str(MAX_BYTES),
            "-t", str(MAX_SECONDS),
            path,
        ]

    def _pump(self):
        """Feed frames until told to stop, the file caps out, or ffmpeg dies."""
        interval = 1.0 / FPS
        next_frame = 0.0
        while True:
            with self._lock:
                process = self.process
            if process is None or process.poll() is not None:
                break

            got, jpeg = self.frames.wait_for_next(timeout=1.0)
            now = time.monotonic()
            if not got or jpeg is None or now < next_frame:
                continue
            next_frame = now + interval
            try:
                process.stdin.write(jpeg)
                self.frames_written += 1
            except (BrokenPipeError, ValueError, OSError):
                break

        # ffmpeg stopping on its own means it hit -fs or -t. Tidy up so the
        #status reported afterwards is "not recording" rather than a stuck one.
        with self._lock:
            if self.process is not None and self.process.poll() is not None:
                self._collect()

    def stop(self):
        with self._lock:
            if self.process is None:
                return False, "not recording"
            process, path = self.process, self.path
            try:
                process.stdin.close()
            except (OSError, ValueError):
                pass

        # Outside the lock: this waits, and status() should stay answerable.
        try:
            process.wait(timeout=FINALISE_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()

        written = self.frames_written
        with self._lock:
            self._collect()

        name = os.path.basename(path) if path else "stopped"
        if path and written < MIN_FRAMES:
            try:
                os.remove(path)
            except OSError:
                pass
            return False, f"only {written} frames, too short to be a video"
        return True, name

    def _collect(self):
        """Clear the recording state and keep whatever ffmpeg complained about."""
        process = self.process
        if process is not None and process.stderr is not None:
            try:
                message = process.stderr.read().decode("utf-8", "replace").strip()
                if message:
                    self.error = message.splitlines()[-1]
                    print(f"[clip] ffmpeg: {self.error}", flush=True)
            except (OSError, ValueError):
                pass
        self.process = None
        self.path = None
        self.started_at = None

    def close(self):
        if self.process is not None:
            self.stop()

    # -- what is on the card -----------------------------------------------

    def clips(self):
        out = []
        try:
            names = os.listdir(self.directory)
        except OSError:
            return out
        for name in names:
            if not name.endswith(".mp4"):
                continue
            path = os.path.join(self.directory, name)
            try:
                stat = os.stat(path)
            except OSError:
                continue
            out.append({
                "name": name,
                "bytes": stat.st_size,
                "mb": round(stat.st_size / (1024 * 1024), 1),
                "when": datetime.fromtimestamp(stat.st_mtime).strftime("%d %b %H:%M"),
                # The file being written to is not finished, and offering it
                # for download would hand over an MP4 with no index.
                "recording": self.path == path,
            })
        out.sort(key=lambda c: c["name"], reverse=True)
        return out

    def resolve(self, name):
        """Full path for a clip name, or None if that is not a clip here.

        Rejects anything with a directory in it rather than trying to clean it
        up: the only valid names are ones this class made.
        """
        if not name or name != os.path.basename(name) or not name.endswith(".mp4"):
            return None
        path = os.path.join(self.directory, name)
        return path if os.path.isfile(path) else None

    def delete(self, name):
        path = self.resolve(name)
        if path is None:
            return False, "no such clip"
        if path == self.path:
            return False, "that one is still recording"
        try:
            os.remove(path)
        except OSError as exc:
            return False, str(exc)
        return True, "deleted"

    def status(self):
        with self._lock:
            recording = self.process is not None
            seconds = (
                round(time.monotonic() - self.started_at, 1)
                if recording and self.started_at else 0.0
            )
            name = os.path.basename(self.path) if self.path else None
        try:
            free_mb = shutil.disk_usage(self.directory).free // (1024 * 1024)
        except OSError:
            free_mb = None
        return {
            "available": self.available,
            "recording": recording,
            "name": name,
            "seconds": seconds,
            "frames": self.frames_written if recording else 0,
            "free_mb": free_mb,
            "error": self.error,
        }
