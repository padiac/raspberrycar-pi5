"""The robot: owns every piece of hardware, runs one control loop.

The original project had six separate programs, each grabbing the camera and
the GPIO for itself, so only one behaviour could ever run and you couldn't
watch it while it did. Here a single process holds the hardware and the active
*mode* decides what to do with it, which means behaviours can be swapped live,
the video feed keeps running across the switch, and you can always take back
control.
"""

import threading
import time

from picamera2 import Picamera2

from modes import Perception, build_modes
from vision import STREAM_SIZE, CameraPipeline

CONTROL_HZ = 20.0


def _try_init(name, factory):
    """Build a hardware object, returning None (not raising) if it isn't there."""
    try:
        obj = factory()
        print(f"[hw] {name}: ok", flush=True)
        return obj
    except Exception as exc:  # noqa: BLE001 - missing hardware must not be fatal
        print(f"[hw] {name}: unavailable ({exc})", flush=True)
        return None


class Robot:
    def __init__(self):
        self.move = _try_init("motors", lambda: __import__("move").CarMove())
        self.ultrasound = _try_init(
            "ultrasound", lambda: __import__("ultrasound").CarUltrasound()
        )
        self.infrared = _try_init(
            "infrared", lambda: __import__("infrared").CarInfrared()
        )
        self.detector = _try_init("detector", lambda: __import__("detector").Detector())

        picam = Picamera2()
        picam.configure(
            picam.create_video_configuration(
                main={"size": STREAM_SIZE, "format": "RGB888"}
            )
        )
        picam.start()
        time.sleep(1.5)
        self.vision = CameraPipeline(picam, self.detector)
        print(f"[hw] camera: streaming at {STREAM_SIZE[0]}x{STREAM_SIZE[1]}", flush=True)

        self.modes = build_modes()
        self.mode = self.modes["idle"]
        self.vision.overlay_hook = self.mode.overlay

        self.throttle = 0.0
        self.steer = 0.0
        self.left = 0.0
        self.right = 0.0
        self.mode_error = None

        self.lock = threading.RLock()
        self.running = True
        threading.Thread(target=self._control_loop, daemon=True).start()

    # -- capability reporting ---------------------------------------------

    @property
    def available(self):
        return {
            "motors": self.move is not None,
            "ultrasound": self.ultrasound is not None,
            "infrared": self.infrared is not None,
            "detector": self.detector is not None,
            "camera": self.vision.camera_enabled,
        }

    def missing_for(self, mode):
        have = self.available
        return [need for need in mode.requires if not have.get(need, False)]

    def mode_list(self):
        out = []
        for mode in self.modes.values():
            out.append(
                {
                    "name": mode.name,
                    "label": mode.label,
                    "description": mode.description,
                    "requires": list(mode.requires),
                    "missing": self.missing_for(mode),
                    "active": mode is self.mode,
                }
            )
        return out

    # -- mode switching ----------------------------------------------------

    def select_mode(self, name):
        """Switch modes. Returns (ok, message)."""
        mode = self.modes.get(name)
        if mode is None:
            return False, f"unknown mode '{name}'"

        missing = self.missing_for(mode)
        if missing:
            return False, f"{mode.label} needs: {', '.join(missing)}"

        with self.lock:
            if mode is self.mode:
                return True, f"already in {mode.label}"

            self.mode.stop(self)
            self._apply(0.0, 0.0)          # never carry speed across a switch

            self.mode = mode
            self.mode_error = None
            self.vision.overlay_hook = mode.overlay
            if mode.wants_detection:
                self.vision.set_detection(True)
            mode.start(self)

        return True, f"switched to {mode.label}"

    def stop(self):
        """Panic button: motors off, back to the idle mode."""
        self.select_mode("idle")
        with self.lock:
            self._apply(0.0, 0.0)

    def manual_command(self, throttle, steer):
        """Feed the manual mode. Auto-selects it so the controls just work."""
        if self.mode.name != "manual":
            ok, message = self.select_mode("manual")
            if not ok:
                return False, message
        self.modes["manual"].command(throttle, steer)
        return True, "ok"

    # -- the loop -----------------------------------------------------------

    def _perceive(self, dt):
        distance = None
        if self.ultrasound is not None:
            try:
                distance = self.ultrasound.DistMeasureMovingAverage()
            except Exception:  # noqa: BLE001 - a flaky sensor shouldn't kill the loop
                distance = None

        obstacle = tracking = None
        if self.infrared is not None:
            try:
                obstacle = self.infrared.InfraredMeasure()
                tracking = self.infrared.TrackingMreasure()
            except Exception:  # noqa: BLE001
                pass

        frame = self.vision.snapshot_frame()
        return Perception(
            frame=frame,
            detections=self.vision.snapshot_detections(),
            distance_cm=distance,
            obstacle=obstacle,
            tracking=tracking,
            frame_size=self.vision.frame_size,
            dt=dt,
        )

    def _apply(self, throttle, steer):
        """Mix throttle+steer into per-side speeds and drive the motors."""
        throttle = max(-1.0, min(1.0, throttle))
        steer = max(-1.0, min(1.0, steer))

        # Scale back so a hard turn at full throttle doesn't clip both sides to
        # the same value, which would drive straight instead of turning.
        left = throttle + steer
        right = throttle - steer
        peak = max(1.0, abs(left), abs(right))
        left, right = left / peak, right / peak

        self.throttle, self.steer = throttle, steer
        self.left, self.right = left, right

        if self.move is not None:
            self.move.set_motors(left, right)

    def _control_loop(self):
        period = 1.0 / CONTROL_HZ
        previous = time.monotonic()
        while self.running:
            time.sleep(period)
            now = time.monotonic()
            dt, previous = now - previous, now

            try:
                view = self._perceive(dt)
                with self.lock:
                    mode = self.mode
                    throttle, steer = mode.update(self, view)
                    self._apply(throttle, steer)
                    self.mode_error = None
            except Exception as exc:  # noqa: BLE001 - a bad mode must not run away
                with self.lock:
                    self.mode_error = f"{type(exc).__name__}: {exc}"
                    self._apply(0.0, 0.0)
                print(f"[control] {self.mode.name}: {exc}", flush=True)
                time.sleep(0.5)

    # -- telemetry ----------------------------------------------------------

    def telemetry(self):
        view_distance = None
        if self.ultrasound is not None:
            try:
                view_distance = round(self.ultrasound.dist_mov_ave, 1)
            except Exception:  # noqa: BLE001
                view_distance = None

        obstacle = tracking = None
        if self.infrared is not None:
            try:
                obstacle = self.infrared.InfraredMeasure()
                tracking = self.infrared.TrackingMreasure()
            except Exception:  # noqa: BLE001
                pass

        return {
            "mode": self.mode.name,
            "mode_label": self.mode.label,
            "mode_status": self.mode.status(),
            "mode_error": self.mode_error,
            "throttle": round(self.throttle, 2),
            "steer": round(self.steer, 2),
            "left": round(self.left, 2),
            "right": round(self.right, 2),
            "distance_cm": view_distance,
            "obstacle": obstacle,
            "tracking": tracking,
            "stream_fps": round(self.vision.stream_fps, 1),
            "detect_fps": round(self.vision.detect_fps, 1),
            "detect_enabled": self.vision.detect_enabled,
            "camera_enabled": self.vision.camera_enabled,
            "objects": [
                {"label": label, "conf": round(conf, 2)}
                for label, conf, _box, _cls in self.vision.snapshot_detections()
            ],
            "backend": self.detector.backend if self.detector else None,
            "available": self.available,
        }

    def close(self):
        self.running = False
        self.stop()
        self.vision.close()
