"""Laptop / USB webcam / UVC capture card via OpenCV.

A drone camera on an analog 5.8 GHz link shows up on the laptop as a plain UVC
video device (a USB capture dongle, an FPV receiver with USB out, or a DJI/
GoPro style USB webcam mode). OpenCV sees it as just another camera, so the
only things that matter are: which device index, the exact UVC mode, and the
buffer size. All three are configurable here - a capture card that is left at
its default 30 fps and never drained hands the eye 200 ms old frames.
"""

from __future__ import annotations

import platform
import time

import numpy as np


def parse_camera(value: str | int) -> int | str:
    """``"0"``/``"1"`` -> int index, anything else -> device node / path.

    On Windows "0" can be ambiguous (OpenCV prefers DSHOW, MSMF and Media
    Foundation agree); a capture card is often easier to pick by its DirectShow
    name, which OpenCV accepts as a string, e.g. ``--camera "USB Video"``.
    """
    s = str(value).strip()
    return int(s) if s.isdigit() else s


def list_cameras(max_index: int = 8) -> list[tuple[int | str, str]]:
    """Probe camera indices and return the ones that open.

    Only useful for USB/UVC devices. On Windows the same physical camera can
    show up under several backends, so duplicates are possible - that is why
    ``--camera`` takes a name as well as an index.
    """
    try:
        import cv2
    except ImportError:  # pragma: no cover - optional dependency
        return []
    found: list[tuple[int | str, str]] = []
    for i in range(max_index):
        for backend, label in _backends(cv2):
            cap = None
            try:
                cap = cv2.VideoCapture(i, backend) if backend else cv2.VideoCapture(i)
                if not cap.isOpened():
                    continue
                ok, _ = cap.read()
                if not ok:
                    continue
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
                fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
                found.append((i, f"index {i} [{label}] {w}x{h} @ {fps:.0f} fps"))
                break
            except Exception:
                continue
            finally:
                if cap is not None:
                    cap.release()
    return found


def _backends(cv2) -> list[tuple[int, str]]:
    if platform.system() == "Windows" and hasattr(cv2, "CAP_DSHOW"):
        return [(cv2.CAP_DSHOW, "DirectShow"), (0, "any")]
    if platform.system() == "Linux" and hasattr(cv2, "CAP_V4L2"):
        return [(cv2.CAP_V4L2, "V4L2"), (0, "any")]
    return [(0, "any")]


class Webcam:
    """Latest camera frame as BGR uint8, or None if the device hiccups."""

    def __init__(self, index: int | str = 0, width: int = 640, height: int = 480, fps: int = 30,
                 mirror: bool = True, mjpg: bool = False, buffersize: int = 1, backend: str = "dshow",
                 warmup: int = 10, strict: bool = False):
        try:
            import cv2
        except ImportError as e:  # pragma: no cover
            raise SystemExit("OpenCV missing: pip install 'flydrones[vision]'") from e
        self.cv2 = cv2
        self.mirror = mirror
        self.index = parse_camera(index)
        self.width, self.height = int(width), int(height)
        self.fps = int(fps)
        self.strict = bool(strict)
        self.warnings: list[str] = []

        api = self._api(backend)
        self.cap = cv2.VideoCapture(self.index, api) if api else cv2.VideoCapture(self.index)
        # MJPG keeps 640x480 USB traffic low enough that the frame rate is real;
        # most UVC capture cards only offer 30 fps in YUYV at 320x240.
        fourcc = cv2.VideoWriter_fourcc(*("MJPG" if mjpg else "YUYV"))
        for prop, want in ((cv2.CAP_PROP_FOURCC, fourcc), (cv2.CAP_PROP_FRAME_WIDTH, self.width),
                           (cv2.CAP_PROP_FRAME_HEIGHT, self.height), (cv2.CAP_PROP_FPS, self.fps),
                           (cv2.CAP_PROP_BUFFERSIZE, int(buffersize))):
            try:
                self.cap.set(prop, want)
            except Exception:
                pass

        if not self.cap.isOpened():
            msg = f"cannot open camera {self.index!r} (backend {backend}). Try 'flydrones camera --list'"
            if self.strict:
                raise SystemExit(msg)
            raise SystemExit(msg)

        # A real capture stream needs a moment to settle; the first frames are
        # often black. These are also what tells us the device really works.
        got = 0
        deadline = time.monotonic() + max(0.5, warmup * 0.05)
        while time.monotonic() < deadline:
            ok, frame = self.cap.read()
            if ok and frame is not None and frame.size:
                got += 1
        self.actual = (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0),
                       int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0),
                       float(self.cap.get(cv2.CAP_PROP_FPS) or 0.0))
        if got == 0:
            self.warnings.append(f"camera {self.index!r} opened but returned no frame")
        if self.actual[0] and (self.actual[0], self.actual[1]) != (self.width, self.height):
            self.warnings.append(f"camera delivered {self.actual[0]}x{self.actual[1]}, not {self.width}x{self.height}")

    def _api(self, backend: str) -> int:
        if backend in ("auto", "any", "", None):
            return 0
        table = {"dshow": "CAP_DSHOW", "msmf": "CAP_MSMF", "v4l2": "CAP_V4L2", "gstreamer": "CAP_GSTREAMER"}
        name = table.get(str(backend).lower())
        return int(getattr(self.cv2, name)) if name and hasattr(self.cv2, name) else 0

    def describe(self) -> str:
        w, h, fps = self.actual
        return f"{self.index!r} -> {w}x{h} @ {fps:.0f} fps" + (f" (asked for {self.width}x{self.height} @ {self.fps})" if (w, h) != (self.width, self.height) else "")

    def read(self) -> np.ndarray | None:
        ok, frame = self.cap.read()
        if not ok or frame is None or not frame.size:
            return None
        return self.cv2.flip(frame, 1) if self.mirror else frame

    def close(self) -> None:
        self.cap.release()


class VideoSource:
    """One UVC device, shared by every consumer.

    Two ``Webcam`` objects on the same device index fight over the driver and
    halve each other's frame rate, so a single source object is created per
    capture device and handed out to whoever needs frames (retina, recorder,
    dashboard). ``Webcam`` is kept as the thin single-consumer wrapper.
    """

    def __init__(self, index: int | str = 0, **kw):
        self.device = parse_camera(index)
        self.cam = Webcam(self.device, **kw)

    def capture(self) -> np.ndarray | None:
        return self.cam.read()

    def describe(self) -> str:
        return self.cam.describe()

    def close(self) -> None:
        self.cam.close()
