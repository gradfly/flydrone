"""USB serial link to a quadcopter flight controller.

This is the adapter for the "real camera on the drone -> USB UVC capture card",
"flight controller on a USB serial port" setup, i.e. a laptop in the middle:

    drone camera --analog 5.8G--> USB UVC capture  ─┐
                                                     ├─ laptop: fly brain
    flight controller  <--USB serial (UART)----------┘

Wire protocol, 4 bytes every control tick (20 Hz by default):

    +--------+----------+------+---------+
    |  0xAA  | throttle | yaw  | forward |
    +--------+----------+------+---------+
      frame    int8       int8   int8        each -120..+120
      header   -120 down  -120 left  -120 back
               +120 up    +120 right +120 forward

The three values are the brain's normalised command (``-1..1``) scaled to
``-scale..+scale`` and rounded to the nearest integer (the default scale is
120). **They are signed bytes** (two's complement) - see ``docs/HARDWARE.md``
for the reference receiver, which is what the tests check against.

A command is sent every tick whether or not you asked for it:

* every ``hz`` tick the latest command is written to the port (default 20 Hz)
* the serial link is opened **once**, configured **once** and never written
  from two threads, so the stream cannot interleave into garbage frames
* while the link is dead (cable pulled, port disappeared) writes are skipped
  and reconnection is attempted with a small backoff, so a stuck USB port can
  never block the 20 Hz control loop
* on exit an explicit zero command is flushed before the port closes

Safety: as with every backend, nothing is sent unless you add ``--send``.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass

from ..motor.command import FlightCommand
from ..safety import Telemetry
from .base import Drone

FRAME_HEADER = 0xAA
PAYLOAD_AXES = ("throttle", "yaw", "forward")  # order inside the packet
MIN_VALUE, MAX_VALUE = -120, 120


# --------------------------------------------------------------------- packet
def _to_int8(v: float) -> int:
    """Signed int -> two's complement byte. 200 and -56 are the same wire byte."""
    return int(v) & 0xFF


def _from_int8(b: int) -> int:
    """Two's complement byte -> signed int (0x88 -> -120)."""
    return b - 256 if b > 127 else b


def scale_axis(value: float, scale: int = 120) -> int:
    """Normalised ``-1..1`` (or a raw stick value) -> integer in ``-scale..scale``.

    Non-finite input (NaN from a stalled decoder) becomes 0, so a bad number can
    never reach the motors as a random byte.
    """
    v = float(value)
    if not math.isfinite(v):
        return 0
    if abs(v) > 1.0:  # already in raw units (e.g. -120..120 from an external source)
        return int(max(-scale, min(scale, round(v))))
    return int(round(max(-1.0, min(1.0, v)) * scale))


@dataclass
class SerialPacket:
    """One decoded 4-byte frame from the flight controller (usually telemetry)."""

    throttle: int
    yaw: int
    forward: int
    raw: bytes

    def as_cmd(self, scale: int = 120) -> FlightCommand:
        s = float(scale) or 1.0
        return FlightCommand(throttle=self.throttle / s, yaw=self.yaw / s, forward=self.forward / s)


def encode_frame(cmd: FlightCommand, scale: int = 120) -> bytes:
    """``FlightCommand`` -> ``AA THR YAW FWD`` (4 bytes, signed values)."""
    return bytes([FRAME_HEADER, *(_to_int8(scale_axis(getattr(cmd, axis), scale)) for axis in PAYLOAD_AXES)])


def probe_frame(buf: bytearray, scale: int = 120) -> SerialPacket | None:
    """Read one frame off the front of ``buf``, resynchronising on the header.

    A frame is ``AA`` followed by three values that must each be inside
    ``-scale..scale``. On a serial line that is the only plausible meaning of
    an ``0xAA``, so it is a cheap sanity check; requiring all three keeps a
    stray ``0xAA`` inside video/telemetry noise from swallowing a real frame
    (which would then have moved the 4-byte alignment for good).

    Returns ``None`` and leaves ``buf`` short when there is nothing to read yet.
    Any byte before the header is dropped.
    """
    limit = min(127, scale)  # a value byte must stay inside one signed byte
    while len(buf) >= 4:
        if buf[0] != FRAME_HEADER:
            del buf[0]
            continue
        if all(-limit <= _from_int8(b) <= limit for b in buf[1:4]):
            chunk = bytes(buf[:4])
            del buf[:4]
            return SerialPacket(_from_int8(chunk[1]), _from_int8(chunk[2]), _from_int8(chunk[3]), chunk)
        del buf[0]  # 0xAA but not a frame: noise, resync past it
    return None


def decode_frames(buf: bytearray, scale: int = 120) -> list[SerialPacket]:
    """Every complete frame currently in ``buf`` (noise and partial frames are dropped)."""
    out: list[SerialPacket] = []
    while (pkt := probe_frame(buf, scale)) is not None:
        out.append(pkt)
    return out


# ---------------------------------------------------------------------- drone
class SerialDrone(Drone):
    """Flight controller on a USB serial port, fed the 4-byte stick protocol."""

    name = "serial"
    has_camera = False  # the camera comes in through UVC, see --camera

    def __init__(self, port: str = "COM3", baud: int = 115200, scale: int = 120, hz: float = 20.0,
                 telemetry: bool = False, reconnect_s: float = 1.0, cells: int = 4,
                 arm_after_s: float = 0.0, send_zero_before_arm: bool = True, takeoff_hold_s: float = 0.0,
                 zero_frame: bool = False):
        self.port = port
        self.baud = int(baud)
        self.scale = int(scale)
        self.hz = float(hz)
        self.period = 1.0 / self.hz if self.hz > 0 else 0.02
        self.reconnect_s = float(reconnect_s)
        self.cells = int(cells)
        self.arm_after_s = float(arm_after_s)
        self.takeoff_hold_s = float(takeoff_hold_s)
        self.zero_frame = bool(zero_frame)  # Opt. B: all-zero payload = disarmed
        self._arm_with_zero = bool(send_zero_before_arm)
        self._want_telemetry = bool(telemetry)

        self.serial = None
        self.armed = False
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._reconnect_at = 0.0
        self._reader: threading.Thread | None = None
        self._buf = bytearray()

        self._last_cmd = FlightCommand()
        self._last_rx = 0.0
        self._last_tx = 0.0  # last tick the rate limiter let a frame through
        self._start = None
        self.frames_sent = 0
        self.bytes_sent = 0
        self.link_errors = 0
        self.received: list[SerialPacket] = []
        self._tel = Telemetry()

    # ------------------------------------------------------------------ link
    def _open(self) -> None:
        try:
            import serial  # pyserial
        except ImportError as e:  # pragma: no cover - optional dependency
            raise SystemExit("pyserial missing: pip install 'flydrones[serial]'") from e
        self.serial = serial.Serial()
        self.serial.port = self.port
        self.serial.baudrate = self.baud
        # Modem lines are not wired on most FC boards and DTR toggling resets
        # some of them, so we promise pyserial not to touch them.
        self.serial.dtr = False
        self.serial.rts = False
        self.serial.open()
        self._buf.clear()
        self.bytes_sent = 0
        print(f"serial: opened {self.port} @ {self.baud} baud, frame AA THR YAW FWD, scale +-{self.scale}, {self.hz:g} Hz")

    def connect(self) -> None:
        self._open()
        if self._want_telemetry:
            self._stop.clear()
            self._reader = threading.Thread(target=self._read_loop, name="flydrones-serial-rx", daemon=True)
            self._reader.start()

    def _send(self, payload: bytes) -> None:
        if self.serial is None:
            return
        try:
            self.serial.write(payload)
        except Exception as exc:  # serial.SerialException, OSError, ...
            self.link_errors += 1
            self._tel.link_ok = False
            if self.link_errors <= 3:
                print(f"serial: write failed ({exc}) - skipping frames, will retry")
            self._drop()
            return
        self.frames_sent += 1
        self.bytes_sent += len(payload)

    def _drop(self) -> None:
        """Close the link; writes fall back to no-ops until it comes back."""
        try:
            if self.serial is not None:
                self.serial.close()
        except Exception:
            pass
        self.serial = None
        self._reconnect_at = time.monotonic() + self.reconnect_s

    def _maybe_reconnect(self) -> None:
        if self.serial is not None or time.monotonic() < self._reconnect_at:
            return
        try:
            self._open()
            self.link_errors = 0
        except Exception:
            self._reconnect_at = time.monotonic() + self.reconnect_s

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            ser = self.serial
            if ser is None:
                self._maybe_reconnect()
                time.sleep(0.05)
                continue
            try:
                n = ser.in_waiting
                data = ser.read(n or 1)
            except Exception:
                self._drop()
                continue
            if not data:
                continue
            self._buf.extend(data)
            for pkt in decode_frames(self._buf, self.scale):
                self._last_rx = time.monotonic()
                self.received.append(pkt)
                del self.received[:-64]
                self._tel.link_ok = True
                self._tel.t = self._last_rx

    # ------------------------------------------------------------------- api
    @property
    def ok(self) -> bool:
        return self.serial is not None and self.armed

    @property
    def link_alive(self) -> bool:
        return time.monotonic() - self._last_rx < 1.0

    def takeoff(self) -> None:
        self._start = time.monotonic()
        if self.arm_after_s <= 0:
            self.armed = True
            print("serial: ARM (sending stick frames; the flight controller's own arming mode is in charge)")
            return
        print(f"serial: arming in {self.arm_after_s:g} s...")
        end = time.monotonic() + self.arm_after_s
        while time.monotonic() < end:
            self._tick()
            time.sleep(min(0.05, self.period))
        self.armed = True
        if self.takeoff_hold_s > 0:
            print(f"serial: arming now, {self.takeoff_hold_s:g} s of climb to leave the ground")
            end = time.monotonic() + self.takeoff_hold_s
            while time.monotonic() < end:
                self._send(encode_frame(FlightCommand(throttle=1.0), self.scale))
                time.sleep(self.period)

    def send(self, cmd: FlightCommand) -> None:
        self._last_cmd = cmd
        self._tick()

    def _tick(self) -> None:
        """Write at most ``hz`` frames/s, whether the caller knows the rate or not.

        The comparison allows a small epsilon: a control loop that sleeps
        "exactly" one period lands on the boundary, and floating point makes
        that sometimes 49.9999 ms and sometimes 50.0001 ms. Without the epsilon
        the effective rate would silently drop to half whenever the loop is
        perfectly on time.
        """
        now = time.monotonic()
        if now - self._last_tx < self.period - 1e-6:
            return
        self._last_tx = now
        if self.serial is None:
            self._maybe_reconnect()
            return
        payload = bytes([FRAME_HEADER, 0, 0, 0]) if (self.zero_frame and not self.armed) else encode_frame(self._last_cmd, self.scale)
        self._send(payload)

    def land(self) -> None:
        """Hover, then stop: the last bytes on the wire are always a zero frame.

        The zero frames bypass the 20 Hz limiter. ``land`` runs once, on the way
        out, usually with the control loop already stopped - waiting for a rate
        window there would just add delay to the one command that matters.
        """
        for _ in range(3):
            self._send(encode_frame(FlightCommand(), self.scale))
            time.sleep(0.02)
        self.armed = False
        for _ in range(5):
            self._send(bytes([FRAME_HEADER, 0, 0, 0]))
            time.sleep(0.02)
        print(f"serial: disarmed, sent {self.frames_sent} frames ({self.bytes_sent} bytes)"
              + (f", {self.link_errors} link errors" if self.link_errors else ""))

    def emergency_stop(self) -> None:
        self.zero_frame = True
        self.armed = False
        for _ in range(3):
            self._send(bytes([FRAME_HEADER, 0, 0, 0]))
            time.sleep(0.01)

    def telemetry(self) -> Telemetry:
        self._tel.flying = self.armed
        if not self._want_telemetry:
            # No position/altitude feedback: the safety governor stays silent
            # (its altitude rules and geofence only act on known values).
            self._tel.link_ok = self.serial is not None
            return self._tel
        if self.received:
            self._tel.alt_m = None  # the 4-byte protocol carries no altitude
        return self._tel

    def close(self) -> None:
        self._stop.set()
        if self._reader is not None:
            self._reader.join(timeout=0.5)
            self._reader = None
        self._drop()
