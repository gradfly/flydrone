"""Ground receiver simulator: the other end of the USB serial link, on your laptop.

It opens a serial port and behaves like a flight-controller receiver that speaks
the FlyDrones 4-byte protocol, so the whole chain (brain -> serial -> receiver)
can be tested end to end before any hardware is armed.

  1. Make a virtual null-modem pair (this is the "USB cable"):
     * Windows: install com0com, then use the two ports it creates (e.g. COM10/COM11)
     * Linux/macOS: ``socat -d -d pty,raw,echo=0 pty,raw,echo=0`` and use the two /dev/pts/N
  2. Terminal 1 - the "flight controller":
     ``python tools/uart_loopback_sim.py --port COM11``
  3. Terminal 2 - the brain, writing the real frames into the pair:
     ``flydrones fly --drone serial --port COM10 --camera 1 --config configs/serial_usb.yaml``

The receiver enforces the only failsafe that really matters here: **if no valid
frame arrives for ``--timeout`` ms it goes to zero**. It also prints the values
and their rate, so you can compare them with the dashboard.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from flydrones.drones.serial_link import _from_int8  # noqa: E402


class Receiver:
    """The protocol decoding, independent of where the bytes come from.

    Split out from the serial I/O so it can be unit-tested (and reused if your
    receiver is a socket or a file rather than a port).
    """

    def __init__(self, scale: int = 120, timeout_ms: float = 300.0):
        self.scale = int(scale)
        self.timeout_s = timeout_ms / 1000.0
        self.buf = bytearray()
        self.command = (0, 0, 0)
        self.frames = 0
        self.dropped = 0
        self.failsafe = False
        self.last_frame_t = 0.0

    def feed(self, data: bytes, now: float) -> list[tuple[int, int, int]]:
        """Consume bytes, return every complete command found in them."""
        limit = min(127, self.scale)
        self.buf.extend(data)
        out: list[tuple[int, int, int]] = []
        while len(self.buf) >= 4:
            if self.buf[0] != 0xAA:
                del self.buf[0]
                self.dropped += 1
                continue
            vals = [_from_int8(b) for b in self.buf[1:4]]
            if any(abs(v) > limit for v in vals):
                del self.buf[0]  # false header, keep resyncing
                self.dropped += 1
                continue
            del self.buf[:4]
            self.command = (vals[0], vals[1], vals[2])
            self.frames += 1
            self.last_frame_t = now
            self.failsafe = False
            out.append(self.command)
        return out

    def poll_failsafe(self, now: float) -> bool:
        """True the first time the link is considered lost."""
        if self.frames and not self.failsafe and now - self.last_frame_t > self.timeout_s:
            self.failsafe = True
            self.command = (0, 0, 0)
            return True
        return False


def main() -> int:
    p = argparse.ArgumentParser(description="Emulate the flight-controller side of the FlyDrones serial link.")
    p.add_argument("--port", required=True, help="COM11 (Windows) or /dev/pts/3")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--scale", type=int, default=120)
    p.add_argument("--timeout", type=float, default=300.0, help="ms without a frame before going to zero (RC-loss failsafe)")
    p.add_argument("--duration", type=float, default=0, help="stop after N seconds (0 = until Ctrl+C)")
    p.add_argument("--quiet", action="store_true", help="only print the summary and any failsafe event")
    args = p.parse_args()

    try:
        import serial
    except ImportError:
        print("pyserial missing: pip install 'flydrones[serial]'")
        return 2

    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = args.baud
    ser.dtr = False
    ser.rts = False
    ser.timeout = 0.02
    try:
        ser.open()
    except Exception as exc:
        print(f"cannot open {args.port}: {exc}")
        return 1

    rx = Receiver(args.scale, args.timeout)
    print(f"receiver emulator on {args.port} @ {args.baud}, scale +-{args.scale}")
    print(f"failsafe: no valid frame for {args.timeout:g} ms -> all axes 0\n")

    t0 = time.monotonic()
    t_end = t0 + args.duration if args.duration > 0 else None
    last_print = 0.0
    last_n = 0
    try:
        while t_end is None or time.monotonic() < t_end:
            data = ser.read(4096)
            now = time.monotonic()
            if data:
                rx.feed(data, now)
            if rx.poll_failsafe(now):
                print("FAILSAFE: no frame for "
                      f"{args.timeout:g} ms -> sticks 0,0,0")
            if not args.quiet and now - last_print > 0.1:
                rate = (rx.frames - last_n) / max(1e-6, now - last_print)
                print(f"\r[{now - t0:7.2f}s]  {rx.frames:6d} frames ({rate:4.1f}/s)  "
                      f"AA {rx.command[0]:+4d} {rx.command[1]:+4d} {rx.command[2]:+4d}"
                      + ("  FAILSAFE" if rx.failsafe else "          "), end="")
                sys.stdout.flush()
                last_print = now
                last_n = rx.frames
            time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        dt = time.monotonic() - t0
        print(f"\n\nreceived {rx.frames} frames in {dt:.1f} s ({rx.frames / max(dt, 1e-6):.1f}/s), "
              f"{rx.dropped} noise bytes discarded")
        if rx.failsafe:
            print("ended in FAILSAFE")
        if rx.frames == 0:
            print("no valid frame: wrong port/baud, wrong pair, or the sender is not writing")
        return 0 if rx.frames else 1


if __name__ == "__main__":
    raise SystemExit(main())
