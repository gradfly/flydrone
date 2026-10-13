"""Measure the serial wire protocol: send 4-byte frames, count bytes/s and any jitter.

This is the ground-station companion to ``--drone serial``. Two uses:

1. **With the FC's app running** (Betaflight Configurator / your own RX sketch):
   watch the receiver's own display while this runs, and confirm it decodes the
   same throttle / yaw / forward you see here.
2. **With ``serial_monitor.py --listen``** on the other side of the link:
   a byte-level ground truth of exactly what went down the wire.

    python tools/uart_stream_test.py --port COM5 --baud 115200 --seconds 10
    python tools/uart_stream_test.py --port COM5 --soak                 # 200 s soak test
    python tools/uart_stream_test.py --port COM5 --auto                 # hands write the frame

It always ends with an all-zero frame and never arms anything: this is a link
test, not a flight. For a real flight use ``flydrones fly --drone serial --send``.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from flydrones.drones.serial_link import encode_frame  # noqa: E402
from flydrones.motor import FlightCommand  # noqa: E402

PATTERNS = {
    "sweep": None,      # filled in below: a walk through the value range
    "hover": FlightCommand(),
    "centre": FlightCommand(),
}


def sample(t: float, pattern: str) -> FlightCommand:
    """A deterministic command value at time ``t`` (so the receiver can check it)."""
    if pattern == "hover" or pattern == "centre":
        return FlightCommand()
    if pattern == "climb":
        return FlightCommand(throttle=1.0)
    if pattern == "yaw-right":
        return FlightCommand(yaw=1.0)
    if pattern == "forward":
        return FlightCommand(forward=1.0)
    if pattern == "square":
        # 2 s per side, all three axes, exercises the largest values
        phase = int(t * 2) % 4
        return [FlightCommand(), FlightCommand(throttle=1.0), FlightCommand(yaw=1.0),
                FlightCommand(forward=1.0)][phase]
    # "sweep": a slow ramp through the whole -120..120 range
    v = math.sin(2 * math.pi * t / 4.0)
    return FlightCommand(throttle=v, yaw=v, forward=v)


def main() -> int:
    p = argparse.ArgumentParser(description="Send the FlyDrones 4-byte serial frames and report what happened.")
    p.add_argument("--port", required=True, help="COM5 (Windows) or /dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--hz", type=float, default=20.0)
    p.add_argument("--scale", type=int, default=120)
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--soak", action="store_true", help="run a 200 s soak test instead of --seconds")
    p.add_argument("--pattern", default="square",
                   choices=["sweep", "hover", "centre", "square", "climb", "yaw-right", "forward"])
    p.add_argument("--auto", action="store_true", help="ask for a pattern by hand on stdin (q to stop)")
    p.add_argument("--log", help="write every sent frame as CSV (t,throttle,yaw,forward,bytes)")
    args = p.parse_args()

    try:
        import serial
    except ImportError:
        print("pyserial missing: pip install 'flydrones[serial]'")
        return 2

    period = 1.0 / args.hz
    seconds = 200.0 if args.soak else args.seconds

    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = args.baud
    ser.dtr = False
    ser.rts = False
    ser.timeout = 0
    try:
        ser.open()
    except Exception as exc:
        print(f"cannot open {args.port}: {exc}")
        print("check the port name (Windows: Device Manager > Ports), that no other app holds it,")
        print("and that the driver is installed (CP210x / CH340 / FTDI).")
        return 1
    print(f"opened {args.port} @ {args.baud} - sending {args.hz:g} Hz, pattern '{args.pattern}', scale +-{args.scale}")
    print("frame = AA <throttle> <yaw> <forward>, signed bytes. Watch the receiver's own read-out.\n")

    log = open(args.log, "w", encoding="utf-8") if args.log else None
    if log:
        log.write("t,throttle,yaw,forward,bytes\n")

    counts: Counter = Counter()
    gaps: list[float] = []
    t0 = time.monotonic()
    last = t0
    n = 0
    pattern = args.pattern
    next_auto = 0.0
    try:
        while True:
            now = time.monotonic()
            elapsed = now - t0
            if elapsed >= seconds:
                break
            if args.auto and now >= next_auto:
                print("pattern [sweep/hover/square/climb/yaw-right/forward, q=quit]: ", end="", flush=True)
                try:
                    line = sys.stdin.readline().strip()
                except (EOFError, KeyboardInterrupt):
                    break
                if line.lower() in ("q", "quit", ""):
                    break
                if line in ("sweep", "hover", "centre", "square", "climb", "yaw-right", "forward"):
                    pattern = line
                next_auto = time.monotonic() + 0.1

            cmd = sample(elapsed, pattern)
            frame = encode_frame(cmd, args.scale)
            try:
                ser.write(frame)
            except Exception as exc:
                print(f"write failed at t={elapsed:.1f}s: {exc}")
                break
            counts[frame[1:].hex()] += 1
            n += 1
            if log:
                log.write(f"{elapsed:.4f},{frame[1]},{frame[2]},{frame[3]},{frame.hex()}\n")
            gaps.append(time.monotonic() - last)
            last = time.monotonic()
            sleep = period - (last - now)
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        print("\nstopped by Ctrl+C")
    finally:
        # Never leave the receiver holding the last command.
        for _ in range(5):
            ser.write(bytes([0xAA, 0, 0, 0]))
            time.sleep(period)
        ser.close()
        if log:
            log.close()

    wall = time.monotonic() - t0
    print(f"\nsent {n} frames in {wall:.1f} s -> {n / wall:.1f} frames/s ({4 * n} bytes, ~{4 * n / wall / 1024:.2f} KB/s)")
    print(f"distinct payloads: {len(counts)}")
    if gaps:
        s = sorted(gaps)
        print(f"inter-frame period: mean {sum(gaps) / len(gaps) * 1000:.2f} ms, "
              f"p95 {s[int(len(s) * 0.95)] * 1000:.2f} ms, max {s[-1] * 1000:.2f} ms")
        if s[-1] > 5 * period:
            print(f"WARNING: worst gap is {s[-1] * 1000:.0f} ms, the receiver's own failsafe needs to cover that")
    print("final frame sent: AA 00 00 00 (all zero)")
    if args.log:
        print(f"log -> {args.log}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
