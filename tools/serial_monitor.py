"""Watch a serial link byte by byte: what is really on the wire, and when.

    python tools/serial_monitor.py COM5 --baud 115200 --decode
    python tools/serial_monitor.py COM5 --decode --duration 30 --log frames.csv

Two readers:

* ``--decode``  parse the 4-byte ``AA THR YAW FWD`` frames and print the values
  as they arrive, with a periodic summary (frames/s, value range, monotonic
  sequence check). Use it against ``flydrones fly --drone serial`` to confirm
  the ground truth of what the brain sent.
* default      a hex/ASCII dump of the raw stream, for debugging a receiver
  that does not understand the frames at all (wiring, baud rate, inverted
  signal, a receiver that re-transmits something of its own).

Reading is passive: this never writes to the port and never arms anything.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from flydrones.drones.serial_link import _from_int8  # noqa: E402


def hexdump(data: bytes, base: int, width: int = 16) -> str:
    out = []
    for i in range(0, len(data), width):
        chunk = data[i : i + width]
        hexs = " ".join(f"{b:02x}" for b in chunk).ljust(width * 3 - 1)
        ascii_ = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        out.append(f"{base + i:08x}  {hexs}  |{ascii_}|")
    return "\n".join(out)


def main() -> int:
    p = argparse.ArgumentParser(description="Read a serial link and show what is actually on it.")
    p.add_argument("port", help="COM5 (Windows) or /dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--decode", action="store_true", help="decode 4-byte FlyDrones frames instead of dumping hex")
    p.add_argument("--scale", type=int, default=120, help="value range of the sender, used to sanity-check bytes")
    p.add_argument("--duration", type=float, default=0, help="stop after N seconds (0 = until Ctrl+C)")
    p.add_argument("--log", help="append decoded frames to a CSV")
    p.add_argument("--summary-s", type=float, default=2.0, help="how often to print the summary line")
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
    ser.timeout = 0.05
    try:
        ser.open()
    except Exception as exc:
        print(f"cannot open {args.port}: {exc}")
        return 1
    print(f"listening on {args.port} @ {args.baud} baud"
          + (" (decoding AA THR YAW FWD frames)" if args.decode else " (hex dump)") + "\n")

    log = open(args.log, "w", encoding="utf-8") if args.log else None
    if log:
        log.write("t,throttle,yaw,forward,raw\n")

    buf = bytearray()
    t0 = time.monotonic()
    total = 0
    nframes = 0
    last_summary = t0
    last_summary_n = 0
    best = {"throttle": (0, 0), "yaw": (0, 0), "forward": (0, 0)}
    ever_aligned = False
    try:
        while args.duration <= 0 or time.monotonic() - t0 < args.duration:
            data = ser.read(4096)
            if not data:
                if args.decode and time.monotonic() - last_summary > args.summary_s:
                    rate = (nframes - last_summary_n) / max(1e-6, time.monotonic() - last_summary)
                    print(f"[{time.monotonic() - t0:7.1f}s]  {rate:5.1f} frames/s   "
                          f"range thr {best['throttle'][0]:+4d}..{best['throttle'][1]:+4d}  "
                          f"yaw {best['yaw'][0]:+4d}..{best['yaw'][1]:+4d}  "
                          f"fwd {best['forward'][0]:+4d}..{best['forward'][1]:+4d}"
                          + ("" if ever_aligned else "   NO VALID HEADER YET"))
                    sys.stdout.flush()
                    last_summary = time.monotonic()
                    last_summary_n = nframes
                continue
            total += len(data)
            if not args.decode:
                print(hexdump(data, total - len(data)))
                sys.stdout.flush()
                continue
            buf.extend(data)
            while len(buf) >= 4:
                if buf[0] != 0xAA:
                    del buf[0]
                    continue
                vals = [_from_int8(b) for b in buf[1:4]]
                if any(abs(v) > min(127, args.scale) for v in vals):
                    del buf[0]  # a false header, keep resyncing
                    continue
                ever_aligned = True
                nframes += 1
                del buf[:4]
                thr, yaw, fwd = vals
                for k, v in (("throttle", thr), ("yaw", yaw), ("forward", fwd)):
                    lo, hi = best[k]
                    best[k] = (min(lo, v), max(hi, v))
                t = time.monotonic() - t0
                if log:
                    log.write(f"{t:.4f},{thr},{yaw},{fwd},{buf.hex()}\n")
                if args.decode and not log:
                    print(f"\r[{t:7.2f}s]  {nframes:6d}  AA {thr:+4d} {yaw:+4d} {fwd:+4d}   ", end="")
                    sys.stdout.flush()
    except KeyboardInterrupt:
        print()
    finally:
        ser.close()
        if log:
            log.close()
        dt = time.monotonic() - t0
        print(f"\n\n{total} bytes, {nframes} valid frames in {dt:.1f} s")
        if args.decode:
            print(f"expected ~{args.baud / 10:.0f} bytes/s max at {args.baud} baud; "
                  f"a FlyDrones link uses 4 bytes x 20 Hz = 80 B/s")
        if not args.decode and total and not ever_aligned:
            print("no header seen: check baud rate, GND, and whether the sender writes to this port at all")
        if args.log:
            print(f"log -> {args.log}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
