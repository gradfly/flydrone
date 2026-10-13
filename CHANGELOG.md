# Changelog

## Unreleased
**Escape becomes a 180° about-face (demo v22 sync).** A head-on loom (both halves of the eye swelling
alike) now triggers a **180° about-face** instead of a climb: full yaw one way, zero forward, zero
throttle, held for `turn180_hold_s` (derived as `≈ π / yaw_rate_dps + 0.35 s`, so it tracks the
airframe's yaw rate). Climbing on every head-on read turned the flight into a yo-yo and gained altitude
the governor then had to undo; an about-face is a pure dodge with no altitude side effect. `climb`
survives only in the no-cue random fallback (`pick_p: {climb: 0.10, left: 0.45}`). `side_close_frac`
0.20 → 0.10, `climb_hold_s` 1.5 → 1.2.

Two new vision/decoder knobs came with it:

- `vision.loom_fov_frac` (default 1.0, tuned to 0.3 for autoplay) — the looming *trigger* reads only the
  central fraction of the frame. Peripheral flow (walls sliding past during cruise) inflated the
  full-width expansion slope and fired the reflex too early. The escape *direction* still reads the full
  half-eye, so the drone still knows which way to turn.
- `decoder.escape.v_ref` (0.8 m/s) — the looming signal ∝ V/Z, so the threshold is now scaled by
  `clamp(speed / v_ref, 0.3, 3.0)`. Faster flight triggers later (closer), slower flight earlier; the
  clamp keeps a standstill from firing on noise. Requires a new `Telemetry.speed` field.

Also synced from the demo: **OBB collision** (`SimDrone` now projects body half-extents
`0.15 × 0.21 × 0.05 m` onto the world axes by yaw, instead of a uniform `r = 0.1` sphere, so a head-on
wall hit stops the nose and a side-on hit stops the motor tips; boxes push out along the shallowest axis
instead of reverting the step), and the **seeded foam-block arena** (`OBSTACLE_COUNT` 6 → 8, tighter mat
spacing, `Room.arena(seed)` / `make_obstacles()` using the demo's `mulberry32`, so a seed reproduces the
same layout in both simulators — verified bit-exact). `docs/live/` is refreshed to v22.

**Deterministic evasion in Python.** The browser demo's v11-v17 escape logic is ported back, so the
desktop brain dodges the same way the demo does: the escape direction is read off the retina (which half
of the eye is swelling harder) instead of being rolled at random, a head-on read becomes a climb only
`climb_p` of the time, a climb keeps flying forward for `climb_hold_s` (a dodge, not a stall), contact
with a surface is a second non-visual trigger plus a physical `SimDrone.push_off`, and the looming brake
can no longer zero the cruise (`escape.cruise_floor`). New `motor/escape.py`, `Telemetry.touching`,
16 new tests. All new knobs live under `decoder.escape` in `defaults.yaml`.

**USB camera in, USB serial out.** A new `serial` drone backend for the "drone camera on a USB UVC
capture device, flight controller on a USB serial port" setup: 640×480 capture card as the fly's eyes,
`throttle`/`yaw`/`forward` scaled to ±120 and sent as 4 bytes (`0xAA` header + three signed bytes) at
20 Hz. Includes a rate-limited, self-reconnecting link that can never stall the control loop, a
`flydrones camera` probe command, `configs/serial_usb.yaml`, an end-to-end loopback ground receiver
(`tools/uart_loopback_sim.py`), a link tester and a passive monitor, and 19 protocol tests.
`--drone serial` is the one backend that writes real frames without `--send` (SERIAL TEST MODE), and its
default input mode is **autoplay**: the drone's own camera is the only eye — no gesture illusions — with a
constant `decoder.cruise` forward drive braked by looming and a giant-fiber escape on the real feed
(`configs/serial_usb.yaml` ships tuned for it). `--input camera|gesture|both` remain for the hand demo.

## 0.1.2 - 2026-09-16
Live demo 2.0: detailed blue quadcopter with a fly mascot, furnished bedroom with walls, day/night themes, shadows,
swat-the-drone game driven by the looming pathway, click-to-stimulate neurons, clickable 3D objects, camera modes,
particles and sound.

## 0.1.1 - 2026-09-15
Browser demo on GitHub Pages (three.js, JS port of the engine, webcam hands), hero GIF, social preview image,
CI check that the browser engine matches Python.

## 0.1.0 - 2026-09-15
First public release: simulator, MaleCNS loader, MiniFly, retina, gestures, decoder, safety governor,
Tello / Crazyflie / MAVLink / ESP32 backends, dashboard, swarm, calibration, documentation.
