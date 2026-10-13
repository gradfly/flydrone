# Hardware

FlyDrones talks to every drone through one small interface ([`drones/base.py`](../src/flydrones/drones/base.py)):
`takeoff()`, `send(FlightCommand)`, `telemetry()`, `frame()`, `land()`. A `FlightCommand` has
`throttle` (vertical speed), `yaw` (yaw rate), `forward`, `lateral`, each in −1..1. The drone's own
flight controller turns that into motor speeds and keeps the airframe level.

> Status: the simulator path and the serial protocol are tested end to end (45 tests). The Tello, Crazyflie,
> MAVLink and ESP32 adapters follow the official SDKs but have **not been flight-tested by the authors yet**.
> Please open an issue with logs (`--log flight.csv`) when you try one.

## DJI / Ryze Tello (recommended first drone)

- **Why:** built-in altitude hold and optical-flow positioning, Wi-Fi video, prop guards, 80 g.
- **Install:** `pip install -e ".[tello,gestures]"`
- **Connect:** join the `TELLO-XXXXXX` Wi-Fi network from your laptop.
- **Run:** `flydrones fly --drone tello --config configs/tello.yaml --input both --live --send`
- **Mapping:** `send_rc_control(lateral, forward, throttle, yaw)` scaled to ±60 % stick by default.
- **Eyes:** the Tello video feed goes into the retina (optic flow and looming). Your hand in front of the
  laptop webcam adds illusions. `--input camera` uses only the drone camera, `--input gesture` only the hand.
- **Kill switch:** Ctrl+C lands. `TelloDrone.emergency_stop()` cuts motors (the drone falls).
- **Tip:** Tello needs light and a textured floor to hold position. Video over Wi-Fi lags 100-200 ms,
  which slows the looming reflex.

## Bitcraze Crazyflie 2.1 (+ Flow deck v2)

- **Why:** 27 g, open firmware, safest indoor platform.
- **Install:** `pip install -e ".[crazyflie,gestures]"` and a Crazyradio PA/2.0 dongle.
- **Run:** `flydrones fly --drone crazyflie --uri radio://0/80/2M/E7E7E7E7E7 --config configs/crazyflie.yaml --input gesture --send`
- **Mapping:** hover setpoints `(vx, vy, yaw_rate, z)`. The brain's throttle is integrated into a height
  target, so the Crazyflie's estimator holds altitude between decisions.
- **Eyes:** no video camera. Use the webcam hand (or add an AI deck and write a `frame()` method).
- **Arming:** newer firmware needs an arming request, the adapter sends it.

## ArduPilot / PX4 (MAVLink)

- **Start in SITL.** ArduPilot: `sim_vehicle.py -v ArduCopter --console --map`. PX4: `make px4_sitl gz_x500`.
- **Install:** `pip install -e ".[mavlink]"`
- **Run:** `flydrones fly --drone mavlink --mavlink udpin:0.0.0.0:14550 --autopilot ardupilot --config configs/mavlink_sitl.yaml --input gesture --send`
- **Serial telemetry radio:** `--mavlink COM5,57600` (Windows) or `/dev/ttyUSB0,57600`.
- **Mapping:** `SET_POSITION_TARGET_LOCAL_NED` in body frame, velocity + yaw rate
  (`type_mask = 1479`). ArduPilot uses GUIDED, PX4 uses OFFBOARD (a setpoint stream is sent before switching).
- **Telemetry used:** `LOCAL_POSITION_NED` (altitude, geofence), `ATTITUDE` (yaw rate → halteres), `SYS_STATUS` (battery).
- **Outdoors only**, with a real RC transmitter able to switch to LOITER/LAND at any time.

## USB camera in, USB serial out (your own flight controller)

The simplest "real drone" path, and the one closest to the fly: **the drone's own camera becomes the fly's
eyes**, and the brain sends three stick values down a serial cable to whatever flight controller is on the
other end.

```
drone camera ──analog 5.8 GHz / digital downlink──► USB UVC capture ─┐
                                                                      ├─ laptop: fly brain
flight controller ◄──────── USB serial (UART), 4 bytes @ 20 Hz ──────┘
```

The laptop does not fly the drone. The flight controller still does that (self-levelling, altitude hold,
arming, its own failsafe). The fly brain is a **pilot with a remote**, sending throttle / yaw / forward
20 times a second.

### Install

```bash
pip install -e ".[serial,vision]"        # pyserial + OpenCV
```

### 1. Find the camera

```bash
flydrones camera                        # probe indices, print the modes each one offers
flydrones camera --probe 1              # measure the frames/s actually delivered + save a still
```

`--probe` writes `camera_probe.png`. **Look at it**: if it is your bedroom instead of the drone's view,
the index is wrong and the brain would react to your furniture. Press play on the downlink first; most
capture cards deliver nothing until a video signal is present.

If two devices share a name on Windows, pass the DirectShow name instead of an index:
`--camera "USB Video"`.

### 2. Bench test with props off

The serial backend is the one that runs **without** `--send`: it really writes the frames (identical
bytes to `--send`) so you can verify a receiver. The brain never arms the flight controller in either
mode - the FC's own arming switch is the only gate, so **props off whenever the FC is connected**.

```bash
flydrones fly --drone serial --camera 1 --port COM5 --config configs/serial_usb.yaml --live
```

**The serial path flies itself: `--input autoplay` is the default.** The drone's own camera is the only
eye — no hand gestures, no laptop webcam. The brain holds a constant forward drive (`decoder.cruise`,
0.35 in this config) and the real optic flow closes the loop: sinking makes the scene drift up in view,
the climb reflex answers, and looming on the real feed fires the giant-fiber escape. On the bench expect:

- at rest: `forward` ≈ **+42** on the wire (cruise 0.35 × scale 120), `throttle` / `yaw` near 0
- pan or tilt the camera: `yaw` answers (optomotor reflex), up/down drift moves `throttle`
- push the camera straight at a wall: a ~0.7 s escape burst — one axis jumps to ±0.8, `forward` drops to 0

### How it dodges (ported from the browser demo)

The escape direction is a **lookup off the retina, not a dice roll**:

| what the eye sees | what the fly does |
|---|---|
| left half of the view swelling harder | turn right (away from it) |
| right half swelling harder | turn left (away from it) |
| both halves alike (coming straight at it) | head-on: a **180° about-face** (`turn180_hold_s`) |
| nothing moving at all (a bump, or a dead camera) | random fallback: `pick_p` mix (climb 10 % / left 45 % / right 45 %) |

The measurement is the **texture-weighted mean flow divergence per half of the eye**. Local expansion is
`2·Vz/Z`, so it is large exactly where a surface is close; averaging it over a half answers "which half
of my world is about to hit me". The floor and far walls sit in *both* halves, so they lift both means
together and cancel — only a genuinely near object tilts the balance. Two tempting alternatives do **not**
work: the per-half affine slope matches the true obstacle side only ~50-60 % of the time, and a per-half
*maximum* divergence always returns the floor, so every loom reads as head-on.

A head-on loom is a **180° about-face**, not a climb: full yaw one way, zero forward, zero throttle, held
for `turn180_hold_s` seconds, then the cruise resumes pointing the other way. The hold is derived from the
airframe's yaw rate (`hold ≈ π / yaw_rate_dps + tau`, tau = 0.35 s of yaw lag), so it stays correct if you
change the yaw rate. It replaced the old climb-on-head-on, which turned the flight into a yo-yo and gained
altitude the governor then had to undo — a 180° turn is a pure dodge with no altitude side effect.

`climb` survives only in the no-cue fallback (a bump, or a dead camera). A fallback climb is not a stall:
it takes over only the vertical channel and **keeps flying forward** for `climb_hold_s` (1.2 s), and stops
gaining height above `escape.ceil_alt` (1.6 m here).

#### Only the centre of the field triggers the reflex

The looming *trigger* reads only the central `vision.loom_fov_frac` of the frame (default 0.3 here;
`minifly.json` leaves it at 1.0 for the desk demo). Peripheral flow — walls sliding past during a cruise —
inflates the full-width expansion slope and fires the reflex too early. Reading only the centre cuts those
false triggers. The escape *direction* still reads the full half-eye (`near_l` / `near_r`), so the drone
still knows which way to turn once it has triggered.

#### The threshold scales with speed

The looming signal is proportional to `V/Z`, so a fixed threshold triggers at very different distances
depending on speed. The effective threshold is

```
threshold_eff = threshold_hz × clamp(speed / v_ref, 0.3, 3.0)
```

with `escape.v_ref` (0.8 m/s, ≈ the cruise speed). Fly faster and the reflex waits until closer; fly
slower and it fires earlier. The clamp keeps a standstill from firing on noise and a burst from making the
reflex unreachable. Drones that do not report a speed fall back to the base threshold.

Three more things make it robust:

- **a bump is a second trigger.** A drone pressed against a surface sees a *static* image, so there is no
  expansion and the looming reflex can never fire. `Telemetry.touching` fires the same escape instead,
  rate-limited by `contact_s` (0.9 s) so sliding along a wall does not become non-stop twitching.
- **a contact escape also shoves the airframe off the surface** (`SimDrone.push_off`). A turn alone leaves
  it pressed there: the brain keeps commanding forward and the collision solver cancels exactly that.
- **the cruise is never braked to zero** (`escape.cruise_floor`, 0.4 here). The looming brake's weighted
  sum can exceed 1, and a plain `max(0, 1-brake)` then parks the drone in mid-air for a second. A fly
  slows down; it does not stop dead.

#### The turn direction is a measured sign, not a guess

The direction rule is one line — and getting it backwards is invisible: the reflex still fires on every
loom, every unit test above still passes, and only the flight gets *worse*. This actually happened during
the port, and the fix turned out to be a single sign. Re-running the 6-seed / 45 s soak on the **same
seeds** after the fix (both variants flown through the real `Pilot`):

| variant | collisions (6 seeds) | escapes | reads with no visual cue |
|---|---|---|---|
| random escape (the old Python behaviour) | **399** `[150, 53, 72, 13, 11, 100]` | 1692 | 116 |
| the ported eye, **correct sign** | **231** `[32, 52, 13, 70, 11, 53]` | 1254 | 29 |
| the ported eye, sign inverted | worse than random (this was the bug) | — | — |

Two things are worth knowing if you ever touch that line:

- **The sign is pinned by a ground-truth test that refuses to be fooled**
  (`tests/test_escape.py::test_direction_points_away_from_the_real_obstacle`). It plants a real box in the
  room, reads the real retina, and asserts the *invariant that matters*: whenever the rule commits to a
  turn, the turn goes **away** from the box. Correct sign scores 3/3 away and 0 into; the inverted sign
  scores 0/3 away and 3 into. Flip the sign and that test fails loudly with
  `turns went *into* the obstacle - escape direction is inverted`.
- **A "how often does the read agree with the truth" test cannot catch this.** Three earlier versions of
  that test were vacuous: aiming the camera *at* the box makes the box's lateral offset exactly 0 by
  construction (so there is no correct answer and a flipped sign scores 50 %); probing the flow geometry
  rather than the picker tests a sign-agnostic layer (so a flipped sign still scored 83 %); and demanding
  "the swelling half is the obstacle half" is wrong for a large box, where both halves swell and the
  correct read is head-on (`turn180`). The head-on rate is genuine too: the retina locates the obstacle side roughly
  **70–83 %** of the time, matching what the browser demo measured, so some looms legitimately read as
  ambiguous.

Note also how *few* turns the rule takes: in the ground-truth setup it committed to a side only 3 times out
of 6 geometries — the other 3 were read as head-on or as no-cue. The `nocue` column above is the useful
sanity check: it fell from 116 to 29, which proves the eye-read is actually firing instead of quietly
falling through to the dice.

The dashboard shows you exactly which bytes are going out. Confirm your receiver decodes the same
throttle / yaw / forward. Then, and only then:

```bash
flydrones fly --drone serial --camera 1 --port COM5 --config configs/serial_usb.yaml --live --send
```

`--hz 20`, `--scale 120`, `--baud`, `--width/--height/--fps`, `--yuyv` and `--cam-backend` override the
config file. `--port`/`--baud`/`--camera` also fall back to the `serial:` and `camera:` blocks in the YAML.
`--input gesture|both|camera` still exist if you want to hand-fly it with the laptop webcam instead.

### The wire protocol (4 bytes, 20 Hz)

```
        byte 0     byte 1      byte 2     byte 3
      +--------+------------+----------+-----------+
      |  0xAA  |  throttle  |   yaw    |  forward  |
      +--------+------------+----------+-----------+
      frame    int8          int8        int8
      header   -120..+120    -120..+120  -120..+120
```

| axis | −120 | 0 | +120 |
|---|---|---|---|
| `throttle` | full down / disarm-down | hold altitude | full up (climb) |
| `yaw` | yaw left (CCW) | hold heading | yaw right (CW) |
| `forward` | full back | hover | full forward (pitch down) |

Three details that will bite you:

1. **The three values are signed bytes (two's complement).** `−120` is on the wire as `0x88`
   (= `256 − 120`), **not** as `0x78` with a separate sign. At scale 120 every legal value fits in int8,
   so no clamping is needed on the wire. In Python the sender does `int(v) & 0xFF`; the receiver does
   `b - 256 if b > 127 else b`.
2. **`0xAA` is also a legal *value* byte** (`−86` at scale 120). A receiver must therefore decode by
   **position — always exactly 4 bytes after a header** — not by searching for the next `0xAA`. Searching
   for headers will resynchronise onto a value byte and stay wrong. The reference receiver
   ([`tools/uart_loopback_sim.py`](../tools/uart_loopback_sim.py)) shows the correct pattern: read 4 bytes
   at a time, and only reject a header (dropping one byte, resyncing) if a payload byte lies outside
   `−scale..+scale` and therefore *cannot* be a value.
3. **There is no checksum and no altitude.** Four bytes is 4 bytes. Byte loss on a noisy cable is
   therefore indistinguishable from a real command, so the receiver **must** implement the
   no-frame timeout below.

Frame rate: one frame every **50 ms** (20 Hz), = 1600 baud of actual payload — trivially inside 115200.
The sender enforces the rate itself (`serial.hz`), so the control loop can tick faster or slower without
changing what goes on the wire. On exit the last bytes written are always `AA 00 00 00`.

### What the flight controller must do

FlyDrones assumes your FC (or your own receiver sketch) consumes these four bytes and treats them as RC
sticks:

- map `throttle`/`yaw`/`forward` to your stick range (`scale` is configurable: `--scale 1000` gives ±1000)
- **arm** through its own arming switch/sequence - the brain never arms anything
- implement an **RX-loss failsafe**: no valid frame for ~300 ms → level off, cut throttle, or land.
  This is the single most important thing on this list; without it, unplugging the cable leaves the last
  command latched. The Python side cannot do it for you once the link is gone.
- keep angle/level mode on. Nothing here stabilises the airframe.

### 3. Test the whole chain without a drone

You do not need a flight controller to prove the brain→bytes→receiver path works:

```bash
# 1. make a virtual null-modem pair ("the USB cable")
#    Windows: install com0com -> it creates e.g. COM10/COM11
#    Linux/macOS: socat -d -d pty,raw,echo=0 pty,raw,echo=0   # use the two /dev/pts/N

# 2. terminal 1 - the "flight controller"
python tools/uart_loopback_sim.py --port COM11

# 3. terminal 2 - the brain
flydrones fly --drone serial --port COM10 --camera 1 --config configs/serial_usb.yaml
```

Terminal 2 prints the byte count and terminal 1 decodes it, so any disagreement is immediately visible.
`tools/uart_loopback_sim.py` is also the **reference implementation** of the protocol on the receiving
side: copy its `Receiver` class into your own firmware/ground station.

Two more tools:

| tool | what it proves |
|---|---|
| `python tools/uart_stream_test.py --port COM5 --pattern sweep` | the *link*: frame rate, jitter, dropouts, and that your receiver decodes the same values. `--soak` runs 200 s, `--log frames.csv` records every frame |
| `python tools/serial_monitor.py COM5 --decode` | the *bytes*: passive read-only decode (or a hex dump if the receiver is not understanding anything at all) |

All three end with a zero frame and never arm anything.

### Optional: telemetry back from the FC

If your FC sends its own 4-byte frames back (`AA thr yaw fwd`), add `--telemetry` and the link is read in
a background thread. Note the protocol carries **no altitude and no battery**, so the safety governor's
ceiling / floor / geofence / battery rules stay inactive. It enforces per-axis limits, slew rate, and the
brain watchdog only. Fly it like a rate-mode pilot.

### Troubleshooting

| symptom | cause |
|---|---|
| `cannot open COM5` | wrong name, another app (Configurator) holds it, or the driver is missing (CP210x / CH340 / FTDI) |
| opens, `0` frames in `flydrones camera --probe` | no video signal on the capture card, or a second app is using the device |
| probe is 5-10 fps | the card fell back to YUYV; it is MJPG-capable, so leave `mjpg: true` |
| frames arrive but values look like garbage | the receiver is searching for `0xAA` instead of decoding by position (see detail 2) |
| drone twitches on connect | the FC is being reset by DTR/RTS. FlyDrones sets both to `False`; check you are not opening the port elsewhere |
| the last command stays latched after unplugging | the FC has no RX-loss failsafe. Fix that before flying |

Safety checklist and pre-flight items: [SAFETY.md](SAFETY.md).

## Betaflight / INAV quad via ESP32 bridge

For FPV-style quads without a companion computer.

```
laptop (fly brain) ──Wi-Fi UDP──► ESP32 ──UART MSP──► flight controller ──► ESCs
```

- **Parts:** any ESP32 dev board (or M5Stack Atom), 3 wires, a Betaflight/INAV FC with a free UART.
- **Firmware:** open [`firmware/esp32_msp_bridge/esp32_msp_bridge.ino`](../firmware/esp32_msp_bridge/esp32_msp_bridge.ino)
  in Arduino IDE (ESP32 core ≥ 2.0), change `AP_PASS`, flash.
- **Wiring:** ESP32 GPIO17 → FC RX, GPIO16 ← FC TX, GND ↔ GND.
- **Betaflight Configurator:** Ports → MSP on that UART. Receiver → "MSP RX input". Modes → ARM on AUX1,
  ANGLE always on. Failsafe → stage 2 "Land".
- **Laptop:** join `FlyDrones-Bridge` Wi-Fi, then `flydrones fly --drone esp32 --config configs/esp32_betaflight.yaml --input gesture --send`.
- **Protocol:** `FD1,seq,arm,thr,yaw,pitch,roll` at up to 50 Hz, values −1000..1000. The ESP32 stops
  sending RC frames after 300 ms without a packet, so the FC's own RX-loss failsafe takes over.
- **Warning:** Betaflight has no altitude hold. Throttle here is a stick offset around `HOVER_PWM`, which
  you must calibrate. This is the hardest and most dangerous path. Props off until everything is verified,
  then fly in a cage or over a net.

## Your own drone

Subclass `Drone`:

```python
from flydrones.drones.base import Drone
from flydrones.safety import Telemetry

class MyDrone(Drone):
    name = "mine"
    has_camera = False
    def takeoff(self): ...
    def land(self): ...
    def send(self, cmd): ...          # cmd.throttle, cmd.yaw, cmd.forward, cmd.lateral in -1..1
    def telemetry(self): return Telemetry(alt_m=..., yaw_rate_dps=..., battery_pct=..., flying=True)
```

Then run it with `flydrones.runtime.run_realtime(Pilot(brain, MyDrone(), cfg))`.

## Laptop

- Any 4-core CPU from the last few years runs MiniFly and sensorimotor cores in real time.
- The full MaleCNS brain needs 8 GB RAM. Speed depends on the CPU; check with `flydrones bench --brain ...`.
- A webcam for gestures; good, even lighting helps both MediaPipe and the OpenCV fallback.
