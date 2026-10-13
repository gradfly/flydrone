# Safety

FlyDrones puts a research model in charge of a machine with spinning blades. Treat it like any
experimental autopilot.

## Layers

1. **Dry run by default.** Hardware commands are only sent with `--send`.
2. **Brain warm-up on the ground.** Resting firing rates are measured before takeoff.
3. **Safety governor** ([`safety.py`](../src/flydrones/safety.py)), independent of the brain:
   - per-axis limits (`max_throttle`, `max_yaw`, `max_forward`, `max_lateral`)
   - slew-rate limit (commands can drop to zero instantly, but ramp up slowly)
   - ceiling: climbing fades out 30 cm below `max_alt_m` and reverses above it
   - floor: no descent command below `min_alt_m` while flying
   - geofence on drones that report position
   - brain watchdog: hover if the brain output is older than `brain_timeout_s`
   - land on low battery or after `max_flight_s`
4. **Ctrl+C / `q` lands** and closes the connection.
5. **Drone-side failsafes** you must configure: Tello and Crazyflie land on link loss; ArduPilot/PX4
   need GCS/RC failsafe set; Betaflight needs RX-loss failsafe (the ESP32 bridge deliberately goes silent).

## The USB serial link (`--drone serial`)

This backend is different from the others: **there is no feedback at all**. Four bytes go out, nothing
normally comes back, and the laptop has no idea whether the drone moved. The brain closes its loop on
the camera, not on the drone. That has consequences:

1. **The flight controller owns the failsafes.** The 4-byte protocol has no heartbeat, no checksum and no
   "am I still connected" bit, so the *receiver* must implement RX-loss: no valid frame for ~300 ms →
   level off and cut throttle or land. **Set this up and test it (pull the cable in the air, over a net)
   before you rely on the brain's physics.** If your FC cannot do it, do not fly this backend.
2. **A latched last command is the failure mode.** Pulling the USB cable when the FC has no RX-loss
   failsafe leaves the drone holding the last throttle value. The Python side cannot help once the link
   is gone.
3. **The safety governor runs blind.** The protocol carries no altitude and no battery, so the ceiling,
   floor, geofence and battery rules in `safety.py` are inactive. What still applies: per-axis limits
   (`max_throttle` etc.), the slew-rate limit, and the brain watchdog. Start with
   `safety.max_throttle: 0.3` from [`configs/serial_usb.yaml`](../configs/serial_usb.yaml).
4. **`scale` is a trust boundary.** `--scale 120` is the default because it is what this project's
   receiver expects. If you raise it (e.g. `--scale 1000`), make sure your receiver agrees, or ±800 will
   be decoded as a small negative number.
5. **Byte loss is silent.** A dropped byte shifts the whole stream by one, and the receiver will only
   notice when a value byte falls outside the legal range. Short, well-shielded cables; keep them away
   from the ESC wiring and the video transmitter.
6. **The port is not a dead-man's switch.** `land()` writes three neutral frames then five zero frames
   and disarms the Python object. It does *not* disarm the flight controller. Only the FC's own arming
   switch does that.
7. **For `serial`, `--send` does not gate the wire.** With or without it, the brain writes real 20 Hz
   frames - the bytes are identical; the `takeoff()` call only flips a Python-side flag. What actually
   keeps the vehicle on the ground is the **flight controller's own arming** (switch / arm sequence),
   which the brain never touches in any mode. Treat every run with the FC connected as live: props off
   until you mean it.

## Checklist before the first real flight

- [ ] `pytest -q` passes, `flydrones demo` behaves in the simulator
- [ ] same config file flown in `--drone sim` first
- [ ] dry run on the real drone shows sensible commands for each gesture
- [ ] props off for the first bench test; motors respond in the right direction
- [ ] **`serial`: the receiver's RX-loss failsafe was tested by unplugging the cable**
- [ ] **`serial`: `tools/uart_loopback_sim.py` decoded exactly the frames the dashboard showed**
- [ ] **`serial`: `flydrones camera --probe` shows your drone's view, not the room**
- [ ] prop guards on, battery charged, 3 × 3 m clear space, nobody in front
- [ ] vendor app / RC transmitter / second laptop ready to land or disarm
- [ ] start with `safety.max_throttle: 0.3` and `--seconds 30`

## Do not

- fly over people, animals, roads, or outdoors where regulations forbid it
- fly FPV quads with this code without a physical kill switch
- fly the `serial` backend with a flight controller that has no RX-loss failsafe
- present a simulated result as a real flight (label your videos)
