"""The USB serial link: 4-byte stick frames, scaling, the send loop and its failsafes.

The receiver side of this protocol is specified in ``tools/uart_loopback_sim.py``
and in ``docs/HARDWARE.md``; these tests are the ground truth the two must agree on.
"""

import time

import pytest

from flydrones.config import load_config
from flydrones.drones.serial_link import (FRAME_HEADER, SerialDrone, SerialPacket, decode_frames, encode_frame,
                                          probe_frame, scale_axis)
from flydrones.motor import FlightCommand


# ------------------------------------------------------------------ scaling
def test_scale_maps_normalised_commands_to_raw_sticks():
    assert scale_axis(1.0) == 120
    assert scale_axis(-1.0) == -120
    assert scale_axis(0.5) == 60
    assert scale_axis(0.0) == 0
    assert scale_axis(2.0) == 2  # already raw units: only clamped to the scale
    assert scale_axis(1.0, scale=100) == 100
    assert scale_axis(-1.0, scale=1000) == -1000


def test_scale_never_emits_a_bad_number():
    # a stalled decoder can hand us NaN / inf; that must become a neutral stick
    assert scale_axis(float("nan")) == 0
    assert scale_axis(float("inf")) == 0
    assert scale_axis(float("-inf")) == 0


# ------------------------------------------------------------------- frames
def test_frame_layout_is_four_bytes_with_the_documented_header():
    f = encode_frame(FlightCommand(throttle=1.0, yaw=-1.0, forward=0.5))
    assert len(f) == 4
    assert f == bytes([0xAA, 120, 0x88, 60])  # 0x88 = -120 as a signed byte


def test_payload_bytes_can_never_say_the_whole_frame_is_garbage():
    # 0xAA is a legal *value* byte at scale 120 (it means -86), so the only thing
    # the parser may trust is the frame length: never reject four bytes because
    # one of them looks like a header. This test pins that behaviour down.
    f = encode_frame(FlightCommand(throttle=-86 / 120.0, yaw=-86 / 120.0, forward=-86 / 120.0))
    assert f == bytes([0xAA, 0xAA, 0xAA, 0xAA])
    assert [(p.throttle, p.yaw, p.forward) for p in decode_frames(bytearray(f))] == [(-86, -86, -86)]


def test_only_the_header_starts_a_frame():
    # 0xAA is also a *legal* value byte at scale 120 (it means -86). A receiver
    # must therefore decide by position, not by looking for bytes that "can't be
    # data" - there are none. The two consequences are pinned down here.
    worst = encode_frame(FlightCommand(throttle=-86 / 120.0, yaw=-86 / 120.0, forward=-86 / 120.0))
    assert worst == bytes([0xAA, 0xAA, 0xAA, 0xAA])
    assert [(p.throttle, p.yaw, p.forward) for p in decode_frames(bytearray(worst))] == [(-86, -86, -86)]

    # ...and a receiver expecting exactly 4 bytes per command never has to guess,
    # because the sender is rate limited to one frame per control tick.
    stream = b"".join(encode_frame(FlightCommand(throttle=v / 10.0, yaw=v / 10.0, forward=v / 10.0)) for v in range(3))
    frames = [(p.throttle, p.yaw, p.forward) for p in decode_frames(bytearray(stream))]
    assert frames == [(0, 0, 0), (12, 12, 12), (24, 24, 24)]


def test_every_value_round_trips():
    for v in range(-120, 121):
        f = encode_frame(FlightCommand(throttle=v / 120.0, yaw=v / 120.0, forward=v / 120.0))
        pkt = decode_frames(bytearray(f))[0]
        assert (pkt.throttle, pkt.yaw, pkt.forward) == (v, v, v)


def test_decoder_resynchronises_on_noise():
    real = encode_frame(FlightCommand(throttle=0.5, yaw=0.25, forward=-0.75))
    # a false header whose first value byte (121) is impossible for a 120-scale link
    noise = bytes([0xFF, 0xAA, 0x79, 0x88, 0x00])
    buf = bytearray(b"\x01\x02" + real + noise + real)
    frames = [(p.throttle, p.yaw, p.forward) for p in decode_frames(buf)]
    assert frames == [(60, 30, -90), (60, 30, -90)]
    assert bytes(buf) == b""


def test_partial_frame_is_kept_for_the_next_read():
    f = encode_frame(FlightCommand(throttle=1.0))
    buf = bytearray(f[:3])
    assert probe_frame(buf) is None
    assert bytes(buf) == f[:3]  # nothing lost, nothing consumed
    buf.extend(f[3:])
    assert probe_frame(buf).throttle == 120


def test_packet_converts_back_to_a_flight_command():
    pkt = SerialPacket(120, -60, 30, b"\xaa\x78\xc4\x1e")
    cmd = pkt.as_cmd(scale=120)
    assert cmd.throttle == pytest.approx(1.0)
    assert cmd.yaw == pytest.approx(-0.5)
    assert cmd.forward == pytest.approx(0.25)


# ----------------------------------------------------------------- the link
class _FakeSerial:
    """Captures what would go down the wire, with a controllable clock."""

    def __init__(self, fail_after=None):
        self.written = bytearray()
        self.fail_after = fail_after
        self.closed = False
        self.clock = 1000.0

    def write(self, payload: bytes) -> None:
        if self.fail_after is not None and len(self.written) >= self.fail_after:
            raise OSError("port disappeared")
        self.written.extend(payload)

    def close(self) -> None:
        self.closed = True


def _drone(monkeypatch, clock: dict | None = None, **kw):
    """A SerialDrone on a fake port, with the clock the test is about to install.

    ``_last_tx`` starts one period *before* the fake "now", i.e. the state a
    running 20 Hz loop is in: the next tick may send immediately. (Leaving the
    real monotonic value there makes the first send land inside a stale 50 ms
    window and get correctly dropped, which reads like a failure but is not.)
    """
    kw.setdefault("hz", 20.0)
    clock = clock or {"t": 1000.0}
    d = SerialDrone(port="COM_TEST", **kw)
    monkeypatch.setattr(d, "_open", lambda: setattr(d, "serial", _FakeSerial()))
    monkeypatch.setattr(d, "_maybe_reconnect", lambda: None)
    d._open()
    d._last_tx = clock["t"] - d.period
    return d


def test_sends_exactly_one_frame_per_tick(monkeypatch):
    now = {"t": 100.0}
    d = _drone(monkeypatch, clock=now)
    d.armed = True
    monkeypatch.setattr(time, "monotonic", lambda: now["t"])

    # a tight burst of commands inside one 50 ms window must collapse to one frame
    for i in range(10):
        d.send(FlightCommand(throttle=i / 10.0))
        now["t"] += 0.001
    assert len(d.serial.written) == 4

    now["t"] += 0.05  # next tick
    d.send(FlightCommand(throttle=1.0))
    assert len(d.serial.written) == 8
    frames = decode_frames(bytearray(d.serial.written))
    assert len(frames) == 2
    assert frames[-1].throttle == 120


def test_send_rate_is_the_control_rate(monkeypatch):
    """Over a run, the port sees one frame per control tick - no more, no less.

    ``frames_sent`` counts frames written to the port. If the send happened per
    ``send()`` call instead of per tick, a brain running faster than real time
    would write several frames into one 50 ms window and the FC would see a
    higher rate than ``control.hz`` - this test pins the rate to the tick.
    """
    now = {"t": 1000.0}
    d = _drone(monkeypatch, hz=20.0, clock=now)
    d.armed = True
    monkeypatch.setattr(time, "monotonic", lambda: now["t"])
    ticks = 200
    for _ in range(ticks):
        now["t"] += 0.05                      # one control period...
        d.send(FlightCommand(throttle=1.0))   # ...then one tick, as run_realtime does
    # One frame per tick, so the FC sees exactly control.hz and nothing more.
    # (The epsilon in the limiter keeps 20 Hz from collapsing to 10 Hz when a
    # loop sleeps exactly one period and floating point lands on the boundary.)
    assert d.frames_sent == ticks
    assert d.bytes_sent == 4 * d.frames_sent
    assert len(d.serial.written) == 4 * d.frames_sent
    assert decode_frames(bytearray(d.serial.written))[0].throttle == 120


def test_bursts_inside_one_window_collapse_to_a_single_frame(monkeypatch):
    """A brain running faster than real time must not flood the flight controller."""
    now = {"t": 1000.0}
    d = _drone(monkeypatch, hz=20.0, clock=now)
    d.armed = True
    monkeypatch.setattr(time, "monotonic", lambda: now["t"])
    for _ in range(20):           # 20 send() calls, all inside the same 50 ms window
        d.send(FlightCommand(throttle=1.0))
        now["t"] += 0.001
    assert d.frames_sent == 1
    assert len(d.serial.written) == 4


def test_unarmed_zero_frame_convention(monkeypatch):
    now = {"t": 1000.0}
    d = _drone(monkeypatch, zero_frame=True, clock=now)
    monkeypatch.setattr(time, "monotonic", lambda: now["t"])
    now["t"] += d.period                       # the first real tick of the loop
    d.send(FlightCommand(throttle=1.0))
    assert bytes(d.serial.written) == bytes([0xAA, 0, 0, 0])  # safe while disarmed
    d.takeoff()
    now["t"] += d.period                       # let the 20 Hz window roll over
    d.send(FlightCommand(throttle=1.0))
    assert bytes(d.serial.written[-4:]) == bytes([0xAA, 120, 0, 0])  # live once armed


def test_dead_link_does_not_stall_the_control_loop(monkeypatch):
    d = SerialDrone(port="COM_TEST", hz=20.0)
    d.serial = _FakeSerial(fail_after=0)
    monkeypatch.setattr(d, "_maybe_reconnect", lambda: None)
    d.armed = True
    t0 = time.monotonic()
    for _ in range(400):  # 20 s of control ticks at 20 Hz, every write failing
        d._last_tx = -1e9
        d.send(FlightCommand(throttle=1.0))
    elapsed = time.monotonic() - t0
    assert elapsed < 1.0, f"a dead port must not stall the control loop (took {elapsed:.2f} s)"
    assert d.link_errors >= 1
    assert d.serial is None  # dropped, ready for the reconnect backoff


def test_land_and_emergency_stop_always_end_at_zero(monkeypatch):
    """Whatever happened before, the last bytes on the wire are a zero frame.

    ``land`` and ``emergency_stop`` must not go through the 20 Hz rate limiter:
    they run on the way out, when the control loop has usually already stopped,
    and they are the paths that have to work when everything else is late.
    """
    now = {"t": 1000.0}
    d = _drone(monkeypatch, clock=now)
    d.armed = True
    monkeypatch.setattr(time, "sleep", lambda s: None)
    monkeypatch.setattr(time, "monotonic", lambda: now["t"])
    d.land()
    assert not d.armed
    assert bytes(d.serial.written[-4:]) == bytes([0xAA, 0, 0, 0])

    d.armed = True
    d.send(FlightCommand(yaw=1.0))
    d.emergency_stop()
    assert bytes(d.serial.written[-4:]) == bytes([0xAA, 0, 0, 0])


def test_config_defaults_cover_the_serial_link():
    cfg = load_config()
    assert cfg["serial"]["scale"] == 120
    assert cfg["control"]["hz"] == 20
    assert cfg["camera"]["width"] == 640 and cfg["camera"]["height"] == 480
    from pathlib import Path

    repo = Path(__file__).resolve().parents[1]
    assert (repo / "configs" / "serial_usb.yaml").exists()
    assert (repo / "tools" / "uart_loopback_sim.py").exists()
    assert (repo / "tools" / "serial_monitor.py").exists()


# ------------------------------------------------- sender to receiver agreement
def _receiver():
    """The reference receiver, as shipped in tools/uart_loopback_sim.py."""
    import importlib.util
    from pathlib import Path

    path = Path(__file__).resolve().parents[1] / "tools" / "uart_loopback_sim.py"
    spec = importlib.util.spec_from_file_location("uart_loopback_sim", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.Receiver


def test_reference_receiver_decodes_what_the_sender_writes():
    """The two halves of the protocol must agree, value for value."""
    Receiver = _receiver()
    cases = [
        FlightCommand(),
        FlightCommand(throttle=1.0, yaw=-1.0, forward=0.5),
        FlightCommand(throttle=-0.25, yaw=0.75, forward=-1.0),
    ]
    stream = b"".join(encode_frame(c) for c in cases)
    rx = Receiver(scale=120)
    got = rx.feed(stream, now=1.0)
    assert got == [(0, 0, 0), (120, -120, 60), (-30, 90, -120)]


def test_reference_receiver_survives_noise_and_partial_frames():
    Receiver = _receiver()
    good = encode_frame(FlightCommand(throttle=0.5, yaw=0.25, forward=-0.75))
    payload = bytes([0x00, 0xFF]) + good[:3] + good[3:] + bytes([0xAA, 0x79, 0x00])
    rx = Receiver(scale=120)
    assert rx.feed(payload, now=1.0) == [(60, 30, -90)]
    assert rx.dropped > 0  # it noticed the noise


def test_reference_receiver_failsafes_to_zero():
    """The only failsafe that matters: silence must mean neutral, not last command."""
    Receiver = _receiver()
    rx = Receiver(scale=120, timeout_ms=300)
    rx.feed(encode_frame(FlightCommand(throttle=1.0, yaw=1.0)), now=10.0)
    assert rx.command == (120, 120, 0)
    assert rx.poll_failsafe(10.1) is False       # still inside the timeout
    assert rx.poll_failsafe(10.4) is True        # link considered lost
    assert rx.command == (0, 0, 0)               # sticks neutral, not the last command
