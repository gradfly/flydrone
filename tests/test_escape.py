"""The escape direction is a lookup off the retina, not a dice roll.

Ported from the browser demo (docs/live/engine.js, build v22). The rule:

    left half swelling harder  -> turn right
    right half swelling harder -> turn left
    both halves alike          -> head-on: a 180 deg about-face (esc_turn180_hold seconds)
    nothing readable at all    -> the random fallback mix (pick_p); a bump does this

A head-on loom used to trigger a *climb*, and ``climb`` is still in the random fallback
mix, but on the visual path the about-face replaced it: climbing on every head-on turned
the flight into a yo-yo and gained altitude the governor had to undo.

Every assertion here is about *determinism*: the same geometry must always give the
same answer, and the answer must point away from the approaching object.
"""
import numpy as np

from flydrones.config import load_config
from flydrones.motor import FlightCommand, MotorDecoder
from flydrones.motor.escape import EscapePicker, NearCue, per_half_divergence
from flydrones.safety import Telemetry
from flydrones.senses.retina import EyeFeatures, FEATURES, VisualFrame


def _frame(vx, vy, tex=None):
    """A VisualFrame carrying only the flow field the near cue reads."""
    vx = np.asarray(vx, dtype=np.float32)
    vy = np.asarray(vy, dtype=np.float32)
    if tex is None:
        tex = np.ones_like(vx)
    zero = np.zeros((1, 1), np.float32)
    eyes = {e: EyeFeatures({f: zero for f in FEATURES}) for e in "LR"}
    return VisualFrame(eyes, vx=vx, vy=vy, tex=np.asarray(tex, dtype=np.float32))


def _diverging(rows=6, cols=8, strength=0.5, sides=("L", "R")):
    """Flow that expands on the named halves, at ``strength`` per cell."""
    vx = np.zeros((rows, cols * 2), np.float32)
    vy = np.zeros((rows, cols * 2), np.float32)
    for r in range(rows):
        for c in range(cols * 2):
            half = "L" if c < cols else "R"
            s = strength if half in sides else 0.0
            x = c - (cols - 1) / 2           # centred on each half
            y = r - (rows - 1) / 2
            vx[r, c] = s * x
            vy[r, c] = s * y
    return vx, vy


def test_divergence_locates_the_swelling_half():
    vx, vy = _diverging(sides=("L",))
    dl, dr, dall = per_half_divergence(_frame(vx, vy))
    assert dl > 0 > dr
    assert abs(dl) > 5 * abs(dr)         # the still half is ~0, the swelling half is not
    assert dall > 0


def test_no_flow_returns_none():
    assert per_half_divergence(_frame(np.zeros((6, 16)), np.zeros((6, 16)))) is not None
    # a frame with no field attached at all (Retina.encode(None)) has nothing to read
    zero = np.zeros((1, 1), np.float32)
    eyes = {e: EyeFeatures({f: zero for f in FEATURES}) for e in "LR"}
    assert per_half_divergence(VisualFrame(eyes)) is None


def test_picker_turns_away_from_the_swelling_half():
    """The whole point of the port: same geometry, same answer, every time.

    The side that is swelling is the side the obstacle is on, so the pick must be the
    *opposite* direction. The concrete sign is the backend's yaw convention (+1 = right,
    see ``SimDrone``) and is pinned here so a careless flip cannot pass unnoticed.
    """
    left_close = EscapePicker()
    right_close = EscapePicker()
    for _ in range(50):  # the EMAs need a few frames to settle
        a = left_close.from_flow((2.0, 1.0, 3.0))   # left half swelling harder
        b = right_close.from_flow((1.0, 2.0, 3.0))  # right half swelling harder
    assert a == "right", "a threat swelling on the image-left must be escaped by turning right"
    assert b == "left"
    # and it stays put: no dice, so repeating the read cannot change the answer
    assert {left_close.from_flow((2.0, 1.0, 3.0)) for _ in range(30)} == {"right"}


def test_direction_points_away_from_the_real_obstacle():
    """Ground truth: with a real box off to one side, every turn goes *away* from it.

    This is the test that caught the sign error. Three earlier versions of it were
    vacuous, so the traps are worth recording:

    * aiming the camera *at* the box makes the box's lateral offset exactly 0 by
      construction, i.e. the "correct answer" is undefined - the test then scores ~50 %
      and a flipped sign looks identical to a correct one;
    * probing ``per_half_divergence`` instead of ``EscapePicker`` tests only geometry,
      which is sign-agnostic, so an inverted sign still scored 83 %;
    * asserting "the swelling half is the obstacle half" is wrong for a large box, where
      both halves swell and the correct read is head-on (``turn180``), not a side.

    The invariant that actually matters - and that inverts cleanly under the bug - is:
    *whenever the rule commits to a turn, the turn must be away from the box.* Correct
    sign scores 3/3 away and 0 into; the inverted sign scored 0/3 away and 3 into.
    """
    import math
    from flydrones.drones.sim import Room, SimDrone
    from flydrones.senses import Retina

    room = Room.bedroom()
    drone = SimDrone(room=room, start=(0, 0, 0.9), seed=0)
    drone.takeoff()
    drone.flying = True
    ret = Retina.from_config(load_config())
    turns = away = into = 0
    for box in room.boxes:
        cx = (box.lo[0] + box.hi[0]) / 2
        cy = (box.lo[1] + box.hi[1]) / 2
        rad = max(box.hi[0] - box.lo[0], box.hi[1] - box.lo[1]) / 2
        dist = rad + 1.3
        for sgn in (+1, -1):
            # Camera on a fixed heading (+x), box parked clearly off-axis in y. This keeps
            # |lateral offset| ~0.9 m instead of the ~0 that aiming at the centre produces.
            px, py, yaw = cx - dist, cy + sgn * 0.9, 0.0
            drone.pos = np.array([px, py, 0.9])
            drone.yaw = yaw
            ret.reset()
            # Body axes as the sim actually flies them: measured empirically, not assumed
            # (yaw=0, forward=+1 -> +x; yaw=0, lateral=+1 -> -y), which matches sim.py.
            fx, fy = math.cos(yaw), math.sin(yaw)
            rx, ry = math.sin(yaw), -math.cos(yaw)
            side = (cx - px) * rx + (cy - py) * ry
            if (cx - px) * fx + (cy - py) * fy <= 0.4:   # box behind the camera: skip
                continue
            obstacle = "R" if side > 0 else "L"
            turn_away = "left" if obstacle == "R" else "right"

            picker = EscapePicker()
            pick = None
            for _ in range(20):                          # let the EMAs settle
                drone.pos[:2] += np.array([fx, fy]) * 0.02
                pick = picker.read(per_half_divergence(ret.encode(drone.frame())), 0.02)
            if pick not in ("left", "right"):
                continue                                 # head-on / no cue: not a turn, don't score
            turns += 1
            if pick == turn_away:
                away += 1
            else:
                into += 1
    assert turns >= 3, f"expected real turn decisions to score, got {turns}"
    assert into == 0, f"{into}/{turns} turns went *into* the obstacle - escape direction is inverted"
    assert away == turns


def test_head_on_read_is_a_turn180():
    """Both halves swelling alike = the thing is coming head-on: turn around and fly back."""
    p = EscapePicker()
    for _ in range(50):
        assert p.from_flow((5.0, 5.0, 10.0)) == "turn180"


def test_turn180_pivots_on_the_spot():
    """The about-face takes over all steer channels and holds for its own window."""
    p = EscapePicker({"duration_s": 0.7, "turn180_hold_s": 1.85})
    assert p.trigger("turn180") == "turn180"
    assert p.pending()
    cmd = FlightCommand(forward=0.35)   # a fresh command, so only the escape's writes count
    p.apply(cmd)
    assert cmd.forward == 0.0 and cmd.throttle == 0.0
    assert cmd.yaw == p.turn180_dir * p.strength
    # the hold is its own knob, not duration_s
    assert abs(p.esc_until - p._elapsed - 1.85) < 1e-9


def test_turn180_hold_tracks_the_yaw_rate():
    """A faster airframe needs a shorter about-face to swing the same pi rad.

    Hold ~= pi / yrMax + tau (tau = 0.35 s yaw lag), the same relation the JS engine uses.
    """
    p = EscapePicker({"yaw_rate_dps": 120})
    assert abs(p.turn180_hold - 1.85) < 0.02          # the engine default
    p.set_yaw_rate(180)
    assert abs(p.turn180_hold - 1.35) < 0.02          # the autoplay yaw rate
    p.set_yaw_rate(90)
    assert abs(p.turn180_hold - 2.35) < 0.02
    # an explicit turn180_hold_s pins the value and is not recomputed
    q = EscapePicker({"turn180_hold_s": 1.0, "yaw_rate_dps": 120})
    q.set_yaw_rate(180)
    assert q.turn180_hold == 1.0


def test_turn180_direction_comes_from_the_margin():
    """A head-on read has no real side, but the marginal sign of side_ema still picks one."""
    p = EscapePicker()
    for _ in range(50):
        p.from_flow((5.2, 5.0, 10.2))                 # left margin slightly harder
    p.trigger("turn180")
    assert p.turn180_dir == 1                         # ...so pivot to the right


def test_static_image_gives_no_cue():
    """A bump against a wall is a static image: no expansion, so no side to read."""
    p = EscapePicker()
    assert p.from_flow((0.0, 0.0, 0.0)) is None
    assert p.from_flow((0.0, 0.0, 0.0001)) is None  # below side_min


def test_no_cue_fallback_uses_the_pick_p_mix():
    """With no cue at all the random mix runs: climb, left, right in pick_p proportions."""
    p = EscapePicker({"pick_p": {"climb": 1.0, "left": 0.0}})
    assert {p.trigger(None) for _ in range(20)} == {"climb"}
    assert p.nocue == 20

    q = EscapePicker({"pick_p": {"climb": 0.0, "left": 0.0}})
    assert {q.trigger(None) for _ in range(20)} == {"right"}


def test_no_cue_climb_is_downgraded_at_the_ceiling():
    """No cue + already at the ceiling: the fallback climb is spent as a turn."""
    p = EscapePicker({"pick_p": {"climb": 1.0, "left": 0.0}})
    picks = [p.trigger(None, above_ceil=True) for _ in range(40)]
    assert set(picks) <= {"left", "right"}


def test_climb_holds_longer_than_a_turn():
    p = EscapePicker({"climb_hold_s": 1.2, "duration_s": 0.7, "pick_p": {"climb": 1.0}})
    p.trigger(None)                                    # no cue -> the fallback can pick climb
    assert p.pending()
    # a climb keeps flying forward: only the vertical channel is taken over
    cmd = FlightCommand(forward=0.35, yaw=0.0, throttle=0.0)
    p.apply(cmd)
    assert cmd.throttle == p.strength
    assert cmd.forward == 0.35, "a climb must not stall the cruise"
    assert cmd.yaw == 0.0
    assert abs(p.esc_until - p._elapsed - 1.2) < 1e-9

    t = EscapePicker({"duration_s": 0.7})
    t.trigger("turn180")
    cmd = FlightCommand(forward=0.35)
    t.apply(cmd)
    assert cmd.forward == 0.0 and cmd.throttle == 0.0


def test_escape_ceiling_stops_the_climb():
    p = EscapePicker({"pick_p": {"climb": 1.0, "left": 0.0}})
    p.trigger(None)
    cmd = FlightCommand(forward=0.35)
    p.apply(cmd, above_ceiling=True)
    assert cmd.throttle == 0.0, "already at the ceiling: stop gaining height"
    assert cmd.forward == 0.35, "...but keep flying"


def test_turn_takes_over_all_steer_channels():
    p = EscapePicker()
    p.trigger("left")
    cmd = FlightCommand(throttle=0.5, yaw=0.9, forward=0.35)
    p.apply(cmd)
    assert cmd.yaw == -p.strength and cmd.forward == 0.0 and cmd.throttle == 0.0


def test_contact_is_a_second_trigger():
    """A drone pressed on a surface sees a static image; the bump must still unstick it."""
    cfg = load_config()
    dec = MotorDecoder(cfg)
    rest = {"DNg02_L": 35, "DNg02_R": 35, "DNp03_L": 0, "DNp03_R": 0, "DNp01_L": 0, "DNp01_R": 0}
    for _ in range(40):
        dec.update(rest, 0.05)                       # settle, no giant-fiber activity at all
    # no looming, no flow cue, but touching something -> escape with the random fallback
    cmd = dec.update(rest, 0.05, tel=Telemetry(t=2, alt_m=1.0, flying=True, touching=True), flow=None)
    assert cmd.escape, "a contact must fire the reflex even with zero looming"
    assert dec.esc_cause == "contact"
    assert dec.picker.nocue == 1, "no visual cue available, so the random mix was used"


def test_contact_cooldown_stops_nonstop_twitch():
    cfg = load_config()
    dec = MotorDecoder(cfg)
    rest = {"DNg02_L": 35, "DNg02_R": 35, "DNp03_L": 0, "DNp03_R": 0, "DNp01_L": 0, "DNp01_R": 0}
    for _ in range(40):
        dec.update(rest, 0.05)
    tel = Telemetry(t=2, alt_m=1.0, flying=True, touching=True)
    dec.update(rest, 0.05, tel=tel, flow=None)
    fired = dec.last_escape_t
    for _ in range(5):                              # still touching, 250 ms later
        dec.update(rest, 0.05, tel=tel, flow=None)
    assert dec.last_escape_t == fired, "a long slide along a wall must not re-trigger every frame"


def test_loom_direction_comes_from_the_eye_when_available():
    """With a flow cue and no dice, the pick is the eye's reading."""
    cfg = load_config()
    dec = MotorDecoder(cfg)
    rest = {"DNg02_L": 35, "DNg02_R": 35, "DNp03_L": 0, "DNp03_R": 0, "DNp01_L": 0, "DNp01_R": 0}
    for _ in range(40):
        dec.update(rest, 0.05)
    loud = {**rest, "DNp01_L": 90}
    for _ in range(5):
        dec.update(loud, 0.05, tel=Telemetry(t=1, alt_m=1.0, flying=True), flow=(2.0, 1.0, 3.0))
    # flow (2.0, 1.0) = left half swelling harder, so the pick is a turn to the right
    assert dec.picker.esc_eye == "right"
    assert dec.esc_cause == "loom"


def test_loom_threshold_scales_with_speed():
    """The looming signal ~ V/Z, so the threshold is scaled by speed / v_ref.

    Slower flight -> lower threshold -> the reflex fires earlier; faster flight -> it waits
    until closer. A missing speed (no estimator) must leave the base threshold alone.
    """
    cfg = load_config()
    cfg["decoder"]["escape"]["v_ref"] = 0.8
    cfg["decoder"]["escape"]["threshold_hz"] = 20
    dec = MotorDecoder(cfg)
    assert dec.esc_v_ref == 0.8
    # 20 Hz * (0.5 / 0.8) = 12.5 Hz effective; 20 Hz * (2.0 / 0.8) = 50 Hz.
    rest = {"DNg02_L": 35, "DNg02_R": 35, "DNp03_L": 0, "DNp03_R": 0, "DNp01_L": 0, "DNp01_R": 0}

    def fires_at(speed: float) -> bool:
        d = MotorDecoder(cfg)
        for _ in range(40):
            d.update(rest, 0.05)
        loud = {**rest, "DNp01_L": 30}   # 30 Hz gf: over 12.5, under 50
        for _ in range(3):
            d.update(loud, 0.05, tel=Telemetry(t=1, alt_m=1.0, flying=True, speed=speed), flow=None)
        return d.last_escape_t is not None

    assert fires_at(0.5), "at half the reference speed a 30 Hz loom must still fire"
    assert not fires_at(2.0), "at 2.5x the reference speed 30 Hz is below the raised threshold"


def test_legacy_axes_still_work():
    """Decoder.cruise / brake are untouched by the port."""
    cfg = load_config()
    cfg["decoder"]["cruise"] = 0.4
    cfg["decoder"]["smoothing"] = 1.0
    dec = MotorDecoder(cfg)
    rest = {"DNg02_L": 35, "DNg02_R": 35, "DNp03_L": 0, "DNp03_R": 0, "DNp01_L": 0, "DNp01_R": 0}
    for _ in range(40):
        dec.update(rest, 0.05)
    assert abs(dec.update(rest, 0.05).forward - 0.4) < 0.02
    assert dec.update({**rest, "DNp03_L": 60, "DNp03_R": 60}, 0.05).forward < 0.1


def test_cruise_floor_keeps_flying_through_a_brake():
    cfg = load_config()
    cfg["decoder"]["cruise"] = 0.5
    cfg["decoder"]["smoothing"] = 1.0
    cfg["decoder"]["escape"]["cruise_floor"] = 0.4
    dec = MotorDecoder(cfg)
    rest = {"DNg02_L": 35, "DNg02_R": 35, "DNp03_L": 0, "DNp03_R": 0, "DNp01_L": 0, "DNp01_R": 0}
    for _ in range(40):
        dec.update(rest, 0.05)
    hard_brake = {**rest, "DNp03_L": 200, "DNp03_R": 200}
    c = dec.update(hard_brake, 0.05)
    assert c.forward >= 0.5 * 0.4 - 0.02, "the cruise must never be braked to a standstill"


def test_near_cue_smooths_and_resets():
    cue = NearCue()
    cue.update(_frame(*_diverging(sides=("L",), strength=0.5)))
    first = cue.near_l
    # stronger expansion: the EMA must move towards it (a smoothing filter, not a latch)
    cue.update(_frame(*_diverging(sides=("L",), strength=1.0)))
    assert cue.near_l > first
    cue.reset()
    assert cue.near_l is None


def test_near_cue_keeps_a_per_half_split():
    """A one-sided loom must leave a clear asymmetry: that is what the pick reads."""
    cue = NearCue()
    for _ in range(10):
        near_l, near_r, _ = cue.update(_frame(*_diverging(sides=("L",))))
    assert near_l > 0 and near_r < near_l * 0.1
