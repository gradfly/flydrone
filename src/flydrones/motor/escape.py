"""Deterministic escape direction: read the looming side off the retina, not off dice.

Ported from the browser demo (``docs/live/engine.js``, build v22). In the simulator the
escape direction is a lookup instead of a lottery:

    left half swelling harder  -> turn right
    right half swelling harder -> turn left
    both halves alike          -> head-on: a 180 deg about-face (``esc_turn180_hold`` s)
    nothing readable at all    -> random fallback (``pick_p`` mix; a bump does this)
A head-on loom used to trigger a *climb*. It now triggers a **180 deg about-face**: a
climb on every head-on read turned the flight into a yo-yo and gained altitude that the
governor then had to undo, whereas an about-face is a pure dodge with no altitude side
effect. ``climb`` survives only as part of the *no-cue* random fallback.

The eye measurement is the **texture-weighted mean divergence of the flow field per
half**, because that is what actually locates an approaching object:

* local expansion is ``2 * Vz / Z`` - large exactly where a surface is close, so its
  mean over a half answers "which half of my world is about to hit me"
* the common background cancels out of the comparison: the floor and far walls sit in
  *both* halves, so they lift both means together and only the near object tilts the
  balance
* two neighbours that look reasonable but do **not** work, both measured on the JS side:
  per-half affine slope (``expL``/``expR``) matched the true obstacle bearing only
  ~50-60 % of the time, and a per-half **maximum** divergence always returns the floor
  (present in both halves) so every loom read as head-on.

Why the sign is backend-specific and measured, not reasoned: it is tempting to argue that
whichever way the image is mirrored, the swelling side and the escaping turn stay opposite,
so the direction must be flip-agnostic. It is not - mirroring the image also flips the
retina's left/right *feature* split while the command sign convention stays put, so the two
do not cancel. The sign in ``from_flow`` was measured against real obstacle bearings in the
simulator (``test_direction_points_away_from_the_real_obstacle``); with the opposite sign the
cue still fired on every loom but pointed at the obstacle instead of away from it, and
collisions in a 6-seed soak went up rather than down.
"""

from __future__ import annotations

import random

from ..senses.retina import VisualFrame

try:  # SciPy is a hard dependency, the fallback only matters in a stripped-down install
    from scipy.ndimage import map_coordinates as _map_coordinates
except Exception:  # pragma: no cover - optional
    _map_coordinates = None


def _neighbour_axis(a, axis: int):
    """Central difference in cell units, edge-clamped (matches ``np.gradient`` in retina.py)."""
    import numpy as np

    n = a.shape[axis]
    lo = np.concatenate([a.take([0], axis=axis), a.take(range(n - 1), axis=axis)], axis=axis)
    hi = np.concatenate([a.take(range(1, n), axis=axis), a.take([n - 1], axis=axis)], axis=axis)
    return (hi - lo) * 0.5


class NearCue:
    """Per-half depth-rate (mean flow divergence) with a short EMA, as the escape reads it."""

    def __init__(self) -> None:
        self.near_l: float | None = None
        self.near_r: float | None = None
        self.near_all: float | None = None

    def reset(self) -> None:
        self.near_l = self.near_r = self.near_all = None

    def update(self, frame: VisualFrame) -> tuple[float, float, float]:
        div = per_half_divergence(frame)
        if div is None:
            self.near_l = self.near_r = self.near_all = 0.0
            return 0.0, 0.0, 0.0
        dl, dr, dall = div
        self.near_l = dl if self.near_l is None else 0.4 * self.near_l + 0.6 * dl
        self.near_r = dr if self.near_r is None else 0.4 * self.near_r + 0.6 * dr
        self.near_all = dall if self.near_all is None else 0.4 * self.near_all + 0.6 * dall
        return self.near_l, self.near_r, self.near_all


def per_half_divergence(frame: VisualFrame) -> tuple[float, float, float] | None:
    """Texture-weighted mean flow divergence seen by the left half, the right half, both.

    Returns ``None`` when the frame carries no flow at all (``Retina.encode(None)``), so
    callers can tell "no information" apart from "no looming".
    """
    import numpy as np

    vx = getattr(frame, "vx", None)
    vy = getattr(frame, "vy", None)
    tex = getattr(frame, "tex", None)
    if vx is None or vy is None or tex is None or vx.size == 0:
        return None
    rows, ncols = vx.shape
    if rows < 2 or ncols < 4:
        return None

    diverg = _neighbour_axis(vx, axis=1) + _neighbour_axis(vy, axis=0)
    # The 0.5 of the central difference divides out of a weighted *mean*, so it is not needed.
    weight = tex + 1e-3
    half = ncols // 2
    wl, wr = weight[:, :half], weight[:, half:]
    sl, sr = float((wl * diverg[:, :half]).sum()), float((wr * diverg[:, half:]).sum())
    wl_s, wr_s = float(wl.sum()), float(wr.sum())
    dl = sl / wl_s if wl_s > 0 else 0.0
    dr = sr / wr_s if wr_s > 0 else 0.0
    wall = wl_s + wr_s
    dall = (sl + sr) / wall if wall > 0 else 0.0
    return dl, dr, dall


def _sample(field, ys, xs) -> float:
    if _map_coordinates is not None:
        return float(_map_coordinates(field, [ys, xs], order=1, mode="nearest"))
    # Nearest-neighbour fallback: only used when SciPy is missing.
    iy = max(0, min(field.shape[0] - 1, int(round(ys))))
    ix = max(0, min(field.shape[1] - 1, int(round(xs))))
    return float(field[iy, ix])


def attach_flow_to_frame(frame: VisualFrame, vx, vy, tex) -> VisualFrame:
    """Attach the raw flow field (px/cell/frame) the near cue reads.

    The retina computes this for its feature grids, but does not keep it; ``Pilot`` calls
    this right after ``Retina.encode`` so nothing is computed twice. Both live in
    ``senses`` and are on the same closed-loop path, so this is an interface, not a leak:
    no pixel data and no motor state crosses over.
    """
    frame.vx, frame.vy, frame.tex = vx, vy, tex
    return frame


# ----------------------------------------------------------------------- decoder
class EscapePicker:
    """Deterministic direction + hold timing for the giant-fibre escape.

    Owned by ``MotorDecoder``. ``configure()`` is fed the ``decoder.escape`` block and
    ``read(flow, dt)`` is called every tick, so the side EMAs carry history into the
    moment an escape actually fires.
    """

    def __init__(self, esc: dict | None = None) -> None:
        self.side_ema = 0.0
        self.tot_ema = 0.0
        self.esc_until = -1.0
        self.esc_pick: str | None = None
        self.esc_retrig_until = 0.0
        self.esc_eye: str | None = None
        self.esc_retrig = 0
        self.nocue = 0
        self.turn180_dir = 1  # yaw sign of the running about-face (+1 = right, -1 = left)
        self._hold_pinned = False  # True once the config sets turn180_hold_s explicitly
        self._elapsed = 0.0
        self.configure(esc)

    def configure(self, esc: dict | None) -> None:
        e = esc or {}
        # Geometry / gating (ported defaults from docs/live/app.js AUTO_TUNE, build v22).
        self.side_min = float(e.get("side_min", 0.01))          # below this the image is static: no cue
        self.side_close_frac = float(e.get("side_close_frac", 0.10))
        self.climb_hold = float(e.get("climb_hold_s", 1.2))     # a *fallback* climb keeps flying this long
        self.climb_retrig = float(e.get("climb_retrig_s", 0.3))  # shortest gap between two picks in a climb
        # 180 deg about-face on a head-on loom. The hold has to be long enough to swing pi rad
        # through the sim's first-order yaw lag (tau = 0.35 s) at the configured yaw rate:
        #   hold ~= pi / yr_max + tau
        # When ``yaw_rate_dps`` is not supplied the default matches the engine's 120 deg/s.
        turn180_hold = e.get("turn180_hold_s")
        self._hold_pinned = turn180_hold is not None
        self.turn180_hold = float(turn180_hold) if self._hold_pinned else self.turn180_hold_for(float(e.get("yaw_rate_dps", 120.0)))
        self.yaw_rate_dps = float(e.get("yaw_rate_dps", 120.0))
        # Legacy random fallback, used only when the retina gives no cue at all.
        self.esc_p = e.get("pick_p", {}) or {}
        self.p_climb = float(self.esc_p.get("climb", 0.10))
        self.p_left = float(self.esc_p.get("left", 0.45))
        # What the pick means mechanically.
        self.duration = float(e.get("duration_s", 0.7))
        self.strength = float(e.get("strength", 0.8))

    def turn180_hold_for(self, yaw_rate_dps: float, tau: float = 0.35) -> float:
        """Hold time that swings a full pi rad at ``yaw_rate_dps`` through the yaw lag."""
        yr = max(1e-3, abs(yaw_rate_dps)) * 3.141592653589793 / 180.0
        return 3.141592653589793 / yr + tau

    def set_yaw_rate(self, yaw_rate_dps: float, tau: float = 0.35) -> None:
        """Retune the about-face hold for the airframe's actual max yaw rate.

        A drone that turns faster needs a shorter hold for the same 180 deg, so the hold is
        derived rather than hard-coded. ``turn180_hold_s`` in the config still wins: setting
        it explicitly means "use this number", and ``apply_auto_tune`` uses that to pin the
        hold for the autoplay yaw rate.
        """
        self.yaw_rate_dps = float(yaw_rate_dps)
        if not self._hold_pinned:
            self.turn180_hold = self.turn180_hold_for(yaw_rate_dps, tau)

    # ------------------------------------------------------------------ reading
    def from_flow(self, flow: tuple[float, float, float] | None) -> str | None:
        """``"left"`` / ``"right"`` / ``"turn180"`` / ``None`` (no visual information)."""
        if flow is None:
            return None
        near_l, near_r, near_all = flow
        self.side_ema = 0.5 * self.side_ema + 0.5 * (near_l - near_r)
        self.tot_ema = 0.5 * self.tot_ema + 0.5 * (near_l + near_r)
        # A bump gives a *static* image: no expansion anywhere, so nothing to read a side
        # off - the caller falls back to the random mix.
        if abs(near_all) < self.side_min or self.tot_ema <= 0:
            return None
        # "Both halves alike" is relative: the divergence scale moves with speed and distance.
        if abs(self.side_ema) < self.side_close_frac * self.tot_ema:
            return "turn180"  # head-on: turn around and fly back the way we came
        # Turn AWAY from the half that is swelling harder.
        #
        # The sign here is backend-specific and was measured, not guessed. `FlightCommand.yaw`
        # is +1 = clockwise/right, while the fly-eye grids index column 0 as the *left* of the
        # image and `SimDrone` maps a +yaw command to a clockwise (right) turn. So a threat
        # swelling on the image-left must be escaped by turning right.
        #
        # Verified against real obstacle bearings in the simulator (test_escape.py
        # ::test_direction_points_away_from_the_real_obstacle). The invariant it pins is
        # "whenever the rule commits to a turn, the turn goes away from the box": the correct
        # sign scores 3/3 away and 0 into, the flipped sign 0/3 away and 3 into, because the
        # reflex still *fires* under the bug - it just steers into the obstacle, so every
        # "does it react?" check passes while the flight gets worse. Do not flip this without
        # re-running that test.
        return "right" if self.side_ema > 0 else "left"

    def read(self, flow: tuple[float, float, float] | None, dt: float) -> str | None:
        """Advance the clock and return this frame's cue (the escape pick uses it)."""
        self._elapsed += dt
        return self.from_flow(flow)

    # ------------------------------------------------------------------ picking
    @property
    def climbing(self) -> bool:
        return self.esc_pick == "climb" and self._elapsed <= self.esc_until

    def can_retrigger(self) -> bool:
        return self.climbing and self._elapsed > self.esc_retrig_until

    def pending(self) -> bool:
        return self._elapsed <= self.esc_until and self.esc_pick is not None

    def is_random_fallback(self, eye: str | None) -> bool:
        return eye is None

    def trigger(self, eye: str | None, above_ceil: bool = False, retrigger: bool = False) -> str:
        """Commit to a maneuver for this escape and arm its hold window."""
        self.esc_eye = eye
        if eye:
            # The reflex is a lookup, not a lottery. A head-on read ("turn180") has no side to
            # pick, so the drone turns around; the margin of side_ema (small but real) decides
            # which way, and only a truly zero side falls back to a coin.
            pick = eye
            if eye == "turn180":
                if abs(self.side_ema) > 1e-6:
                    self.turn180_dir = 1 if self.side_ema >= 0 else -1
                else:
                    self.turn180_dir = -1 if random.random() < 0.5 else 1
        else:
            self.nocue += 1  # blind escape: contact bump / dead vision
            r = random.random()
            self.esc_retrig += 1 if retrigger else 0
            if above_ceil and r < self.p_climb:
                # No cue + already at the ceiling: the fallback climb is spent as a turn.
                pick = "left" if random.random() < 0.5 else "right"
            else:
                pick = "climb" if r < self.p_climb else ("left" if r < self.p_climb + self.p_left else "right")
        self.esc_pick = pick
        if pick == "climb":
            self.esc_until = self._elapsed + self.climb_hold
            self.esc_retrig_until = self._elapsed + self.climb_retrig
        elif pick == "turn180":
            self.esc_until = self._elapsed + self.turn180_hold
            self.esc_retrig_until = 0.0
        else:
            self.esc_until = self._elapsed + self.duration
            self.esc_retrig_until = 0.0
        return pick

    def note(self) -> str:
        return f"giant fiber escape ({self.esc_pick})"

    def apply(self, cmd, above_ceiling: bool = False) -> None:
        """Take over the steer channels for the held window.

        ``above_ceiling`` is passed in by ``MotorDecoder`` (it owns ``esc_ceil_alt``): a
        *fallback* climb that has already reached the escape ceiling stops gaining height,
        so the altitude governor does not have to undo ~1 m after every loom. It does not
        affect a 180 deg about-face, which never changes altitude.
        """
        if self.esc_pick == "climb":
            # Fallback climb AND keep going: only the vertical channel is taken over, the wing
            # command stays the cruise the brain asked for, so the drone gains height while
            # still covering ground.
            cmd.throttle = 0.0 if above_ceiling else self.strength
        elif self.esc_pick == "turn180":
            # Pivot on the spot: full yaw one way, no forward, no throttle. When the hold
            # releases, the cruise resumes pointing the other way.
            cmd.forward = 0.0
            cmd.throttle = 0.0
            cmd.yaw = self.turn180_dir * self.strength
        else:
            cmd.forward = 0.0
            cmd.throttle = 0.0
            cmd.yaw = -self.strength if self.esc_pick == "left" else self.strength

    def summary(self) -> dict:
        return {"pick": self.esc_pick, "eye": self.esc_eye, "nocue": self.nocue,
                "retrig": self.esc_retrig, "turn180_dir": self.turn180_dir}
