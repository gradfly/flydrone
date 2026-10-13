"""Descending-neuron firing rates -> FlightCommand.

The decoder is deliberately a transparent linear read-out:

    axis = gain * sum_i  weight_i * (rate_i - baseline_i)

Baselines are measured while the brain settles with no stimulus. Weights come
from ``defaults.yaml`` (literature-guided) or from ``flydrones calibrate``
(fitted on the connectome you actually use). The brain's own wiring is never
changed.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

from .command import AXES, FlightCommand
from .escape import EscapePicker

# Giant-fiber escape reflex. The *direction* is read off the retina (see escape.py); the
# two axes below only affect escapes with no visual cue at all (a bump against something,
# or a dead camera), where the fly falls back to a random maneuver.
ESCAPE_CHOICES = ("climb", "left", "right")


class MotorDecoder:
    def __init__(self, cfg: dict):
        d = cfg.get("decoder", {})
        self.smoothing = float(d.get("smoothing", 0.35))
        self.deadzone = float(d.get("deadzone", 0.04))
        self.settle_s = float(d.get("settle_s", 1.5))
        self.axes = {a: dict(d.get("axes", {}).get(a, {"gain": 0.0, "terms": {}})) for a in AXES}
        self.cruise = float(d.get("cruise", 0.0))
        # The looming brake (DNp03) can exceed 1, and with a plain max(0, 1 - brake) the
        # cruise then goes to zero and the drone hangs motionless in mid-air until the
        # brake decays (~1 s). That is the third and least obvious source of "hovering",
        # and has nothing to do with touching anything. A fly slows down, it does not
        # stop dead, so autoplay keeps a floor under the cruise.
        esc = d.get("escape", {}) or {}
        self.cruise_floor = float(esc.get("cruise_floor", 0.0))
        self.brake_terms = dict(d.get("brake_terms", {}))
        self.escape_terms = list(esc.get("terms", []))
        self.escape_threshold = float(esc.get("threshold_hz", 25))
        # Speed-adaptive looming threshold. The looming signal (expansion rate) is proportional
        # to V/Z, so a fixed threshold triggers at very different distances depending on speed.
        # Scaling it by speed / v_ref keeps the trigger distance roughly constant:
        #   speed > v_ref -> threshold rises -> triggers later (closer to the obstacle)
        #   speed < v_ref -> threshold drops -> triggers earlier (not too late at low speed)
        # Clamped to [0.3, 3.0] x v_ref so a standstill does not fire on noise and a burst does
        # not make the reflex unreachable. v_ref defaults to 0.8 m/s, near a typical cruise.
        self.esc_v_ref = float(esc.get("v_ref", 0.8))
        # Contact escape: a drone pressed against a surface sees a *static* image, so there
        # is no expansion and the looming reflex can never fire. The bump itself becomes a
        # second trigger for the same reflex (rate-limited, so a long slide along a wall
        # does not turn into non-stop twitching).
        self.contact_cooldown = float(esc.get("contact_s", 0.9))
        self.esc_ceil_alt = esc.get("ceil_alt")  # None = no clamp (drones without altitude feedback)
        self.picker = EscapePicker(esc)
        # The 180 deg about-face holds full yaw until the drone has swung pi rad. Yaw is a
        # first-order lag (tau = 0.35 s) on the max yaw rate, so hold ~= pi / yaw_rate + tau.
        # Drones advertise their own yaw rate; without one we keep the picker's configured value.
        yaw_rate_dps = esc.get("yaw_rate_dps")
        if yaw_rate_dps is None:
            yaw_rate_dps = getattr(self, "yaw_rate_max_dps", None)
        if yaw_rate_dps:
            self.picker.set_yaw_rate(float(yaw_rate_dps))
        self._contact_until = -1.0
        self.baseline: dict[str, float] = {}
        self._settle_sum: dict[str, float] = {}
        self._settle_n = 0
        self._elapsed = 0.0
        self._state = {a: 0.0 for a in AXES}
        self._gf = 0.0  # smoothed giant-fiber rate
        self._brake = 0.0
        if d.get("readout_file"):
            self.load_readout(d["readout_file"])
        self.last_escape_t = None
        self.esc_cause: str | None = None  # "loom" | "contact"

    # ------------------------------------------------------------------ held state
    # Kept as properties so the names the old random-only decoder used still work.
    @property
    def _escape_until(self) -> float:
        return self.picker.esc_until

    @property
    def _escape_pick(self) -> str | None:
        return self.picker.esc_pick

    def load_readout(self, path: str | Path) -> None:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        for axis, spec in data.get("axes", {}).items():
            self.axes[axis] = {"gain": float(spec.get("gain", 1.0)), "terms": dict(spec["terms"])}
        if "baseline" in data:
            self.baseline = {k: float(v) for k, v in data["baseline"].items()}
            self.settle_s = 0.0

    @property
    def settled(self) -> bool:
        return self._elapsed >= self.settle_s

    def update(self, rates: dict[str, float], dt: float, tel=None, flow=None) -> FlightCommand:
        """Descending-neuron rates -> one command.

        ``tel`` (telemetry) and ``flow`` (the per-half near cue from the retina) are
        optional: without them the decoder behaves exactly as before - a looming escape
        is still triggered, but its direction comes from the random fallback instead of
        from the eye, and contact escapes cannot fire.
        """
        self._elapsed += dt
        self.picker._elapsed = self._elapsed
        if not self.settled:
            for k, v in rates.items():
                self._settle_sum[k] = self._settle_sum.get(k, 0.0) + v
            self._settle_n += 1
            return FlightCommand.hover("settling: measuring resting rates")
        if self._settle_n and not self.baseline:
            self.baseline = {k: v / self._settle_n for k, v in self._settle_sum.items()}

        raw = {}
        for axis, spec in self.axes.items():
            val = sum(w * (rates.get(g, 0.0) - self.baseline.get(g, 0.0)) for g, w in spec.get("terms", {}).items())
            raw[axis] = float(spec.get("gain", 0.0)) * val
        brake = sum(w * max(0.0, rates.get(g, 0.0) - self.baseline.get(g, 0.0)) for g, w in self.brake_terms.items())
        self._brake = max(brake, self._brake * 0.93)  # a scare keeps the fly cautious for a couple of seconds
        raw["forward"] += self.cruise * max(self.cruise_floor, 1.0 - self._brake)

        a = self.smoothing
        for axis in AXES:
            v = max(-1.0, min(1.0, raw[axis]))
            self._state[axis] = (1 - a) * self._state[axis] + a * v
        out = {k: (0.0 if abs(v) < self.deadzone else v) for k, v in self._state.items()}
        cmd = FlightCommand(**out)

        # ---- giant-fibre escape reflex -----------------------------------------
        if self.escape_terms:
            gf_now = max(rates.get(g, 0.0) for g in self.escape_terms)
            self._gf = 0.6 * self._gf + 0.4 * gf_now  # ~100 ms memory: one stray spike is not an escape
            touching = bool(getattr(tel, "touching", False))
            contact_ready = touching and self._elapsed > self._contact_until
            alt = getattr(tel, "alt_m", None)
            above_ceil = self.esc_ceil_alt is not None and alt is not None and alt > self.esc_ceil_alt

            # Read the eye every frame, so the side EMAs carry history into the moment
            # the escape actually fires.
            eye = self.picker.read(flow, 0.0)
            retrigger = self.picker.can_retrigger()
            # Speed-adaptive threshold: the looming signal ~ V/Z, so scaling the threshold by
            # speed / v_ref keeps the trigger distance roughly constant across speeds. A missing
            # speed (drones without an estimator, or the warmup pass) is treated as v_ref, i.e.
            # the base threshold is used unchanged.
            speed = getattr(tel, "speed", None)
            speed = self.esc_v_ref if speed is None or not math.isfinite(speed) else float(speed)
            speed_ratio = min(3.0, max(0.3, speed / max(1e-6, self.esc_v_ref)))
            threshold_eff = self.escape_threshold * speed_ratio
            if (self._gf >= threshold_eff or contact_ready) and (not self.picker.pending() or retrigger):
                if contact_ready:
                    self._contact_until = self._elapsed + self.contact_cooldown
                self.esc_cause = "contact" if contact_ready else "loom"
                self.picker.trigger(eye, above_ceil=above_ceil, retrigger=retrigger)
                self.last_escape_t = self._elapsed

        if self.picker.pending():
            cmd.escape = True
            self.picker.apply(cmd, above_ceiling=self._above_escape_ceiling(tel))
            cmd.note = self.picker.note()
        return cmd

    def _above_escape_ceiling(self, tel) -> bool:
        if self.esc_ceil_alt is None or tel is None:
            return False
        alt = getattr(tel, "alt_m", None)
        return alt is not None and alt > self.esc_ceil_alt
