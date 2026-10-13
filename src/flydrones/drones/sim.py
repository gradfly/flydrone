"""A tiny indoor quadcopter simulator with a ray-cast camera.

The simulated drone behaves like a Tello or a Crazyflie in velocity mode: the
onboard flight controller keeps it level and we command velocities. The
camera renders a textured room (checkered floor, striped walls) plus boxes
such as a chair, so the fly's eyes get real optic flow and looming.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ..motor.command import FlightCommand
from ..safety import Telemetry
from .base import Drone


@dataclass
class Box:
    lo: tuple[float, float, float]
    hi: tuple[float, float, float]
    shade: float = 0.18
    name: str = "box"


# ---------------------------------------------------------------------- arena layout
# Foam-block obstacles dropped into the arena at random (seeded RNG). Ported from the
# browser demo's makeObstacles(), including the launch pads kept clear around the pilot
# start positions and the minimum separation so the drone can slip between boxes.
OBSTACLE_COUNT = 8       # how many boxes the arena holds (tune me)
_MAT = 0.5               # floor-mat tile: the box footprint and the grid the lo corner snaps to
_STARTS = ((-1.5, 0.0), (-0.9, 0.05), (-1.9, -0.3))  # pilot starts (x, y): gestures / chair / game


def _i32(v: int) -> int:
    """Coerce to signed 32-bit, the way every JS bitwise operator does."""
    v &= 0xFFFFFFFF
    return v - 0x100000000 if v >= 0x80000000 else v


def _shr(v: int, n: int) -> int:
    """JS ``>>>`` : logical shift right on the unsigned 32-bit pattern."""
    return (v & 0xFFFFFFFF) >> n


def _imul32(a: int, b: int) -> int:
    """``Math.imul``: 32-bit signed integer multiplication, matching JS exactly."""
    return _i32(_i32(a) * _i32(b))


def _mulberry32(seed: int):
    """The browser demo's PRNG, so a given seed lays out the *same* arena in both simulators.

    JavaScript's bitwise ops coerce to **signed** int32, so the intermediate values have to
    be kept signed here too (``_i32``) or the sequence diverges on the first multiply.
    """
    i32 = _i32
    mul = _imul32
    a = i32(int(seed))

    def nxt() -> float:
        nonlocal a
        a = i32(a + 0x6D2B79F5)
        t = i32(mul(i32(a ^ _shr(a, 15)), i32(1 | a)))
        # NOTE the ``+``: it is ``(t + imul(t ^ t>>>7, 61|t)) ^ t``, not ``imul(...) ^ t``.
        t = i32(i32(t + mul(i32(t ^ _shr(t, 7)), i32(61 | t))) ^ t)
        return (i32(t ^ _shr(t, 14)) & 0xFFFFFFFF) / 4294967296.0

    return nxt


def make_obstacles(seed: int | None = None, count: int = OBSTACLE_COUNT) -> list[Box]:
    if seed is None:
        seed = int(np.random.default_rng().integers(0, 0xFFFFFFFF))
    rng = _mulberry32(int(seed))
    boxes: list[Box] = []
    guard = 0
    while len(boxes) < count and guard < 5000:
        guard += 1
        # lo in [-2.0, 1.5], so a 0.5 m box stays >= 1 m from the +/-3 m walls
        lo = [-2.0 + int(rng() * 8) * _MAT, -2.0 + int(rng() * 8) * _MAT]
        hi = [lo[0] + _MAT, lo[1] + _MAT]
        h = 1.1 + rng() * 0.9
        # keep a launch pad clear around every pilot start position
        if any(sx > lo[0] - 0.9 and sx < hi[0] + 0.9 and sy > lo[1] - 0.9 and sy < hi[1] + 0.9
               for sx, sy in _STARTS):
            continue
        # keep obstacles apart so the drone can slip between them
        if any(lo[0] < b.hi[0] + 0.8 and hi[0] > b.lo[0] - 0.8
               and lo[1] < b.hi[1] + 0.8 and hi[1] > b.lo[1] - 0.8 for b in boxes):
            continue
        boxes.append(Box((lo[0], lo[1], 0.0), (hi[0], hi[1], h),
                         0.16 + rng() * 0.14, f"obstacle{len(boxes) + 1}"))
    return boxes


@dataclass
class Room:
    size_x: float = 6.0
    size_y: float = 6.0
    height: float = 2.7
    boxes: list[Box] = field(default_factory=list)

    @staticmethod
    def bedroom() -> Room:
        return Room(boxes=[
            Box((1.2, -0.25, 0.0), (1.7, 0.25, 1.15), 0.12, "chair"),
            Box((-2.9, 1.2, 0.0), (-1.3, 2.9, 0.55), 0.3, "bed"),
            Box((2.4, -2.9, 0.0), (2.9, -1.9, 1.9), 0.22, "wardrobe"),
        ])

    @staticmethod
    def arena(seed: int | None = None, count: int = OBSTACLE_COUNT) -> Room:
        """The foam-block training arena, laid out exactly like the browser demo.

        Ported from ``docs/live/engine.js`` (build v22): ``count`` boxes on a 0.5 m mat
        grid, seeded so the simulator, the retina view and the 3D visuals always agree on
        the same layout. Pass a seed to reproduce a run.
        """
        return Room(boxes=make_obstacles(seed, count))


class RayCamera:
    def __init__(self, width: int = 96, height: int = 72, fov_deg: float = 82.0):
        self.w, self.h = width, height
        tx = math.tan(math.radians(fov_deg) / 2)
        ty = tx * height / width
        u = (np.arange(width) + 0.5) / width * 2 - 1
        v = 1 - (np.arange(height) + 0.5) / height * 2
        self.U, self.V = np.meshgrid(u * tx, v * ty)

    def render(self, room: Room, pos: np.ndarray, yaw: float) -> np.ndarray:
        fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        right = np.array([math.sin(yaw), -math.cos(yaw), 0.0])
        d = fwd[None, None, :] + self.U[..., None] * right[None, None, :] + self.V[..., None] * np.array([0, 0, 1.0])
        d /= np.linalg.norm(d, axis=-1, keepdims=True)
        dx, dy, dz = d[..., 0], d[..., 1], d[..., 2]
        px, py, pz = pos
        INF = 1e9
        t_best = np.full(dx.shape, INF)
        shade = np.zeros(dx.shape)

        def safe_div(a, b):
            with np.errstate(divide="ignore", invalid="ignore"):
                r = a / b
            r[~np.isfinite(r) | (r <= 1e-6)] = INF
            return r

        hx, hy = room.size_x / 2, room.size_y / 2
        # walls
        for plane, t in (("x+", safe_div(hx - px, dx)), ("x-", safe_div(-hx - px, dx)), ("y+", safe_div(hy - py, dy)), ("y-", safe_div(-hy - py, dy))):
            m = t < t_best
            if m.any():
                along = (py + t * dy) if plane[0] == "x" else (px + t * dx)
                zz = pz + t * dz
                stripe = (np.floor(along / 0.3) % 2) * 0.35 + 0.35
                band = np.where((np.floor(zz / 0.45) % 2) == 0, 1.0, 0.8)
                shade = np.where(m, stripe * band, shade)
                t_best = np.where(m, t, t_best)
        # floor and ceiling
        t = safe_div(-pz, dz)
        m = t < t_best
        fx, fy = px + t * dx, py + t * dy
        checker = ((np.floor(fx / 0.35) + np.floor(fy / 0.35)) % 2) * 0.35 + 0.25
        shade = np.where(m, checker, shade)
        t_best = np.where(m, t, t_best)
        t = safe_div(room.height - pz, dz)
        m = t < t_best
        shade = np.where(m, 0.9, shade)
        t_best = np.where(m, t, t_best)
        # boxes (slab method)
        for b in room.boxes:
            lo, hi = np.array(b.lo), np.array(b.hi)
            with np.errstate(divide="ignore", invalid="ignore"):
                t1 = (lo[0] - px) / dx
                t2 = (hi[0] - px) / dx
                t3 = (lo[1] - py) / dy
                t4 = (hi[1] - py) / dy
                t5 = (lo[2] - pz) / dz
                t6 = (hi[2] - pz) / dz
            tmin = np.maximum(np.maximum(np.minimum(t1, t2), np.minimum(t3, t4)), np.minimum(t5, t6))
            tmax = np.minimum(np.minimum(np.maximum(t1, t2), np.maximum(t3, t4)), np.maximum(t5, t6))
            hit = (tmax >= tmin) & (tmin > 1e-6) & (tmin < t_best)
            if hit.any():
                hx_, hy_, hz_ = px + tmin * dx, py + tmin * dy, pz + tmin * dz
                # fabric-like checker so the camera can see the surface move
                edge = ((np.floor((hx_ + hy_) / 0.12) + np.floor(hz_ / 0.12)) % 2) * 0.22
                shade = np.where(hit, b.shade + edge, shade)
                t_best = np.where(hit, tmin, t_best)
        shade = shade / (1 + 0.06 * np.minimum(t_best, 50))
        g = np.clip(shade * 255, 0, 255).astype(np.uint8)
        return np.stack([g, g, np.clip(g.astype(int) + 8, 0, 255).astype(np.uint8)], axis=-1)  # BGR, slight warm tint


class SimDrone(Drone):
    name = "sim"
    has_camera = True

    def __init__(self, room: Room | None = None, start=(0.0, 0.0, 0.0), yaw_deg: float = 0.0, seed: int = 0,
                 v_max: float = 1.0, vz_max: float = 0.8, yaw_rate_max_dps: float = 120.0, tau: float = 0.35,
                 wind: float = 0.03, camera: RayCamera | None = None, arena_seed: int | None = None):
        # The default arena is the seeded foam-block layout of the browser demo, so the two
        # simulators line up. ``Room.bedroom()`` (the old named-boxes room) is still available
        # and is what the unit tests use when they want fixed, known geometry.
        self.room = room or Room.arena(arena_seed if arena_seed is not None else seed)
        self.pos = np.array(start, dtype=float)
        self.vel = np.zeros(3)
        self.yaw = math.radians(yaw_deg)
        self.yaw_rate = 0.0
        self.v_max, self.vz_max, self.yr_max = v_max, vz_max, math.radians(yaw_rate_max_dps)
        # Body-frame half-extents used for collision, covering body + motors (not the props).
        # A uniform r = 0.1 m sphere is far smaller than the airframe (body 0.26 x 0.15 m,
        # motor span 0.47 m), so the drone visibly clipped through walls and obstacles. These
        # half-extents are projected onto the world axes by yaw in _collide() to form a
        # yaw-aligned bounding box.
        self.drone_half_x = 0.15  # forward (body 0.13 + motor 0.165 + motor radius 0.036)
        self.drone_half_z = 0.21  # lateral (motor 0.235 + motor radius 0.036)
        self.drone_half_y = 0.05  # vertical (body is thin; legs / props are narrow enough to ignore)
        self.tau = tau
        self.wind = wind
        self.rng = np.random.default_rng(seed)
        self.cam = camera or RayCamera()
        self.cmd = FlightCommand()
        self.flying = False
        self.t = 0.0
        self.collisions = 0
        self._touching = False
        self._contact_n = None  # unit vector pushing away from the last surface we hit
        self._takeoff_target: float | None = None
        self._landing = False
        self.battery = 100.0

    # ---------------------------------------------------------------- api
    def takeoff(self) -> None:
        self.flying = True
        self._takeoff_target = 0.9
        self._landing = False

    def land(self) -> None:
        self._landing = True

    def emergency_stop(self) -> None:
        self.flying = False
        self.vel[:] = 0
        self.pos[2] = 0

    def send(self, cmd: FlightCommand) -> None:
        self.cmd = cmd

    def telemetry(self) -> Telemetry:
        return Telemetry(t=self.t, alt_m=float(self.pos[2]), vz_mps=float(self.vel[2]), yaw_deg=math.degrees(self.yaw) % 360,
                         yaw_rate_dps=-math.degrees(self.yaw_rate), x_m=float(self.pos[0]), y_m=float(self.pos[1]),
                         speed=float(math.hypot(self.vel[0], self.vel[1])),
                         battery_pct=self.battery, flying=self.flying, touching=self._touching)

    def push_off(self, speed: float = 0.9) -> None:
        """Shove the airframe off whatever it just bumped into, bypassing the slew limit.

        A contact escape that only yaws leaves the drone pressed against the surface: the
        brain keeps commanding forward and the collision solver cancels exactly that much,
        so the position never changes. The physical shove is what actually unsticks it.
        """
        if self._contact_n is None:
            return
        self.vel[:2] += self._contact_n[:2] * float(speed)

    def frame(self) -> np.ndarray:
        return self.cam.render(self.room, self.pos, self.yaw)

    # ---------------------------------------------------------------- physics
    def step(self, dt: float) -> None:
        self.t += dt
        self.battery = max(0.0, self.battery - dt * 100 / 600)  # ~10 min battery
        if not self.flying:
            self.vel[:] = 0
            self.yaw_rate = 0
            return
        c = self.cmd
        if self._takeoff_target is not None:
            vz_t = 0.6 if self.pos[2] < self._takeoff_target else 0.0
            if self.pos[2] >= self._takeoff_target:
                self._takeoff_target = None
            target = np.array([0.0, 0.0, vz_t])
            yr_t = 0.0
        elif self._landing:
            target = np.array([0.0, 0.0, -0.5])
            yr_t = 0.0
        else:
            f = np.array([math.cos(self.yaw), math.sin(self.yaw)])
            r = np.array([math.sin(self.yaw), -math.cos(self.yaw)])
            vxy = (c.forward * f + c.lateral * r) * self.v_max
            target = np.array([vxy[0], vxy[1], c.throttle * self.vz_max])
            yr_t = -c.yaw * self.yr_max  # +yaw command = clockwise, math yaw is counter-clockwise
        k = 1 - math.exp(-dt / self.tau)
        self.vel += (target - self.vel) * k + self.rng.standard_normal(3) * self.wind * math.sqrt(dt)
        self.yaw_rate += (yr_t - self.yaw_rate) * k
        self.yaw += self.yaw_rate * dt
        new = self.pos + self.vel * dt
        new, hit = self._collide(new)  # also sets self._contact_n (the shove-off direction)
        if hit:
            if not self._touching:
                self.collisions += 1
            self.vel *= -0.2
        self._touching = hit
        self.pos = new
        if self._landing and self.pos[2] <= 0.02:
            self.flying = False
            self.pos[2] = 0.0

    def _collide(self, p: np.ndarray) -> tuple[np.ndarray, bool]:
        """Yaw-aligned bounding-box collision against the room and every box.

        The airframe is a box, not a point: the drone yaws in the X-Y plane (forward =
        ``[cos, sin]``, lateral = ``[sin, -cos]``), so the world-space half-extents are the
        sum of each body half-extent weighted by |cos| / |sin| of the yaw. Projecting them
        rotates the collision boundary with the drone, so a head-on wall hit stops the nose
        rather than a 0.1 m sphere at the centre, and a side-on hit stops the motor tips.

        Boxes push the drone out along the *shallowest* axis (minimal translation) instead
        of reverting the step, so a glancing hit keeps its speed along the surface. A few
        passes are enough even for a box tucked into a corner.
        """
        ca, sa = abs(math.cos(self.yaw)), abs(math.sin(self.yaw))
        hwx = self.drone_half_x * ca + self.drone_half_z * sa  # world X half-extent
        hwy = self.drone_half_x * sa + self.drone_half_z * ca  # world Y half-extent
        hwz = self.drone_half_y                                # vertical (no yaw rotation)
        hx, hy, hz = self.room.size_x / 2 - hwx, self.room.size_y / 2 - hwy, self.room.height - hwz
        q = p.copy()
        q[0] = min(hx, max(-hx, q[0]))
        q[1] = min(hy, max(-hy, q[1]))
        q[2] = min(hz, max(0.0, q[2]))
        n = np.zeros(3)
        if p[0] > hx:
            n[0] = -1.0
        elif p[0] < -hx:
            n[0] = 1.0
        if p[1] > hy:
            n[1] = -1.0
        elif p[1] < -hy:
            n[1] = 1.0
        if p[2] > hz:
            n[2] = -1.0  # ceiling only: resting on the floor is not a bump
        hit = bool(np.any(n != 0))
        eps = 1e-4
        for _ in range(3):
            moved = False
            for b in self.room.boxes:
                x0, x1 = b.lo[0] - hwx, b.hi[0] + hwx
                y0, y1 = b.lo[1] - hwy, b.hi[1] + hwy
                z0, z1 = b.lo[2] - hwz, b.hi[2] + hwz
                if not (x0 < q[0] < x1 and y0 < q[1] < y1 and z0 < q[2] < z1):
                    continue
                hit = moved = True
                ways = [(q[0] - x0, 0, -1.0, x0 - eps), (x1 - q[0], 0, 1.0, x1 + eps),
                        (q[1] - y0, 1, -1.0, y0 - eps), (y1 - q[1], 1, 1.0, y1 + eps)]
                if z0 >= 0:
                    ways.append((q[2] - z0, 2, -1.0, z0 - eps))  # dip under (never for floor boxes)
                if z1 <= hz:
                    ways.append((z1 - q[2], 2, 1.0, z1 + eps))  # hop up on top
                axis, sign, dest = min(ways, key=lambda w: w[0])[1:]
                q[axis] = dest
                n[axis] += sign
            q[0] = min(hx, max(-hx, q[0]))
            q[1] = min(hy, max(-hy, q[1]))
            q[2] = min(hz, max(0.0, q[2]))
            if not moved:
                break
        nrm = float(np.linalg.norm(n))
        if nrm > 1e-6:
            n /= nrm
        self._contact_n = n if hit else None
        return q, hit
