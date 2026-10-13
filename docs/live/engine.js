// FlyDrones browser engine: a line-by-line port of the Python package
// (brain/lif.py, senses/retina.py, senses/encoder.py, senses/gestures.py,
//  motor/decoder.py, safety.py, drones/sim.py) so the demo behaves like `flydrones demo`.

export function mulberry32(seed) {
  let a = seed >>> 0;
  return function () {
    a |= 0; a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

// ------------------------------------------------------------------ brain
export class Brain {
  constructor(net, seed = 0) {
    const cfg = net.config;
    const p = cfg.brain.lif;
    this.net = net; this.cfg = cfg; this.p = p;
    this.n = net.n;
    this.indptr = Int32Array.from(net.indptr);
    this.indices = Int32Array.from(net.indices);
    this.w = Float32Array.from(net.data, (x) => x * p.w_syn);
    this.groups = {};
    for (const [k, v] of Object.entries(net.groups)) this.groups[k] = Int32Array.from(v);
    this.D = Math.max(1, Math.round(p.delay / p.dt));
    this.refSteps = Math.max(1, Math.round(p.t_ref / p.dt));
    this.decay = Math.exp(-p.dt / p.tau_syn);
    this.kv = p.dt / p.tau_m;
    this.rng = mulberry32(seed + 1);
    this.bias = new Float32Array(this.n);
    for (const [g, mv] of Object.entries(cfg.brain.bias || {})) for (const i of this.groups[g] || []) this.bias[i] = mv;
    this.inputNames = Object.keys(cfg.inputs);
    this.outputNames = Object.keys(cfg.outputs);
    // neuron class for drawing: 0 hidden, 1 input, 2 output
    this.role = new Uint8Array(this.n);
    for (const k of this.inputNames) for (const i of this.groups[k] || []) this.role[i] = 1;
    for (const k of this.outputNames) for (const i of this.groups[k] || []) this.role[i] = 2;
    this.reset();
  }
  reset() {
    const n = this.n;
    this.v = new Float32Array(n).fill(this.p.v_rest);
    this.g = new Float32Array(n);
    this.ref = new Int32Array(n);
    this.buf = new Float32Array(this.D * n);
    this.rate = new Float32Array(n);
    this.extra = new Float32Array(n); // extra Poisson Hz, e.g. when you click a neuron group
    this.t = 0; this.tMs = 0; this.totalSpikes = 0;
    this.activity = new Float32Array(n); // for glow
  }
  setInputs(rates) {
    this.rate.fill(0);
    for (const [name, arr] of Object.entries(rates)) {
      const idx = this.groups[name];
      if (!idx) continue;
      for (let k = 0; k < idx.length; k++) this.rate[idx[k]] = arr[k];
    }
    for (let i = 0; i < this.n; i++) if (this.extra[i] > 0) this.rate[i] += this.extra[i];
  }
  poke(start, count, hz) { for (let i = start; i < start + count; i++) this.extra[i] = hz; }
  unpoke() { this.extra.fill(0); }
  // run `ms` of brain time; returns {rates: {group: Hz}, spikes: [[tFrac, neuron], ...]}
  tick(inputRates, ms) {
    this.setInputs(inputRates);
    const { n, p, D, indptr, indices, w, v, g, ref, buf, rate, bias } = this;
    const steps = Math.max(1, Math.round(ms / p.dt));
    const counts = new Int32Array(n);
    const spikes = [];
    const pk = p.dt / 1000, kick = p.w_syn * p.poisson_factor;
    for (let s = 0; s < steps; s++) {
      const off = (this.t % D) * n;
      for (let i = 0; i < n; i++) { g[i] += buf[off + i]; buf[off + i] = 0; }
      for (let i = 0; i < n; i++) if (rate[i] > 0 && this.rng() < rate[i] * pk) g[i] += kick;
      for (let i = 0; i < n; i++) {
        if (ref[i] > 0) { v[i] = p.v_reset; ref[i]--; continue; }
        v[i] += this.kv * (p.v_rest - v[i] + g[i] + bias[i]);
        g[i] *= this.decay;
        if (v[i] > p.v_th) {
          v[i] = p.v_reset; g[i] = 0; ref[i] = this.refSteps;
          counts[i]++; spikes.push([s / steps, i]);
          for (let k = indptr[i]; k < indptr[i + 1]; k++) buf[off + indices[k]] += w[k];
        }
      }
      this.t++; this.tMs += p.dt;
    }
    const out = {};
    for (const name of [...this.outputNames, ...this.inputNames]) {
      const idx = this.groups[name];
      let c = 0;
      for (const i of idx) c += counts[i];
      out[name] = idx.length ? (c / idx.length) * 1000 / ms : 0;
    }
    this.totalSpikes += spikes.length;
    return { rates: out, spikes, counts };
  }
}

// ------------------------------------------------------------------ room + camera
// Foam-block obstacles are dropped into the arena at random (seeded RNG, so the
// simulator, the retina view and the 3D visuals always agree on the same layout).
export const OBSTACLE_COUNT = 8; // how many boxes the arena holds (tune me)
// Footprint is a fixed 0.5 m x 0.5 m (one floor mat tile); the lo corner snaps to the
// 0.5 m mat grid, only the height varies (1.1..2.0 m).
export function makeObstacles(seed = (Math.random() * 0xffffffff) >>> 0, count = OBSTACLE_COUNT) {
  const rng = mulberry32(seed);
  const starts = [[-1.5, 0], [-0.9, 0.05], [-1.9, -0.3]]; // pilot start (x, y): gestures / chair / game
  const boxes = [];
  let guard = 0;
  while (boxes.length < count && guard++ < 5000) {
    // lo ∈ [-2.0, 1.5], so a 0.5 m box stays ≥1 m from the ±3 m walls
    const lo = [-2.0 + Math.floor(rng() * 8) * 0.5, -2.0 + Math.floor(rng() * 8) * 0.5];
    const hi = [lo[0] + 0.5, lo[1] + 0.5];
    const h = 1.1 + rng() * 0.9;
    // keep a launch pad clear around every pilot start position
    if (starts.some(([sx, sy]) => sx > lo[0] - 0.9 && sx < hi[0] + 0.9 && sy > lo[1] - 0.9 && sy < hi[1] + 0.9)) continue;
    // keep obstacles apart so the drone can slip between them
    if (boxes.some((b) => lo[0] < b.hi[0] + 0.8 && hi[0] > b.lo[0] - 0.8 && lo[1] < b.hi[1] + 0.8 && hi[1] > b.lo[1] - 0.8)) continue;
    boxes.push({ lo: [lo[0], lo[1], 0], hi: [hi[0], hi[1], h], shade: 0.16 + rng() * 0.14, name: `obstacle${boxes.length + 1}` });
  }
  return boxes;
}

export const ROOM = {
  sx: 6, sy: 6, h: 2.7,
  // ?seed=<n> pins the obstacle layout, so a run is reproducible (headless checks, bug reports)
  boxes: makeObstacles(typeof location !== "undefined" ? Number(new URLSearchParams(location.search).get("seed")) || undefined : undefined),
};

export class RayCamera {
  constructor(w = 96, h = 72, fovDeg = 82) {
    this.w = w; this.h = h;
    const tx = Math.tan((fovDeg * Math.PI) / 360), ty = (tx * h) / w;
    this.U = new Float32Array(w * h); this.V = new Float32Array(w * h);
    for (let j = 0; j < h; j++) for (let i = 0; i < w; i++) {
      this.U[j * w + i] = (((i + 0.5) / w) * 2 - 1) * tx;
      this.V[j * w + i] = (1 - ((j + 0.5) / h) * 2) * ty;
    }
  }
  // returns Float32Array gray 0..1 (row major)
  render(room, pos, yaw) {
    const { w, h, U, V } = this;
    const out = new Float32Array(w * h);
    const fx = Math.cos(yaw), fy = Math.sin(yaw), rx = Math.sin(yaw), ry = -Math.cos(yaw);
    const [px, py, pz] = pos;
    const hx = room.sx / 2, hy = room.sy / 2;
    const INF = 1e9;
    for (let k = 0; k < w * h; k++) {
      let dx = fx + U[k] * rx, dy = fy + U[k] * ry, dz = V[k];
      const nrm = Math.hypot(dx, dy, dz); dx /= nrm; dy /= nrm; dz /= nrm;
      let tb = INF, shade = 0;
      const wall = (t, along, zz) => {
        if (t > 1e-6 && t < tb) {
          const stripe = (Math.floor(along / 0.3) % 2 + 2) % 2 * 0.35 + 0.35;
          const band = ((Math.floor(zz / 0.8) % 2) + 2) % 2 === 0 ? 1.0 : 0.8;
          tb = t; shade = stripe * band;
        }
      };
      if (dx !== 0) { let t = (hx - px) / dx; wall(t, py + t * dy, pz + t * dz); t = (-hx - px) / dx; wall(t, py + t * dy, pz + t * dz); }
      if (dy !== 0) { let t = (hy - py) / dy; wall(t, px + t * dx, pz + t * dz); t = (-hy - py) / dy; wall(t, px + t * dx, pz + t * dz); }
      if (dz !== 0) {
        let t = -pz / dz;
        if (t > 1e-6 && t < tb) {
          // foam-mat floor: 0.5 m checker tiles, like the 3D arena
          const cx = Math.floor((px + t * dx) / 0.5) + Math.floor((py + t * dy) / 0.5);
          tb = t; shade = ((cx % 2) + 2) % 2 * 0.35 + 0.25;
        }
        t = (room.h - pz) / dz;
        if (t > 1e-6 && t < tb) { tb = t; shade = 0.9; }
      }
      for (const b of room.boxes) {
        const t1 = (b.lo[0] - px) / dx, t2 = (b.hi[0] - px) / dx;
        const t3 = (b.lo[1] - py) / dy, t4 = (b.hi[1] - py) / dy;
        const t5 = (b.lo[2] - pz) / dz, t6 = (b.hi[2] - pz) / dz;
        const tmin = Math.max(Math.min(t1, t2), Math.min(t3, t4), Math.min(t5, t6));
        const tmax = Math.min(Math.max(t1, t2), Math.max(t3, t4), Math.max(t5, t6));
        if (tmax >= tmin && tmin > 1e-6 && tmin < tb) {
          const hx_ = px + tmin * dx, hy_ = py + tmin * dy, hz_ = pz + tmin * dz;
          // moving objects carry their texture with them (object coordinates), static ones use world coordinates like the Python sim
          const ox = b.moving ? b.lo[0] : 0, oy = b.moving ? b.lo[1] : 0, oz = b.moving ? b.lo[2] : 0;
          const chk = ((Math.floor((hx_ - ox + hy_ - oy) / 0.12) + Math.floor((hz_ - oz) / 0.12)) % 2 + 2) % 2;
          tb = tmin; shade = b.shade + chk * 0.22;
        }
      }
      shade = shade / (1 + 0.06 * Math.min(tb, 50));
      out[k] = Math.floor(clamp(shade * 255, 0, 255)) / 255;
    }
    return out;
  }
}

// ------------------------------------------------------------------ drone physics
export class SimDrone {
  constructor(start = [-1.5, 0, 0], yawDeg = 0, seed = 0) {
    this.room = { ...ROOM, boxes: ROOM.boxes.map((b) => ({ ...b })) };
    this.pos = [...start]; this.vel = [0, 0, 0];
    this.yaw = (yawDeg * Math.PI) / 180; this.yawRate = 0;
    this.vMax = 1.0; this.vzMax = 0.8; this.yrMax = (120 * Math.PI) / 180; this.tau = 0.35; this.wind = 0.03;
    this.bounce = 0.4; // restitution on a wall / obstacle bump (0 = pure slide, 1 = full rebound)
    // Body-frame half-extents for collision (covers body + motors, not props). The old code used a
    // uniform r = 0.1 m which is far smaller than the 3D model (body 0.26×0.15 m, motor span 0.47 m),
    // so the drone visibly clipped through walls and obstacles. These half-extents are projected onto
    // the world axes by yaw in collide() to form a yaw-aligned bounding box.
    this.droneHalfX = 0.15; // forward (covers body 0.13 + motor 0.165 + motor radius 0.036)
    this.droneHalfZ = 0.21; // lateral (covers motor 0.235 + motor radius 0.036)
    this.droneHalfY = 0.05; // vertical (body is thin; legs/props are narrow enough to ignore)
    this.rng = mulberry32(seed + 99);
    this.cam = new RayCamera();
    this.cmd = { throttle: 0, yaw: 0, forward: 0, lateral: 0 };
    this.flying = false; this.t = 0; this.collisions = 0; this.touching = false;
    this.contactN = null; // last contact normal (points away from the surface, into free space)
    this.takeoffTarget = null; this.landing = false; this.battery = 100;
  }
  gauss() { const u = 1 - this.rng(), v = this.rng(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }
  takeoff() { this.flying = true; this.takeoffTarget = 0.5; this.landing = false; }
  // browser game only: a giant-fiber "takeoff jump" = short velocity burst up and back
  jump(up = 1.6, back = 0.8) {
    if (!this.flying || this.takeoffTarget !== null) return;
    this.vel[0] -= Math.cos(this.yaw) * back; this.vel[1] -= Math.sin(this.yaw) * back; this.vel[2] += up;
  }
  land() { this.landing = true; }
  // Physical separation, used by the *contact* escape. A drone pressed against a surface is held there
  // by its own cruise: the brain keeps commanding "forward" and collide() cancels exactly that inward
  // component, so the position never changes — it hangs on the wall. Turning does not help either,
  // because yaw on its own does not move the drone. The only thing that unsticks it is a velocity
  // that points away from the surface, and the command path cannot deliver one quickly: Safety.slew_per_s
  // (2.5/s) turns a cruise of 1.2 into a reverse of -0.6 only after ~0.7 s, which is longer than the
  // escape itself. So the shove is applied straight to the velocity, like the game's jump().
  pushOff(speed = 0.9) {
    const n = this.contactN;
    if (!this.flying || !n) return false;
    this.vel[0] += n[0] * speed; this.vel[1] += n[1] * speed;
    if (n[2] > 0) this.vel[2] += n[2] * speed; // n[2] is 0 for walls/boxes, -1 under the ceiling
    return true;
  }
  send(cmd) { this.cmd = cmd; }
  telemetry() {
    return { t: this.t, alt: this.pos[2], x: this.pos[0], y: this.pos[1], yawDeg: ((this.yaw * 180) / Math.PI % 360 + 360) % 360,
      yawRateDps: (-this.yawRate * 180) / Math.PI, speed: Math.hypot(this.vel[0], this.vel[1]),
      battery: this.battery, flying: this.flying, touching: this.touching };
  }
  frame() { return this.cam.render(this.room, this.pos, this.yaw); }
  step(dt) {
    this.t += dt;
    this.battery = Math.max(0, this.battery - (dt * 100) / 600);
    if (!this.flying) { this.vel = [0, 0, 0]; this.yawRate = 0; return; }
    const c = this.cmd;
    let target, yrT;
    if (this.takeoffTarget !== null) {
      const vz = this.pos[2] < this.takeoffTarget ? 0.6 : 0;
      if (this.pos[2] >= this.takeoffTarget) this.takeoffTarget = null;
      target = [0, 0, vz]; yrT = 0;
    } else if (this.landing) { target = [0, 0, -0.5]; yrT = 0; }
    else {
      const f = [Math.cos(this.yaw), Math.sin(this.yaw)], r = [Math.sin(this.yaw), -Math.cos(this.yaw)];
      target = [(c.forward * f[0] + c.lateral * r[0]) * this.vMax, (c.forward * f[1] + c.lateral * r[1]) * this.vMax, c.throttle * this.vzMax];
      yrT = -c.yaw * this.yrMax;
    }
    const k = 1 - Math.exp(-dt / this.tau);
    for (let i = 0; i < 3; i++) this.vel[i] += (target[i] - this.vel[i]) * k + this.gauss() * this.wind * Math.sqrt(dt);
    this.yawRate += (yrT - this.yawRate) * k;
    this.yaw += this.yawRate * dt;
    const nw = this.pos.map((p, i) => p + this.vel[i] * dt);
    const [q, hit, n] = this.collide(nw);
    if (hit) {
      if (!this.touching) this.collisions++;
      // Cancel only the velocity component that points INTO the surface and keep the tangential
      // part, so the drone slides along a wall / obstacle instead of gluing itself to it. (The old
      // code snapped the whole position back and reversed the whole velocity, which left the drone
      // pinned against a surface for up to 20 s at a time.)
      const vn = this.vel[0] * n[0] + this.vel[1] * n[1] + this.vel[2] * n[2];
      if (vn < 0) for (let i = 0; i < 3; i++) this.vel[i] -= vn * n[i] * (1 + this.bounce);
      this.contactN = [n[0], n[1], n[2]];
    }
    this.touching = hit;
    this.pos = q;
    if (this.landing && this.pos[2] <= 0.02) { this.flying = false; this.pos[2] = 0; }
  }
  collide(p) {
    const room = this.room;
    // Project the drone's body-frame OBB onto world axes. The drone yaws in the X-Y plane: forward =
    // [cos(yaw), sin(yaw)], lateral = [sin(yaw), -cos(yaw)]. The world-space AABB half-extents are the
    // sum of each body half-extent weighted by |cos| / |sin| of the yaw. This rotates the collision
    // boundary with the drone, so a head-on wall hit stops the nose (not just a 0.1 m sphere at the
    // centre) and a side-on hit stops the motor tips.
    const ca = Math.abs(Math.cos(this.yaw)), sa = Math.abs(Math.sin(this.yaw));
    const hWX = this.droneHalfX * ca + this.droneHalfZ * sa; // world X half-extent
    const hWY = this.droneHalfX * sa + this.droneHalfZ * ca; // world Y half-extent
    const hWZ = this.droneHalfY;                             // vertical (no yaw rotation)
    const hx = room.sx / 2 - hWX, hy = room.sy / 2 - hWY, hz = room.h - hWZ;
    const q = [clamp(p[0], -hx, hx), clamp(p[1], -hy, hy), clamp(p[2], 0, hz)];
    const n = [0, 0, 0];
    if (p[0] > hx) n[0] = -1; else if (p[0] < -hx) n[0] = 1;
    if (p[1] > hy) n[1] = -1; else if (p[1] < -hy) n[1] = 1;
    if (p[2] > hz) n[2] = -1; // ceiling only: resting on the floor is not a bump
    let hit = n[0] !== 0 || n[1] !== 0 || n[2] !== 0;
    // Push out of every obstacle along its shallowest axis (minimal translation) instead of
    // reverting the step, then re-clamp to the room. Three passes are enough even for a box that
    // sits in a corner; the push-out is at most one sub-step deep, so it is always along the
    // direction of travel and the drone keeps its speed along the surface.
    for (let pass = 0; pass < 3; pass++) {
      let moved = false;
      for (const b of room.boxes) {
        if (b.solid === false) continue;
        const x0 = b.lo[0] - hWX, x1 = b.hi[0] + hWX, y0 = b.lo[1] - hWY, y1 = b.hi[1] + hWY, z0 = b.lo[2] - hWZ, z1 = b.hi[2] + hWZ;
        if (!(q[0] > x0 && q[0] < x1 && q[1] > y0 && q[1] < y1 && q[2] > z0 && q[2] < z1)) continue;
        hit = moved = true;
        const eps = 1e-4;
        const ways = [
          [q[0] - x0, 0, -1, x0 - eps], [x1 - q[0], 0, 1, x1 + eps],
          [q[1] - y0, 1, -1, y0 - eps], [y1 - q[1], 1, 1, y1 + eps],
        ];
        if (z0 >= 0) ways.push([q[2] - z0, 2, -1, z0 - eps]); // dip under (never for floor boxes)
        if (z1 <= hz) ways.push([z1 - q[2], 2, 1, z1 + eps]);  // hop up on top
        ways.sort((a, c) => a[0] - c[0]);
        const w = ways[0];
        q[w[1]] = w[3]; n[w[1]] += w[2];
      }
      q[0] = clamp(q[0], -hx, hx); q[1] = clamp(q[1], -hy, hy); q[2] = clamp(q[2], 0, hz);
      if (!moved) break;
    }
    const len = Math.hypot(n[0], n[1], n[2]);
    if (len > 1e-6) { n[0] /= len; n[1] /= len; n[2] /= len; }
    return [q, hit, n];
  }
}

// ------------------------------------------------------------------ retina
function weightedSlope(v, w, rows, cols, c0, c1, axis) {
  let sw = 0, cm = 0, vm = 0;
  for (let r = 0; r < rows; r++) for (let c = c0; c < c1; c++) {
    const k = r * cols + c, x = axis === "x" ? c - c0 - (c1 - c0 - 1) / 2 : r - (rows - 1) / 2;
    sw += w[k]; cm += w[k] * x; vm += w[k] * v[k];
  }
  sw += 1e-6; cm /= sw; vm /= sw;
  let cov = 0, vr = 0;
  for (let r = 0; r < rows; r++) for (let c = c0; c < c1; c++) {
    const k = r * cols + c, x = axis === "x" ? c - c0 - (c1 - c0 - 1) / 2 : r - (rows - 1) / 2;
    cov += w[k] * (x - cm) * (v[k] - vm); vr += w[k] * (x - cm) ** 2;
  }
  return cov / (vr + 1e-6);
}

export class Retina {
  constructor(cfg) {
    const v = cfg.vision;
    this.w = v.width; this.h = v.height; this.rows = v.grid[0]; this.cols = v.grid[1];
    this.flowGain = v.flow_gain; this.loomGain = v.loom_gain; this.loomFloor = v.loom_floor; this.blurR = v.blur;
    // Fraction of the field width the looming trigger reads (1.0 = full width, 0.5 = central 50 %).
    // Peripheral flow (walls sliding by during cruise) inflates the full-width expansion slope and
    // fires the reflex too early; reading only the centre cuts those false triggers without affecting
    // the escape direction, which still reads the full half-eye (nearL / nearR in escFromEye).
    this.loomFovFrac = Number.isFinite(v.loom_fov_frac) ? clamp(v.loom_fov_frac, 0.1, 1.0) : 1.0;
    this.prev = null; this.prevExp = { L: 0, R: 0 }; this.loomEma = null;
    this.nearL = null; this.nearR = null; this.nearAll = null; // depth-rate per half, for the escape direction
  }
  reset() { this.prev = null; this.loomEma = null; this.nearL = null; this.nearR = null; this.nearAll = null; }
  blur(img) {
    const { w, h } = this, r = this.blurR;
    if (r <= 0) return img;
    const out = new Float32Array(w * h), k = 2 * r + 1;
    for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
      let s = 0;
      for (let dy = -r; dy <= r; dy++) for (let dx = -r; dx <= r; dx++) {
        s += img[clamp(y + dy, 0, h - 1) * w + clamp(x + dx, 0, w - 1)];
      }
      out[y * w + x] = s / (k * k);
    }
    return out;
  }
  encode(gray) {
    const { w, h, rows, cols } = this;
    const img = this.blur(gray);
    const prev = this.prev || img;
    this.prev = img;
    const ncols = cols * 2, rh = Math.floor(h / rows), cw = Math.floor(w / ncols), nc = rows * ncols;
    const sxx = new Float64Array(nc), syy = new Float64Array(nc), sxy = new Float64Array(nc), sxt = new Float64Array(nc), syt = new Float64Array(nc), br = new Float64Array(nc);
    const avg = (x, y) => (img[y * w + x] + prev[y * w + x]) * 0.5;
    for (let y = 0; y < rh * rows; y++) for (let x = 0; x < cw * ncols; x++) {
      const ix = x === 0 ? avg(1, y) - avg(0, y) : x === w - 1 ? avg(x, y) - avg(x - 1, y) : (avg(x + 1, y) - avg(x - 1, y)) / 2;
      const iy = y === 0 ? avg(x, 1) - avg(x, 0) : y === h - 1 ? avg(x, y) - avg(x, y - 1) : (avg(x, y + 1) - avg(x, y - 1)) / 2;
      const it = img[y * w + x] - prev[y * w + x];
      const k = Math.floor(y / rh) * ncols + Math.floor(x / cw);
      sxx[k] += ix * ix; syy[k] += iy * iy; sxy[k] += ix * iy; sxt[k] += ix * it; syt[k] += iy * it; br[k] += img[y * w + x];
    }
    const vx = new Float32Array(nc), vy = new Float32Array(nc), tex = new Float32Array(nc), energy = new Float64Array(nc);
    for (let k = 0; k < nc; k++) {
      const a = sxx[k] + 1e-4, d = syy[k] + 1e-4, det = a * d - sxy[k] ** 2;
      vx[k] = (-d * sxt[k] + sxy[k] * syt[k]) / det;
      vy[k] = (sxy[k] * sxt[k] - a * syt[k]) / det;
      energy[k] = a + d; br[k] /= rh * cw;
    }
    const sorted = Array.from(energy).sort((p, q) => p - q);
    const med = nc % 2 ? sorted[(nc - 1) / 2] : (sorted[nc / 2 - 1] + sorted[nc / 2]) / 2;
    const vxr = Float32Array.from(vx), vyr = Float32Array.from(vy); // raw flow, before clamp/tex
    for (let k = 0; k < nc; k++) {
      tex[k] = clamp(energy[k] / (med + 1e-6), 0, 1);
      vx[k] = clamp(vx[k], -3, 3) * tex[k]; vy[k] = clamp(vy[k], -3, 3) * tex[k];
    }
    const expansion = (c0, c1) => Math.min(weightedSlope(vx, tex, rows, ncols, c0, c1, "x"), weightedSlope(vy, tex, rows, ncols, c0, c1, "y"));
    // expAll reads only the central loomFovFrac of the field (e.g. 50 %) so peripheral flow does not
    // inflate the expansion slope. expL / expR stay full-width — they feed the L/R split, not the trigger.
    const _fovMargin = Math.round(ncols * (1 - this.loomFovFrac) / 2);
    const expAll = expansion(_fovMargin, ncols - _fovMargin), expL = expansion(0, cols), expR = expansion(cols, ncols);
    // expL / expR split the looming across the two eyes (which eye gets the reflex), but they cannot
    // say *where* the looming object is: they are affine slopes fitted over a whole half of the eye,
    // and for any smooth expansion field those two slopes come out nearly equal — verified against the
    // real obstacle bearings, expL vs expR matched the true side only ~half the time.
    // The escape direction therefore reads a second pair, from the *divergence* of the flow field:
    // local expansion = 2 * Vz / Z, i.e. it is large exactly where a surface is close. Averaged over a
    // half of the eye (texture-weighted) it answers "which half of my world is about to hit me".
    // The common background cancels out of the comparison: the floor and the far walls sit in both
    // halves, so they lift both means together and only the near object tilts the balance. (A per-half
    // *maximum* was tried first and failed for that reason: the max is always the floor, in both
    // halves, so every loom looked head-on.)
    this.expL = expL; this.expR = expR;
    const px = w / ncols, py = h / rows; // cell size in pixels, so both derivatives share a scale
    let sl = 0, wl = 0, sr = 0, wr = 0, sAll = 0, wAll = 0;
    for (let r = 0; r < rows; r++) for (let c = 0; c < ncols; c++) {
      const k = r * ncols + c;
      const kx1 = r * ncols + Math.min(ncols - 1, c + 1), kx0 = r * ncols + Math.max(0, c - 1);
      const ky1 = Math.min(rows - 1, r + 1) * ncols + c, ky0 = Math.max(0, r - 1) * ncols + c;
      const div = (vxr[kx1] - vxr[kx0]) / (2 * px) + (vyr[ky1] - vyr[ky0]) / (2 * py);
      if (!Number.isFinite(div)) continue;
      const wt = tex[k] + 1e-3;
      if (c < cols) { sl += wt * div; wl += wt; } else { sr += wt * div; wr += wt; }
      sAll += wt * div; wAll += wt;
    }
    const dL = wl > 0 ? sl / wl : 0, dR = wr > 0 ? sr / wr : 0, dAll = wAll > 0 ? sAll / wAll : 0;
    // lightly smoothed: the escape pick is made once per escape, off the peak of the looming
    this.nearL = this.nearL === null ? dL : 0.4 * this.nearL + 0.6 * dL;
    this.nearR = this.nearR === null ? dR : 0.4 * this.nearR + 0.6 * dR;
    this.nearAll = this.nearAll === null ? dAll : 0.4 * this.nearAll + 0.6 * dAll;
    this.loomEma = this.loomEma === null ? expAll : 0.5 * this.loomEma + 0.5 * expAll;
    const level = clamp((this.loomEma - this.loomFloor) * this.loomGain, 0, 1);
    const sideL = clamp(0.5 + (expL - expR) * 2, 0, 1);
    const loomBy = { L: level * Math.min(1, 2 * sideL), R: level * Math.min(1, 2 * (1 - sideL)) };
    const g = this.flowGain, eyes = {};
    for (const [eye, c0] of [["L", 0], ["R", cols]]) {
      const n = rows * cols, sign = eye === "L" ? -1 : 1;
      const grids = { brightness: new Float32Array(n), ftb: new Float32Array(n), btf: new Float32Array(n), up: new Float32Array(n), down: new Float32Array(n), loom: new Float32Array(n), loom_speed: new Float32Array(n) };
      const lv = loomBy[eye], ls = clamp((lv - this.prevExp[eye]) * 3, 0, 1);
      for (let r = 0; r < rows; r++) for (let c = 0; c < cols; c++) {
        const k = r * ncols + c0 + c, e = r * cols + c;
        grids.brightness[e] = br[k];
        grids.ftb[e] = clamp((sign * vx[k]) / g, 0, 1); grids.btf[e] = clamp((-sign * vx[k]) / g, 0, 1);
        grids.up[e] = clamp(-vy[k] / g, 0, 1); grids.down[e] = clamp(vy[k] / g, 0, 1);
        grids.loom[e] = lv; grids.loom_speed[e] = ls;
      }
      this.prevExp[eye] = 0.7 * this.prevExp[eye] + 0.3 * lv;
      eyes[eye] = grids;
    }
    return { eyes, vx, vy, tex, loomLevel: level, expL, expR, nearL: this.nearL, nearR: this.nearR, nearAll: this.nearAll, ncols, rows };
  }
  blank() {
    const n = this.rows * this.cols, z = () => new Float32Array(n);
    const e = () => ({ brightness: z(), ftb: z(), btf: z(), up: z(), down: z(), loom: z(), loom_speed: z() });
    return { eyes: { L: e(), R: e() }, vx: new Float32Array(n * 2), vy: new Float32Array(n * 2), tex: new Float32Array(n * 2), loomLevel: 0, expL: 0, expR: 0, nearL: 0, nearR: 0, nearAll: 0, ncols: this.cols * 2, rows: this.rows };
  }
}

// ------------------------------------------------------------------ encoder
export class Encoder {
  constructor(brain) { this.brain = brain; this.specs = brain.cfg.inputs; this.yawSat = (brain.cfg.imu || {}).yaw_saturation_dps || 150; }
  encode(vision, yawRateDps) {
    const out = {};
    for (const [name, spec] of Object.entries(this.specs)) {
      const idx = this.brain.groups[name];
      if (!idx || !idx.length) continue;
      const arr = new Float32Array(idx.length), max = spec.max_hz || 100;
      if (spec.feature === "yaw_pos" || spec.feature === "yaw_neg") {
        const val = spec.feature === "yaw_pos" ? yawRateDps : -yawRateDps;
        arr.fill(max * clamp(val / this.yawSat, 0, 1));
      } else {
        const grid = vision.eyes[spec.eye || "L"][spec.feature];
        if (!grid) continue;
        for (let k = 0; k < idx.length; k++) arr[k] = max * grid[Math.floor((k * grid.length) / idx.length)];
      }
      out[name] = arr;
    }
    return out;
  }
}

// ------------------------------------------------------------------ gestures
export const DEMO_TIMELINE = [
  [0.0, { present: false, openness: 0, x: 0, y: 0, size: 0, label: "no hand" }],
  [2.5, { present: true, openness: 0.95, x: 0, y: 0, size: 0.2, label: "open palm" }],
  [4.5, { present: true, openness: 0.1, x: 0, y: 0, size: 0.15, label: "fist" }],
  [9.5, { present: true, openness: 0.1, x: 0.85, y: 0, size: 0.15, label: "fist, moved right" }],
  [12.0, { present: true, openness: 0.1, x: 0, y: 0, size: 0.15, label: "fist" }],
  [13.5, { present: true, openness: 0.1, x: 0, y: 0, size: 0.55, label: "hand rushes at camera" }],
  [14.0, { present: true, openness: 0.1, x: 0, y: 0, size: 0.55, label: "fist, close" }],
  [15.5, { present: false, openness: 0, x: 0, y: 1, size: 0, label: "hand dropped" }],
];

export class GestureIllusion {
  constructor(strength = 0.5, dropHold = 30, loomGrowth = 0.03) {
    this.s = strength; this.dropHold = dropHold; this.loomGrowth = loomGrowth;
    this.lastPresent = null; this.prevSize = 0; this.loom = 0; this.mode = "no hand"; this.channel = "";
  }
  apply(vision, gst, t) {
    const s = this.s;
    let up = 0, down = 0, rot = 0;
    if (gst.present) {
      if (this.lastPresent === null || t - this.lastPresent > 0.5) this.prevSize = gst.size;
      this.lastPresent = t;
      if (gst.openness > 0.6) { up = (s * (gst.openness - 0.6)) / 0.4; this.mode = "open palm -> sinking illusion"; this.channel = "scene drifts up -> T4c -> VS -> DNg02 up -> climb"; }
      else { this.mode = "fist -> still scene"; this.channel = "no motion -> DNg02 at rest -> hold"; }
      if (Math.abs(gst.x) > 0.35) { rot = (s * Math.sign(gst.x) * (Math.abs(gst.x) - 0.35)) / 0.65; this.mode += " + rotate"; this.channel = `scene rotates ${rot > 0 ? "right" : "left"} -> HS -> DNg02 ${rot > 0 ? "R>L" : "L>R"} -> yaw`; }
      const growth = gst.size - this.prevSize;
      if (growth > this.loomGrowth) this.loom = Math.max(this.loom, Math.min(1, growth / (4 * this.loomGrowth)));
      this.prevSize = gst.size;
    } else if (this.lastPresent !== null && t - this.lastPresent < this.dropHold) {
      down = Math.min(1, s * 1.4); this.mode = "hand dropped -> rising illusion"; this.channel = "scene drifts down -> T4d -> LPi -| DNg02 -> descend"; this.prevSize = 0;
    } else { this.mode = "no hand"; this.channel = "camera optic flow only"; }
    this.loom *= 0.85;
    const loom = this.loom > 0.05 ? this.loom : 0;
    if (loom) { this.mode = "hand rushing in -> looming"; this.channel = "expansion -> LPLC2 + LC4 -> giant fiber DNp01"; }
    for (const eye of ["L", "R"]) {
      const gr = vision.eyes[eye];
      const key = rot > 0 ? (eye === "L" ? "btf" : "ftb") : eye === "L" ? "ftb" : "btf";
      for (let k = 0; k < gr.up.length; k++) {
        gr.up[k] = clamp(gr.up[k] + up, 0, 1); gr.down[k] = clamp(gr.down[k] + down, 0, 1);
        gr[key][k] = clamp(gr[key][k] + Math.abs(rot), 0, 1);
        gr.loom[k] = clamp(gr.loom[k] + loom, 0, 1); gr.loom_speed[k] = clamp(gr.loom_speed[k] + loom, 0, 1);
      }
    }
    return vision;
  }
}

// ------------------------------------------------------------------ decoder
export class Decoder {
  constructor(cfg) {
    const d = cfg.decoder;
    this.smoothing = d.smoothing; this.deadzone = d.deadzone; this.settle = d.settle_s;
    this.axes = d.axes; this.cruise = d.cruise; this.brakeTerms = d.brake_terms;
    this.cruiseFloor = 0; // min fraction of cruise kept while the looming brake is on (see update())
    this.esc = d.escape;
    // Speed-adaptive looming threshold: the looming signal (expansion rate) is proportional to V/Z,
    // so a fixed threshold triggers at very different distances depending on speed. Scaling the
    // threshold by speed/vRef makes the trigger distance roughly constant across speeds.
    //   speed > vRef  -> threshold rises  -> triggers later (closer to the obstacle)
    //   speed < vRef  -> threshold drops  -> triggers earlier (not too late at low speed)
    // Clamped to [0.3, 3.0] × vRef so a standstill does not fire on noise and a burst does not
    // make the reflex unreachable. vRef ≈ cruise speed (≈ 0.5 m/s in autoplay).
    this.escVRef = (d.escape && Number.isFinite(d.escape.v_ref)) ? d.escape.v_ref : 0.8;
    this.baseline = null; this.sum = {}; this.nSettle = 0; this.elapsed = 0;
    this.state = { throttle: 0, yaw: 0, forward: 0, lateral: 0 };
    this.escUntil = -1; this.gf = 0; this.brake = 0; this.escapes = 0; this.escPick = null; this.escCause = "loom";
    // Escape direction. It is NOT a dice roll any more: it is read off the eye (escFromEye), so the
    // same visual situation always produces the same reflex —
    //   left half swelling harder  -> turn RIGHT (away from it)
    //   right half swelling harder -> turn LEFT
    //   both halves swelling alike -> the thing is coming head-on, there is no side to pick, so the
    //                                 escape becomes a 180° about-face (escTurn180Hold seconds of full
    //                                 yaw) — the drone turns around and flies back the way it came.
    //                                 A head-on loom used to trigger a climb, but climbing on every
    //                                 head-on turned the flight into a yo-yo and gained altitude that
    //                                 the governor then had to undo. Turning 180° is a pure dodge with
    //                                 no altitude side-effect.
    // escP only survives as the fallback for when there is nothing to read at all — a bump against a
    // surface gives a *static* image, hence zero expansion, and a random turn off the wall is what
    // unsticks the drone. escSideMin is a depth-rate floor (static image detector);
    // escSideCloseFrac makes "the two sides are alike" scale with how hard the looming is.
    this.escP = { climb: 0.10, left: 0.45 }; this.escCeilAlt = null;
    this.escSideMin = 0.01;  // mean depth-rate below this = the image is not moving = no cue at all
    this.escSideCloseFrac = 0.10; // |left - right| below 10 % of (left + right) = "both sides alike"
    this.escClimbHold = 1.5;  // s, how long a no-cue fallback climb is held (it keeps cruising)
    this.escClimbRetrig = 0.3; // s, shortest gap between two looming picks inside one climb
    this.escRetrigUntil = 0;  // s, when the running climb may be interrupted by a new looming
    this.escRetrig = 0;       // how many escapes were re-picked inside a running climb
    // 180° about-face: hold full yaw until the drone has swung π rad. SimDrone yaw is a first-order
    // lag (tau = 0.35 s) on yrMax, so the hold time ≈ π / yrMax + tau. The engine default yrMax is
    // 120 °/s (≈ 2.094 rad/s) → 1.85 s; applyAutoTune overrides this for 180 °/s → 1.35 s.
    this.escTurn180Hold = 1.85;  // s, duration of the 180° about-face at the default yaw rate
    this.escTurn180Dir = 1;      // yaw sign for the current about-face (+1 = right, -1 = left)
    this.sideEma = 0; this.totEma = 0;
    // which reflex each escape turned into (readable from the page via FD.state().escHist) + the
    // eye reading that picked it, so a run can be audited instead of guessed at.
    this.escHist = { left: 0, right: 0, climb: 0, turn180: 0, nocue: 0 };
    this.escEye = null;
    this.contactUntil = -1;
    // Retry gap while the drone is *still* on a surface. It used to be 2 s, which is a long time to
    // hang there if the first attempt happens to turn the nose into a neighbour (a box next to a wall,
    // a corner): the escape fires once, fails, and nothing else may happen for two seconds. The escape
    // itself now backs the drone off (escBack / pendingPush), so a shorter gap is safe.
    this.contactCooldown = (d.escape && d.escape.contact_s) || 0.9;
    this.escBack = 0.9;   // m/s shove away from the surface on a contact escape (see SimDrone.pushOff)
    this.pendingPush = 0; // set when a contact escape fires, consumed by Pilot.tick()
  }
  // Direction for the escape that is about to fire, from the eye rather than from dice.
  // nearL / nearR come from Retina.encode(): the mean depth-rate (flow divergence) seen by the left /
  // right half of the eye, i.e. how close the closest surface on that side is. Image column 0 is the
  // drone's left (RayCamera: dx = forward + U * right, U = -1 at column 0), so the half that is being
  // rushed at swells harder and the fly turns away from it. Both halves swelling alike means the
  // object is coming straight at it — no side to pick, so the escape becomes a 180° about-face
  // (escTurn180Hold seconds of full yaw, direction from the marginal sign of sideEma or random).
  // Returns "left" | "right" | "turn180" | null (null = no visual information, use the escP mix).
  escFromEye(v) {
    const L = v && Number.isFinite(v.nearL) ? v.nearL : 0, R = v && Number.isFinite(v.nearR) ? v.nearR : 0;
    // Short EMAs: the per-frame divergence figures are noisy and the pick happens only once per escape.
    this.sideEma = 0.5 * this.sideEma + 0.5 * (L - R);
    this.totEma = 0.5 * this.totEma + 0.5 * (L + R);
    const flow = v && Number.isFinite(v.nearAll) ? Math.abs(v.nearAll) : 0;
    // A bump gives a *static* image: no expansion anywhere, hence nothing to read a side off, hence
    // the escP mix (with its random turn) takes over. escSideMin is in depth-rate units (per frame).
    if (flow < this.escSideMin || this.totEma <= 0) return null;
    // "Both halves alike" is a *relative* test now: the divergence scale moves with speed and distance.
    const close = (this.escSideCloseFrac || 0.2) * this.totEma;
    if (Math.abs(this.sideEma) < close) return "turn180";            // both halves -> head-on, about-face
    return this.sideEma > 0 ? "right" : "left";                      // turn away from the swelling half
  }
  update(rates, dt, tel = null, vision = null) {
    this.elapsed += dt;
    if (this.elapsed < this.settle) {
      for (const [k, v] of Object.entries(rates)) this.sum[k] = (this.sum[k] || 0) + v;
      this.nSettle++;
      return { throttle: 0, yaw: 0, forward: 0, lateral: 0, escape: false, note: "settling" };
    }
    if (!this.baseline) { this.baseline = {}; for (const [k, v] of Object.entries(this.sum)) this.baseline[k] = v / Math.max(1, this.nSettle); }
    const b = (k) => this.baseline[k] || 0;
    const raw = {};
    for (const [axis, spec] of Object.entries(this.axes)) {
      let val = 0;
      for (const [gname, wt] of Object.entries(spec.terms || {})) val += wt * ((rates[gname] || 0) - b(gname));
      raw[axis] = (spec.gain || 0) * val;
    }
    let brake = 0;
    for (const [gname, wt] of Object.entries(this.brakeTerms)) brake += wt * Math.max(0, (rates[gname] || 0) - b(gname));
    this.brake = Math.max(brake, this.brake * 0.93);
    // cruiseFloor keeps the looming brake (DNp03) from ever zeroing the cruise. brake > 1 is entirely
    // reachable, and with a plain max(0, 1 - brake) the drone then hangs motionless in mid-air for as
    // long as the brake takes to decay — `this.brake` above is a decayed peak hold, so roughly a
    // second. Measured: it is the third and least obvious source of "hovering", and the one that has
    // nothing to do with touching anything: every mid-air freeze in a soak had brake 0.6–2.4 while
    // touching was false, at radii from 0.4 m (open floor) to 2.9 m (next to a box). A fly does not
    // stop dead in the air, it slows down, so autoplay keeps a floor under the cruise; the chair run
    // and the swat game leave cruiseFloor at 0 and keep the hard stop.
    raw.forward += this.cruise * Math.max(this.cruiseFloor || 0, 1 - this.brake);
    const a = this.smoothing, cmd = { escape: false, note: "" };
    for (const axis of ["throttle", "yaw", "forward", "lateral"]) {
      this.state[axis] = (1 - a) * this.state[axis] + a * clamp(raw[axis], -1, 1);
      cmd[axis] = Math.abs(this.state[axis]) < this.deadzone ? 0 : this.state[axis];
    }
    const gfNow = Math.max(...this.esc.terms.map((k) => rates[k] || 0));
    this.gf = 0.6 * this.gf + 0.4 * gfNow;
    // Mechanical contact is a second, non-visual trigger for the same giant-fibre escape, and it is
    // what actually gets the drone unstuck: a drone pressed against a surface sees a *static* image,
    // and with no optic flow there is no expansion, so the looming reflex can never fire. It is
    // rate-limited so a long slide along a wall does not turn into non-stop twitching.
    const contactReady = !!(tel && tel.touching) && this.elapsed > this.contactUntil;
    const eye = this.escFromEye(vision); // "left" | "right" | "turn180" | null (read every frame, so the EMA has history when the escape fires)
    // A no-cue fallback climb no longer freezes the drone: it keeps flying forward for the whole hold,
    // so a *new* loom can develop while it is still climbing, and that new loom is allowed to interrupt
    // it (a fresh direction then takes over). Rate-limited, otherwise a rate that simply stays above
    // the threshold would re-pick on every single frame.
    const climbing = this.escPick === "climb" && this.elapsed <= this.escUntil;
    const retrigger = climbing && this.elapsed > this.escRetrigUntil;
    // Speed-adaptive threshold: the looming signal ∝ V/Z, so scaling the threshold by speed/vRef
    // keeps the trigger distance roughly constant across speeds (see escVRef in the constructor).
    // tel is null during warmup — treat that as vRef so the threshold stays at its base value.
    const speed = tel && Number.isFinite(tel.speed) ? tel.speed : this.escVRef;
    const speedRatio = clamp(speed / this.escVRef, 0.3, 3.0);
    const thresholdEff = this.esc.threshold_hz * speedRatio;
    if ((this.gf >= thresholdEff || contactReady) && (this.elapsed > this.escUntil || retrigger)) {
      if (contactReady) this.contactUntil = this.elapsed + this.contactCooldown;
      this.escCause = contactReady ? "contact" : "loom";
      // A contact escape also gets a physical shove off the surface — a turn alone leaves the drone
      // pressed against it (see SimDrone.pushOff). Queued here, applied by Pilot.tick().
      if (contactReady) this.pendingPush = Number.isFinite(this.escBack) ? this.escBack : 0.9;
      if (retrigger) this.escRetrig++; else this.escapes++;
      const r = Math.random();
      const aboveCeil = this.escCeilAlt !== null && tel && tel.alt > this.escCeilAlt;
      // Resolve the weights field-by-field: a partial (or accidentally emptied) mix must fall back
      // to the default split instead of turning every escape into the same direction.
      const pClimb = Number.isFinite(this.escP && this.escP.climb) ? this.escP.climb : 0.10;
      const pLeft = Number.isFinite(this.escP && this.escP.left) ? this.escP.left : 0.45;
      const climbHold = Number.isFinite(this.escClimbHold) ? this.escClimbHold : 1.5;
      this.escEye = eye;
      if (eye) {
        // The whole point of the eye read: the reflex is now a lookup, not a lottery. Whichever half
        // of the retina is swelling harder decides, and the same geometry always picks the same side.
        // A head-on read (both halves alike) is a 180° about-face: no side to pick, so the drone turns
        // around and flies back. The about-face direction comes from the marginal sign of sideEma
        // (small but real), or random if sideEma is truly zero. A turn180 does not climb, so the
        // ceiling check is irrelevant for the eye path.
        this.escPick = eye;
        if (eye === "turn180") {
          this.escTurn180Dir = Math.abs(this.sideEma) > 1e-6
            ? (this.sideEma >= 0 ? 1 : -1)
            : (Math.random() < 0.5 ? -1 : 1);
        }
      } else if (aboveCeil && r < pClimb) {
        // No visual cue + above ceiling: the escP climb fallback is downgraded to a turn.
        this.escPick = Math.random() < 0.5 ? "left" : "right";
      } else if (r < pClimb) this.escPick = "climb";
      else if (r < pClimb + pLeft) this.escPick = "left";
      else this.escPick = "right";
      if (!eye) this.escHist.nocue++;                       // blind escape: contact bump / dead vision
      this.escHist[this.escPick] = (this.escHist[this.escPick] || 0) + 1;
      // A no-cue climb is held for climbHold; a 180° about-face for escTurn180Hold; turns for duration_s.
      if (this.escPick === "climb") {
        this.escUntil = this.elapsed + climbHold;
        this.escRetrigUntil = this.elapsed + (Number.isFinite(this.escClimbRetrig) ? this.escClimbRetrig : 0.3);
      } else if (this.escPick === "turn180") {
        this.escUntil = this.elapsed + (Number.isFinite(this.escTurn180Hold) ? this.escTurn180Hold : 1.85);
        this.escRetrigUntil = 0;
      } else {
        this.escUntil = this.elapsed + this.esc.duration_s;
        this.escRetrigUntil = 0;
      }
    }
    if (this.elapsed <= this.escUntil && this.escPick) {
      cmd.escape = true;
      if (this.escPick === "climb") {
        // No-cue fallback climb: only the vertical channel is taken over, the wing command stays
        // the cruise the brain asked for, so the drone gains height while still covering ground.
        cmd.throttle = this.esc.strength;
        // A ceiling is still a ceiling: once the climb has reached the escape ceiling it stops gaining
        // height (the altitude governor would otherwise have to undo ~1 m after every head-on loom).
        if (this.escCeilAlt !== null && tel && tel.alt > this.escCeilAlt) cmd.throttle = 0;
      } else if (this.escPick === "turn180") {
        // 180° about-face: full yaw in the chosen direction, zero forward / throttle. The drone
        // pivots on the spot for escTurn180Hold seconds (≈ π / yrMax + tau), then the escape
        // releases and the cruise resumes — now pointing the other way.
        cmd.forward = 0; cmd.throttle = 0; cmd.yaw = 0;
        cmd.yaw = this.escTurn180Dir * this.esc.strength;
      } else {
        cmd.forward = 0; cmd.throttle = 0; cmd.yaw = 0;
        cmd.yaw = this.escPick === "left" ? -this.esc.strength : this.esc.strength;
      }
      cmd.note = `giant fiber escape (${this.escPick})`;
    }
    return cmd;
  }
}

// ------------------------------------------------------------------ safety
export class Safety {
  constructor(cfg) {
    const s = cfg.safety;
    this.max = { throttle: s.max_throttle, yaw: s.max_yaw, forward: s.max_forward, lateral: s.max_lateral };
    this.slew = s.slew_per_s; this.minAlt = s.min_alt_m; this.maxAlt = s.max_alt_m; this.fence = s.geofence_radius_m;
    this.prev = { throttle: 0, yaw: 0, forward: 0, lateral: 0 };
    this.lastEvent = "";
  }
  filter(cmd, tel, dt) {
    const out = {}, notes = [];
    for (const a of ["throttle", "yaw", "forward", "lateral"]) out[a] = clamp(cmd[a], -this.max[a], this.max[a]);
    const soft = this.maxAlt - 0.3;
    if (tel.alt > soft && tel.alt < this.maxAlt && out.throttle > 0) out.throttle *= (this.maxAlt - tel.alt) / 0.3;
    if (tel.alt >= this.maxAlt && out.throttle > 0) { out.throttle = Math.min(0, out.throttle) - 0.2; notes.push("ceiling"); }
    if (tel.flying && tel.alt <= this.minAlt && out.throttle < 0) { out.throttle = 0; notes.push("floor"); }
    // Geofence. This used to read `if (r > fence && forward > 0) forward = 0`, which FROZE the drone:
    // the radius was tested with no regard for which way the nose pointed, so a drone at the wall or in
    // a corner had its forward command zeroed *even when heading straight back inside*. And because yaw
    // does not change the radius, it could never satisfy the condition that would give it its forward
    // back — it simply hovered at r ≈ 3 m (measured: the 5 longest stalls of a 6-seed soak, 1.4–1.8 s
    // each, all carried the geofence flag with touching = 0 %). Now the fence only removes the part of
    // the command that points OUT and steers the nose home, so the drone always keeps a way back in.
    const rad = Math.hypot(tel.x, tel.y);
    if (rad > this.fence) {
      const yaw = (tel.yawDeg * Math.PI) / 180;
      const ox = tel.x / rad, oy = tel.y / rad;                     // outward unit vector
      const inward = -(Math.cos(yaw) * ox + Math.sin(yaw) * oy);    // 1 = nose points straight home
      if (out.forward > 0) out.forward *= Math.max(0, inward);      // keep only the inbound part
      if (inward < 0.5) {
        // SimDrone turns with yrT = -cmd.yaw * yrMax, so a *positive* error (the centre is to the left
        // of the nose) needs a negative yaw command.
        const want = Math.atan2(-oy, -ox);                          // bearing of the arena centre
        const err = Math.atan2(Math.sin(want - yaw), Math.cos(want - yaw));
        out.yaw = clamp(out.yaw - clamp(err, -1, 1) * 0.8, -this.max.yaw, this.max.yaw);
      }
      // ...and slide in sideways while the nose comes round. From the wall the heading has to swing
      // ~180° before forward can come back, and at yawRateDps 180 that is a whole second of pirouetting
      // on the spot (the remaining 1 s freezes in a soak, all of them at r ≈ 3.1 m). A strafe has an
      // inward component immediately — 0.8 m/s clears the 0.3 m band in under half a second — so the
      // drone peels off the wall instead of thinking about it.
      const rx = Math.sin(yaw), ry = -Math.cos(yaw);                // SimDrone's lateral axis
      const inLat = -(rx * ox + ry * oy);                           // >0 = strafing this way moves inward
      if (Math.abs(inLat) > 0.2) out.lateral = clamp(out.lateral + inLat * this.max.lateral, -this.max.lateral, this.max.lateral);
      notes.push("geofence");
    }
    const step = this.slew * Math.max(dt, 1e-3);
    const sgn = (v) => (Object.is(v, -0) || v < 0 ? -1 : 1);
    for (const a of ["throttle", "yaw", "forward", "lateral"]) {
      const p = this.prev[a];
      let v = out[a];
      if (Math.abs(v) > Math.abs(p) || sgn(v) !== sgn(p)) v = clamp(v, p - step, p + step);
      out[a] = v;
    }
    this.prev = { ...out };
    if (notes.length) this.lastEvent = notes.join(", ");
    return { ...out, escape: cmd.escape, note: [cmd.note, ...notes].filter(Boolean).join("; ") };
  }
}

// ------------------------------------------------------------------ pilot
export class Pilot {
  constructor(net, opts = {}) {
    this.net = net;
    this.brain = new Brain(net, opts.seed || 0);
    this.cfg = net.config;
    this.retina = new Retina(this.cfg);
    this.encoder = new Encoder(this.brain);
    this.decoder = new Decoder(this.cfg);
    if (opts.cruise !== undefined) this.decoder.cruise = opts.cruise;
    this.safety = new Safety(this.cfg);
    this.illusion = new GestureIllusion();
    this.drone = new SimDrone(opts.start || [-1.5, 0, 0], opts.yawDeg || 0, opts.seed || 0);
    // The eye MUST render exactly the frame the retina expects: Retina.encode() indexes the frame
    // with its own width (blur stride + grid cells), so a mismatch between vision.width/height and
    // the camera silently corrupts the optic-flow maths and crops the field of view — looming then
    // reads as ~0 and the drone never dodges. Keep the two in lockstep by construction.
    if (this.drone.cam.w !== this.retina.w || this.drone.cam.h !== this.retina.h) {
      console.warn(`flydrones: eye ${this.drone.cam.w}x${this.drone.cam.h} vs retina ${this.retina.w}x${this.retina.h} — matching the eye to vision.width/height (check minifly.json)`);
      this.drone.cam = new RayCamera(this.retina.w, this.retina.h);
    }
    this.hz = 20; this.dt = 1 / this.hz; this.t = -(this.decoder.settle + 0.1);
    this.phase = "warmup";
    this.gesture = { present: false, openness: 0, x: 0, y: 0, size: 0, label: "no hand" };
    this.last = null;
    // Optional residual altitude governor (autoplay sets it, chair run / swat game leave it null):
    // { target_m, dead_m, kp, max } — see altitudeTrim().
    this.altHold = null;
    // ---- stall breaker: the last line of defence against the drone parking itself somewhere.
    // Whatever the cause (a corner, a box, a command that got jammed), a drone that has been below
    // stallSpeed for stallAfter seconds gets shoved backwards and swung round, alternating sides.
    // Measured over 6 seeds x 60 s: the longest *legitimate* slow-down was 0.75 s (a looming escape
    // stops forward on purpose, and the command slew takes ~0.5 s to spin back up), so a 1.0 s
    // threshold catches real locks without touching normal dodges.
    this.stallSpeed = 0.08; this.stallAfter = 1.0; this.stallHold = 0.7;
    this.stallBack = -0.6; this.stallTurn = 0.7;
    this.stallT = 0; this.stallBreaks = 0; this.pushes = 0;
    this.unstickUntil = -1; this.unstickDir = 1;
  }
  // The hand illusion (app.js) is what visibly steers the altitude, but its balance point depends on
  // the scene: the stronger the forward optic flow, the more the brain drifts upward, so a faster
  // cruise used to creep up to the safety ceiling and stay there. This is a small-authority P trim on
  // the leftover error, applied AFTER the safety filter. It is skipped during an escape so that a
  // contact escape can still pull the drone off a surface.
  altitudeTrim(cmd, alt) {
    const g = this.altHold;
    if (cmd.escape) return cmd;
    const e = alt - g.target_m;
    if (Math.abs(e) <= g.dead_m) return cmd;
    const trim = clamp(-g.kp * (e - Math.sign(e) * g.dead_m), -g.max, g.max);
    cmd.throttle = clamp(cmd.throttle + trim, -1, 1);
    cmd.note = [cmd.note, "alt hold"].filter(Boolean).join("; ");
    return cmd;
  }
  // Back off + swing round whenever the drone has stopped moving for real. Applied AFTER the safety
  // filter on purpose: this is the one command that may not be filtered away, and the values are well
  // inside the safety envelope anyway. It is not an escape (no giant-fibre story behind it), so it does
  // not set cmd.escape — it only shows up as a note and in Pilot.stallBreaks.
  stallBreaker(cmd, dt) {
    const d = this.drone;
    if (!d.flying) return cmd;
    const shove = () => {
      cmd.forward = this.stallBack; cmd.yaw = this.unstickDir * this.stallTurn;
      cmd.note = [cmd.note, "stall breaker"].filter(Boolean).join("; ");
      return cmd;
    };
    if (this.t < this.unstickUntil) return shove();     // still inside a shove
    const sp = Math.hypot(d.vel[0], d.vel[1]);
    this.stallT = sp < this.stallSpeed ? this.stallT + dt : 0;
    if (this.stallT < this.stallAfter) return cmd;
    this.stallT = 0; this.stallBreaks++;
    this.unstickUntil = this.t + this.stallHold;
    this.unstickDir = -this.unstickDir;                 // alternate, so two breaks do not loop the same way
    return shove();
  }
  tick() {
    const dt = this.dt, d = this.drone;
    const gray = d.frame();
    let vision = this.retina.encode(gray);
    const tel = d.telemetry();
    let rates, spikes, cmd;
    if (this.phase === "warmup") {
      const r = this.brain.tick(this.encoder.encode(vision, 0), dt * 1000);
      rates = r.rates; spikes = r.spikes;
      this.decoder.update(rates, dt, null, vision);
      cmd = { throttle: 0, yaw: 0, forward: 0, lateral: 0, escape: false, note: "warming up the brain" };
      this.t += dt;
      if (this.t >= 0) { this.t = 0; this.phase = "flight"; d.takeoff(); }
    } else {
      vision = this.illusion.apply(vision, this.gesture, this.t);
      const r = this.brain.tick(this.encoder.encode(vision, tel.yawRateDps), dt * 1000);
      rates = r.rates; spikes = r.spikes;
      const raw = this.decoder.update(rates, dt, tel, vision);
      cmd = this.safety.filter(raw, tel, dt);
      if (this.altHold) cmd = this.altitudeTrim(cmd, tel.alt);
      // The contact reflex's shove goes to the airframe rather than through the command (see pushOff).
      if (this.decoder.pendingPush > 0) {
        if (d.pushOff(this.decoder.pendingPush)) this.pushes++;
        this.decoder.pendingPush = 0;
      }
      cmd = this.stallBreaker(cmd, dt);
      d.send(cmd);
      this.t += dt;
    }
    for (let s = 0; s < 4; s++) d.step(dt / 4);
    this.last = { gray, vision, rates, spikes, cmd, tel: d.telemetry() };
    return this.last;
  }
}
