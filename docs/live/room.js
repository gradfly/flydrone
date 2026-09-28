// The flight arena the drone flies in: white walls that hide when the camera is behind them,
// a blue/white/gray foam-mat floor, and eight foam-block obstacles that mirror the simulator's
// collision boxes (passed in from engine.js ROOM.boxes). Two themes (day / night).
import * as THREE from "three";
import { createObstacleMesh } from "./models.js";

const SX = 6, SY = 6, H = 2.7;

function tex(w, h, draw, rx = 1, ry = 1) {
  const c = document.createElement("canvas"); c.width = w; c.height = h;
  draw(c.getContext("2d"), w, h);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; t.anisotropy = 8;
  t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(rx, ry);
  return t;
}
const std = (color, extra = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.8, ...extra });

export function createRoom(scene, boxes = []) {
  const root = new THREE.Group(); scene.add(root);
  const themed = []; // [mesh, dayMaterial, nightMaterial]
  const neonLines = [];
  const add = (parent, geo, day, night, pos, opts = {}) => {
    const m = new THREE.Mesh(geo, day); m.position.set(...pos);
    m.castShadow = opts.cast ?? true; m.receiveShadow = opts.receive ?? true;
    if (opts.rot) m.rotation.set(...opts.rot);
    parent.add(m); if (night) themed.push([m, day, night]);
    return m;
  };

  // ------------------------------------------------------------ textures
  // floor: 0.5 m foam puzzle mats in blue / white / gray (4x4 tiles per repeat, repeated 3x over 6 m)
  const drawMats = (palette, bevelAlpha, speckle) => (g, w, h) => {
    const n = 4, s = w / n;
    for (let j = 0; j < n; j++) for (let i = 0; i < n; i++) {
      g.fillStyle = palette[j * n + i]; g.fillRect(i * s, j * s, s, s);
      for (let k = 0; k < speckle; k++) {
        g.fillStyle = `rgba(${Math.random() < 0.5 ? "255,255,255" : "30,36,46"},${0.03 + Math.random() * 0.05})`;
        g.fillRect(i * s + Math.random() * s, j * s + Math.random() * s, 2.5, 2.5);
      }
      g.strokeStyle = `rgba(20,26,36,${bevelAlpha})`; g.lineWidth = 3; g.strokeRect(i * s + 1.5, j * s + 1.5, s - 3, s - 3);
      g.strokeStyle = "rgba(255,255,255,.45)"; g.lineWidth = 1.5; g.strokeRect(i * s + 4, j * s + 4, s - 8, s - 8);
    }
  };
  const matDayPalette = ["#e9e7e0", "#d2d6da", "#e9e7e0", "#a9afb6",
                         "#d2d6da", "#e9e7e0", "#8fb0cf", "#d2d6da",
                         "#e9e7e0", "#c3c8cd", "#e9e7e0", "#d2d6da",
                         "#a9afb6", "#d2d6da", "#e9e7e0", "#8fb0cf"];
  const matNightPalette = ["#151a23", "#181e29", "#151a23", "#1f2634",
                           "#181e29", "#151a23", "#17293c", "#181e29",
                           "#151a23", "#1b2129", "#151a23", "#181e29",
                           "#1f2634", "#181e29", "#151a23", "#17293c"];
  const floorDay = tex(512, 512, drawMats(matDayPalette, 0.16, 90), SX / 2, SY / 2);
  const floorNight = tex(256, 256, drawMats(matNightPalette, 0.5, 0), SX / 2, SY / 2);
  const wallDay = tex(512, 512, (g, w, h) => {
    g.fillStyle = "#f1f3f5"; g.fillRect(0, 0, w, h);
    for (let x = 0; x < w; x += 128) { g.fillStyle = "rgba(150,160,172,.2)"; g.fillRect(x, 0, 2, h); }
    g.fillStyle = "#dde2e7"; g.fillRect(0, h * 0.72, w, h * 0.28);
    g.fillStyle = "#c8cdd3"; g.fillRect(0, h * 0.72, w, 5);
  }, SX / 1.5, 1);
  const wallNight = tex(256, 256, (g, w, h) => {
    g.fillStyle = "#0b1017"; g.fillRect(0, 0, w, h);
    g.fillStyle = "#0f1622"; g.fillRect(0, 0, w / 2, h);
    g.fillStyle = "rgba(76,201,240,.18)"; g.fillRect(0, h * 0.92, w, 3);
  }, SX / 0.6, 3);

  const matFloorDay = new THREE.MeshStandardMaterial({ map: floorDay, roughness: 0.92 });
  const matFloorNight = new THREE.MeshStandardMaterial({ map: floorNight, roughness: 0.85, metalness: 0.1 });
  const matWallDay = new THREE.MeshStandardMaterial({ map: wallDay, roughness: 0.9 });
  const matWallNight = new THREE.MeshStandardMaterial({ map: wallNight, roughness: 0.9 });

  // foam-mat floor
  add(root, new THREE.PlaneGeometry(SX, SY), matFloorDay, matFloorNight, [0, 0, 0], { rot: [-Math.PI / 2, 0, 0], cast: false });

  // ------------------------------------------------------------ walls
  // sim +x wall = three x=+3, sim -x = three x=-3, sim +y = three z=-3, sim -y = three z=+3
  const walls = [];
  const wallDefs = [
    { name: "east", pos: [SX / 2, H / 2, 0], rotY: -Math.PI / 2, test: (c) => c.x > SX / 2 },
    { name: "west", pos: [-SX / 2, H / 2, 0], rotY: Math.PI / 2, test: (c) => c.x < -SX / 2 },
    { name: "north", pos: [0, H / 2, -SY / 2], rotY: 0, test: (c) => c.z < -SY / 2 },
    { name: "south", pos: [0, H / 2, SY / 2], rotY: Math.PI, test: (c) => c.z > SY / 2 },
  ];
  for (const d of wallDefs) {
    const grp = new THREE.Group(); grp.position.set(...d.pos); grp.rotation.y = d.rotY; root.add(grp);
    add(grp, new THREE.PlaneGeometry(SX, H), matWallDay, matWallNight, [0, 0, 0], { cast: false });
    add(grp, new THREE.BoxGeometry(SX, 0.1, 0.03), std(0xf4f6f8), std(0x0e141d), [0, -H / 2 + 0.05, 0.015]);
    const strip = new THREE.Mesh(new THREE.BoxGeometry(SX, 0.012, 0.01), new THREE.MeshBasicMaterial({ color: 0x4cc9f0 }));
    strip.position.set(0, -H / 2 + 0.11, 0.03); grp.add(strip); neonLines.push(strip);
    walls.push({ ...d, grp });
  }

  // ------------------------------------------------------------ obstacles (mirror the simulator boxes)
  const obstacles = new THREE.Group(); root.add(obstacles);
  const obstacleList = [];
  for (const box of boxes) {
    const { group, neon } = createObstacleMesh(box);
    obstacles.add(group); neonLines.push(neon);
    obstacleList.push({ name: box.name, box, neon, cx: (box.lo[0] + box.hi[0]) / 2, cy: (box.lo[1] + box.hi[1]) / 2 });
  }

  // ------------------------------------------------------------ lights
  const hemi = new THREE.HemisphereLight(0xffffff, 0x9aa0a8, 1.0); root.add(hemi);
  const sun = new THREE.DirectionalLight(0xfff1d6, 2.2);
  sun.position.set(1.5, 4.5, -4.5); sun.target.position.set(-0.5, 0, 0.5);
  sun.castShadow = true; sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, { left: -4, right: 4, top: 4, bottom: -4, near: 0.5, far: 14 });
  sun.shadow.bias = -0.0006; sun.shadow.normalBias = 0.02;
  root.add(sun, sun.target);
  // soft ceiling fill (replaces the old hanging lamp)
  const fill = new THREE.PointLight(0xdfe8ff, 0, 9, 1.8); fill.position.set(0, H - 0.4, 0); root.add(fill);
  const neonA = new THREE.PointLight(0x4cc9f0, 0, 9, 1.6); neonA.position.set(-2.4, 1.6, 2.4); root.add(neonA);
  const neonB = new THREE.PointLight(0xff4d8d, 0, 9, 1.6); neonB.position.set(2.4, 1.6, -2.2); root.add(neonB);

  // floating dust
  const dustN = 260, dustGeo = new THREE.BufferGeometry(), dustPos = new Float32Array(dustN * 3), dustVel = new Float32Array(dustN * 3);
  for (let i = 0; i < dustN; i++) {
    dustPos.set([(Math.random() - 0.5) * SX * 0.9, Math.random() * H, (Math.random() - 0.5) * SY * 0.9], i * 3);
    dustVel.set([(Math.random() - 0.5) * 0.02, (Math.random() - 0.5) * 0.01, (Math.random() - 0.5) * 0.02], i * 3);
  }
  dustGeo.setAttribute("position", new THREE.BufferAttribute(dustPos, 3));
  const dust = new THREE.Points(dustGeo, new THREE.PointsMaterial({ color: 0xfff6dc, size: 0.012, transparent: true, opacity: 0.55, depthWrite: false }));
  root.add(dust);

  let theme = "day";
  const api = {
    root, walls, obstacles, sun, dust,
    get theme() { return theme; },
    setTheme(t, { scene: sc, bloom, renderer } = {}) {
      theme = t;
      const night = t === "night";
      for (const [m, day, nightMat] of themed) m.material = night ? nightMat : day;
      for (const l of neonLines) l.visible = night;
      hemi.intensity = night ? 0.25 : 1.0; hemi.color.set(night ? 0x8fb8ff : 0xffffff);
      hemi.groundColor.set(night ? 0x1a2030 : 0x9aa0a8);
      sun.intensity = night ? 0.15 : 2.2; sun.color.set(night ? 0x8fa8ff : 0xfff1d6);
      fill.intensity = night ? 2.4 : 0; fill.color.set(night ? 0x9fd8ff : 0xdfe8ff);
      neonA.intensity = night ? 10 : 0; neonB.intensity = night ? 8 : 0;
      dust.material.opacity = night ? 0.25 : 0.55;
      if (sc) { sc.background = new THREE.Color(night ? 0x04060a : 0xcfd6de); sc.fog = new THREE.Fog(night ? 0x04060a : 0xcfd6de, 10, 24); }
      if (bloom) { bloom.strength = night ? 0.85 : 0.25; bloom.threshold = night ? 0.18 : 0.92; bloom.radius = night ? 0.45 : 0.3; }
      if (renderer) renderer.toneMappingExposure = night ? 1.05 : 1.0;
    },
    update(camera, dt) {
      for (const w of walls) w.grp.visible = !w.test(camera.position);
      for (let i = 0; i < dustN; i++) {
        for (let k = 0; k < 3; k++) {
          dustPos[i * 3 + k] += dustVel[i * 3 + k] * dt;
          const lim = k === 1 ? [0, H] : [-SX * 0.45, SX * 0.45];
          if (dustPos[i * 3 + k] < lim[0]) dustPos[i * 3 + k] = lim[1];
          if (dustPos[i * 3 + k] > lim[1]) dustPos[i * 3 + k] = lim[0];
        }
      }
      dustGeo.attributes.position.needsUpdate = true;
    },
    // outline the obstacle closest to the drone (sim coords [x, y, z]) when something looms
    highlightNearest(simPos, level) {
      let best = null, bd = Infinity;
      for (const o of obstacleList) {
        const d = (o.cx - simPos[0]) ** 2 + (o.cy - simPos[1]) ** 2;
        if (d < bd) { bd = d; best = o; }
      }
      for (const o of obstacleList) {
        const on = o === best && level > 0.12;
        if (theme === "day") o.neon.visible = on;
        o.neon.material.color.set(level > 0.4 ? 0xffb020 : 0x4cc9f0);
      }
    },
  };
  api.setTheme("day");
  return api;
}
