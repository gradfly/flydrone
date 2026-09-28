// Headless browser check: render the arena, catch a frame where an obstacle proximity-alarm is lit,
// and screenshot day + night. Drives a Chrome we launch ourselves over raw CDP (the agent-browser
// daemon is unreliable on this machine).
import { spawn } from "node:child_process";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";

const PORT = process.env.PORT || "8941";
const SEED = process.env.SEED || "";
const URL_ = `http://127.0.0.1:${PORT}/index.html?record=1&autoplay=1&view=orbit&ang=2.4&rad=4.6&h=2.9${SEED ? `&seed=${SEED}` : ""}`;
const OUT = "D:/OneDrive/project/飞控/果蝇神经网络模型/flydrone/.workbuddy";

function findChrome() {
  const cands = [
    "C:/Program Files/Google/Chrome/Application/chrome.exe",
    "C:/Program Files (x86)/Google/Chrome/Application/chrome.exe",
    path.join(os.homedir(), "AppData/Local/Google/Chrome/Application/chrome.exe"),
    "C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe",
    "C:/Program Files/Microsoft/Edge/Application/msedge.exe",
  ];
  for (const c of cands) if (fs.existsSync(c)) return c;
  // chromium bundled with agent-browser
  const tmp = path.join(os.homedir(), "AppData/Local/Temp");
  for (const d of fs.readdirSync(tmp).filter((n) => n.startsWith("agent-browser-chrome-"))) {
    for (const sub of ["chrome-win64/chrome.exe", "chrome-win/chrome.exe", "chrome.exe"]) {
      const p = path.join(tmp, d, sub);
      if (fs.existsSync(p)) return p;
    }
    const hit = fs.readdirSync(path.join(tmp, d), { withFileTypes: true })
      .filter((e) => e.isDirectory()).map((e) => path.join(tmp, d, e.name, "chrome.exe")).find(fs.existsSync);
    if (hit) return hit;
  }
  throw new Error("no chrome/edge binary found");
}

const exe = findChrome();
const udd = fs.mkdtempSync(path.join(os.tmpdir(), "cdp-verify-"));
const chrome = spawn(exe, [
  "--headless=new", "--remote-debugging-port=0", `--user-data-dir=${udd}`,
  "--no-first-run", "--no-default-browser-check", "--disable-gpu-sandbox",
  "--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--window-size=1500,900",
  "about:blank",
], { stdio: "ignore" });

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
let ws, id = 0;
const pending = new Map(), events = [];
const send = (method, params = {}, sessionId) => new Promise((res, rej) => {
  const m = ++id; pending.set(m, { res, rej });
  ws.send(JSON.stringify(sessionId ? { id: m, method, params, sessionId } : { id: m, method, params }));
});
const evaluate = async (expr) => {
  const r = await send("Runtime.evaluate", { expression: expr, awaitPromise: true, returnByValue: true });
  if (r.exceptionDetails) throw new Error("eval: " + (r.exceptionDetails.exception?.description || r.exceptionDetails.text));
  return r.result.value;
};

try {
  // wait for the debugger port file
  let port = 0;
  for (let i = 0; i < 80 && !port; i++) {
    const f = path.join(udd, "DevToolsActivePort");
    if (fs.existsSync(f)) port = fs.readFileSync(f, "utf8").split("\n")[0].trim();
    else await sleep(250);
  }
  if (!port) throw new Error("chrome never opened a devtools port");
  const targets = await (await fetch(`http://127.0.0.1:${port}/json/list`)).json();
  const page = targets.find((t) => t.type === "page");
  ws = new WebSocket(page.webSocketDebuggerUrl);
  await new Promise((res, rej) => { ws.onopen = res; ws.onerror = rej; });
  ws.onmessage = (e) => {
    const m = JSON.parse(e.data);
    if (m.id && pending.has(m.id)) { const p = pending.get(m.id); pending.delete(m.id); m.error ? p.rej(new Error(JSON.stringify(m.error))) : p.res(m.result); }
    else if (m.method === "Runtime.consoleAPICalled") events.push("console " + m.params.args.map((a) => a.value).join(" "));
    else if (m.method === "Runtime.exceptionThrown") events.push("EXC " + (m.params.exceptionDetails.exception?.description || m.params.exceptionDetails.text));
  };
  await send("Runtime.enable");
  await send("Page.enable");
  await send("Emulation.setDeviceMetricsOverride", { width: 1500, height: 900, deviceScaleFactor: 1, mobile: false });
  await send("Page.navigate", { url: URL_ });

  for (let i = 0; i < 60; i++) { if (await evaluate("!!(window.FD && FD.ready)").catch(() => false)) break; await sleep(400); }
  console.log("build:", await evaluate("FD.state().build"), "| boxes:", await evaluate("FD.state().boxes"));

  // ---- anti-stick soak: fast-forward 90 s of sim time, track the longest continuous contact
  await evaluate("FD.advance(2)");
  let streak = 0, maxStreak = 0, bumps = 0, prevT = false, escContact = 0, prevEsc = 0, cols = 0;
  for (let i = 0; i < 360; i++) {
    await evaluate("FD.advance(0.25)");
    const s = await evaluate("JSON.stringify(FD.state())").then(JSON.parse);
    streak = s.touching ? streak + 0.25 : 0;
    maxStreak = Math.max(maxStreak, streak);
    if (s.touching && !prevT) bumps++;
    prevT = s.touching;
    if (s.escapes > prevEsc && s.escCause === "contact") escContact++;
    prevEsc = s.escapes;
    cols = s.collisions;
  }
  console.log(`soak 90s: collisions=${cols} bumps=${bumps} contactEscapes=${escContact} longestContact=${maxStreak.toFixed(2)}s`);

  // ---- catch a lit-alarm frame for the screenshots (highlightNearest runs inside FD.frame)
  let lit = [], st = null;
  for (let i = 0; i < 400; i++) {
    await evaluate("FD.advance(0.25); FD.frame(0.05)");
    st = await evaluate("JSON.stringify(FD.state())").then(JSON.parse);
    lit = st.lit;
    if (lit.length && (st.touching || st.loom > 0.15)) break;
  }
  for (let i = 0; i < 2; i++) await evaluate("FD.frame(0.05)"); // stay on the detected alarm frame
  const shot = async (name) => {
    const r = await send("Page.captureScreenshot", { format: "png" });
    fs.writeFileSync(path.join(OUT, name), Buffer.from(r.data, "base64"));
    console.log("wrote", name);
  };
  console.log("lit obstacles:", JSON.stringify(lit), "state:", await evaluate("JSON.stringify(FD.state())"));
  await evaluate("FD.setView('orbit')");
  await shot("alarm_day.png");

  await evaluate("FD.theme('night'); FD.advance(0.05)");
  for (let i = 0; i < 3; i++) { await evaluate("FD.advance(0.05)"); await evaluate("FD.frame(0.05)"); }
  await shot("alarm_night.png");
  await evaluate("FD.theme('day'); FD.setView('drone'); FD.frame(0.05)");
  await shot("alarm_eyes.png");

  const errs = events.filter((e) => e.startsWith("EXC"));
  console.log("console lines:", events.length, "| js errors:", errs.length);
  if (errs.length) console.log(errs.slice(0, 5).join("\n"));
} finally {
  try { ws && ws.close(); } catch {}
  chrome.kill();
}
