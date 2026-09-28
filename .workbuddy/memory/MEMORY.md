# FlyDrone live demo — long-term project notes

- 版本戳纪律：改任何 `docs/live/*.js` 都要同步 bump 三处 —— `docs/index.html` 的 `app.js?v=N`、`app.js` 顶部 `BUILD`、以及 app.js 内部对 engine/models/room 的 `?v=N` import。否则浏览器缓存旧文件，用户会看到旧行为（已发生过两次误报）。
- 用户验证偏好：改完代码**不要**创建 .mjs 验证脚本，用户自己在浏览器验证；需要数据时用 `node --input-type=module -e '...'` 内联跑 engine.js（可直接 Node import；改 `ROOM.boxes = makeObstacles(seed)` 可换布局）。复用现成的 `.workbuddy/hl_verify.mjs` 无头 harness 也算合规。
- autoplay 专属参数集中在 `app.js` 的 `AUTO_TUNE`（cruise/maxForward/loomGain/loomFloor/yawRateDps/yawGain/escPickP/altHold），用户会自己改这些数值；chair run / swat game 不受影响。
- 手部幻觉机制：DROP = `{present:false}` → "rising illusion" → 下降；张开手掌 → 上升；静止手(size 0)保持。幻视平衡点随前向光流强度漂移，所以高度需要 Pilot.altitudeTrim 兜底。
