# FlyDrone live demo — long-term project notes

- 版本戳纪律：改任何 `docs/live/*.js` 都要同步 bump 三处 —— `docs/index.html` 的 `app.js?v=N`、`app.js` 顶部 `BUILD`、以及 app.js 内部对 engine/models/room 的 `?v=N` import。否则浏览器缓存旧文件，用户会看到旧行为（已发生过两次误报）。
- 用户验证偏好：改完代码**不要**创建 .mjs 验证脚本，用户自己在浏览器验证；需要数据时用 `node --input-type=module -e '...'` 内联跑 engine.js（可直接 Node import；改 `ROOM.boxes = makeObstacles(seed)` 可换布局）。复用现成的 `.workbuddy/hl_verify.mjs` 无头 harness 也算合规。
- autoplay 专属参数集中在 `app.js` 的 `AUTO_TUNE`（cruise/maxForward/loomGain/loomFloor/yawRateDps/yawGain/escPickP/altHold），用户会自己改这些数值；chair run / swat game 不受影响。
- 手部幻觉机制：DROP = `{present:false}` → "rising illusion" → 下降；张开手掌 → 上升；静止手(size 0)保持。幻视平衡点随前向光流强度漂移，所以高度需要 Pilot.altitudeTrim 兜底。
- 用户会**边聊边改** `AUTO_TUNE`：动这个文件前先重读当前内容（loomFloor/yawGain 已被用户改过 0.15→0.12、0.08→0.1），只改自己负责的部分，别覆盖他的数值。
- 障碍物数量：`engine.js` 的 `OBSTACLE_COUNT`（当前 6）+ `makeObstacles(seed, count)`；room.js / models.js / app.js 全部从 `ROOM.boxes` 跟随，改一处即可。
- 逃逸（giant fiber）现在是**确定性查表 + 概率门**：`Decoder.escFromEye` 每帧读眼 → 左半侧更近→右转 / 右半侧更近→左转 / 两侧接近→**按 `escClimbP`(0.35) 概率**爬升 `escClimbHold`(1.5 s) **且保持巡航前进**（没抽到爬升就朝略大一侧转），爬升期间允许新 looming 每 `escClimbRetrig`(0.3 s) 重选；只有"盲逃逸"（撞到东西→画面静止→无任何膨胀）才回落到 `escP` 随机转身。方向线索是**每半侧纹理加权平均 flow divergence（=2·Vz/Z）**，不是 expL/expR。
- **climb 与转向上有代价权衡**：爬升会带高度、纯转向不带，所以把 climb 概率压得过低（0.20/0.10）反而碰撞/接触变多（6 种子实测 6 → 43 / 39 碰撞）。碰撞数对参数高度混沌（同值换种子差 20 倍），只当方向性证据。
- **"悬停不动"的三个真凶（v17 全部修掉，症状≠碰撞代码）**：①`Safety.filter` 的 geofence 只看半径不看朝向，`r>3` 就把 forward 清零且永远无法解除 → 贴墙/角悬停；②接触逃逸只 yaw 不位移（贴面时 collide 刚好抵消朝里那份速度）→ 现在给 `SimDrone.pushOff()` 物理推离 + `contactCooldown` 0.9 s；③`cruise * max(0, 1-brake)` 里 brake 能 >1 → 半空直接把巡航清零 → 现在有 `decoder.cruiseFloor`(autoplay 0.4)。外加 `Pilot.stallBreaker()`（<0.08 m/s 持续 1.0 s → 倒车+转向）兜底，在 safety.filter 之后执行。停顿时间 70.2 s → 4.9 s。
- **诊断法（可复用）**：报"卡住/悬停/抖动"先写症状探测器（低速持续时长 + 同时刻 touching/note/brake/半径），别凭直觉改物理。本次直觉（碰撞箱）全错。
- 视网膜线索的正/副作用：`expL/expR`（每半侧仿射斜率）**无法**判断 looming 在哪一侧（对真实障碍方位只有 ~50–60% 一致）；每半侧取 max divergence 也**错**（max 恒为地板→全部判成 head-on）。只有"每半侧平均 divergence"能定位（~70–83%）。
- loomFloor 与"何时 flinch"不是单调关系：5 种子 ×45 s 实测 0.15 → 46 碰撞 / 0.08 → 71 碰撞（逃逸次数 3 倍、长时间停在高处）。调 loomFloor 必须连 escClimbHold 一起看。
- 无头验证：`node %TEMP%/fly_esc_soak.mjs`（临时目录的 harness，仓库内不留 .mjs）+ 本地 server **必须在同一次工具调用里启动并跑完**；A/B 参数用 CDP `Fetch.enable` 拦截 `*app.js*` 改写响应体，不用改仓库文件。沙箱 `http_proxy` 存在，curl 探活要 `--noproxy '*'`。
