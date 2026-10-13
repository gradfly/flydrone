# FlyDrone live demo — long-term project notes

- 版本戳纪律：改任何 `docs/live/*.js` 都要同步 bump 三处 —— `docs/index.html` 的 `app.js?v=N`、`app.js` 顶部 `BUILD`、以及 app.js 内部对 engine/models/room 的 `?v=N` import。否则浏览器缓存旧文件，用户会看到旧行为（已发生过两次误报）。
- 用户验证偏好：改完代码**不要**创建 .mjs 验证脚本，用户自己在浏览器验证；需要数据时用 `node --input-type=module -e '...'` 内联跑 engine.js（可直接 Node import；改 `ROOM.boxes = makeObstacles(seed)` 可换布局）。复用现成的 `.workbuddy/hl_verify.mjs` 无头 harness 也算合规。
- autoplay 专属参数集中在 `app.js` 的 `AUTO_TUNE`（cruise/maxForward/loomGain/loomFloor/yawRateDps/yawGain/escPickP/altHold），用户会自己改这些数值；chair run / swat game 不受影响。
- 手部幻觉机制：DROP = `{present:false}` → "rising illusion" → 下降；张开手掌 → 上升；静止手(size 0)保持。幻视平衡点随前向光流强度漂移，所以高度需要 Pilot.altitudeTrim 兜底。
- 用户会**边聊边改** `AUTO_TUNE`：动这个文件前先重读当前内容（loomFloor/yawGain 已被用户改过 0.15→0.12、0.08→0.1），只改自己负责的部分，别覆盖他的数值。
- 障碍物数量：`engine.js` 的 `OBSTACLE_COUNT`（**v22 起 = 8**，原 6）+ `makeObstacles(seed, count)`；room.js / models.js / app.js 全部从 `ROOM.boxes` 跟随，改一处即可。Python 侧镜像在 `src/flydrones/drones/sim.py`（`OBSTACLE_COUNT` / `make_obstacles` / `Room.arena`），**`_mulberry32` 必须 bit-exact**（signed int32 + `>>>`，且 `t = (t + imul(t^(t>>>7), 61|t)) ^ t` 这个 `+` 不能漏，漏了就布局不同；已对 5 seed 逐位验证）。
- 逃逸（giant fiber）**v22 起改为"确定性查表 + 概率兜底"**：读眼 → 左半侧更近→右转 / 右半侧更近→左转 / 两侧接近→**180° about-face（`turn180`）**；只有"盲逃逸"（撞到东西→画面静止→无任何膨胀）才落到 `escP` 兜底（`{climb:0.10, left:0.45}`）。方向线索是**每半侧纹理加权平均 flow divergence（=2·Vz/Z）**，不是 expL/expR。
  - v17 的 head-on **爬升**已被 about-face 取代；climb 只剩兜底用途，`escClimbP` 参数消失（变 `escPickP`），`escHist` 多一个 `turn180` 键，`climbDown` 计数器删除。
  - `escTurn180Hold ≈ π/yrMax + tau`（tau=0.35 s）；180 °/s → 1.35 s，120 °/s → 1.85 s。`escSideCloseFrac` 0.20 → **0.10**；`escClimbHold` 1.5 → 1.2。
- **v22 新增 `loom_fov_frac`**：looming **触发**只读画面中央 `frac` 宽度（`margin = round(ncols*(1-frac)/2)`），方向判读仍是全宽。默认 1.0（minifly.json），app.js `AUTO_TUNE.loomFovFrac: 0.3` = 中央 **30%**（注释里写"50%"是旧默认残留，实现是 30%，JS/Python 两边一致）。
- **v22 新增速度自适应 looming 阈值**：`threshold_eff = threshold_hz * clamp(speed/v_ref, 0.3, 3.0)`，`v_ref` 默认 0.8 m/s。**依赖 `telemetry.speed`**（新增字段，= `hypot(vel[0], vel[1])`）；没有 speed 时退回基础阈值。
- **v22 碰撞改为 OBB**：yaw 投影机体半长 `half_x=0.15`(前)/`half_z=0.21`(侧)/`half_y=0.05`(垂直)，世界半长 `hwx = hx*|cos| + hz*|sin|`、`hwy = hx*|sin| + hz*|cos|`；箱子沿**最浅轴**推出（minimal translation，3 遍）而不是 revert step。实测 yaw=0 停距 0.150、45° 0.255、30° 0.235，无穿模。
- **★ 碰撞数不能用来判断"规则好坏"，必须 A/B 同一竞技场**：v22 首轮 soak 282 次碰撞看着像退步，实为 8-box 竞技场更挤。同一竞技场只换规则的对照：v17 694 → **v22 282（−59%）**，stall 31.7→19.4 s，低速 65.2→38.3 s。碰撞数对参数高度混沌（同值换种子差 20 倍），只当方向性证据。
- **climb 与转向上有代价权衡**（v17 时代结论，仍适用）：爬升会带高度、纯转向不带，所以把 climb 概率压得过低（0.20/0.10）反而碰撞/接触变多（6 种子实测 6 → 43 / 39 碰撞）。

- **"悬停不动"的三个真凶（v17 全部修掉，症状≠碰撞代码）**：①`Safety.filter` 的 geofence 只看半径不看朝向，`r>3` 就把 forward 清零且永远无法解除 → 贴墙/角悬停；②接触逃逸只 yaw 不位移（贴面时 collide 刚好抵消朝里那份速度）→ 现在给 `SimDrone.pushOff()` 物理推离 + `contactCooldown` 0.9 s；③`cruise * max(0, 1-brake)` 里 brake 能 >1 → 半空直接把巡航清零 → 现在有 `decoder.cruiseFloor`(autoplay 0.4)。外加 `Pilot.stallBreaker()`（<0.08 m/s 持续 1.0 s → 倒车+转向）兜底，在 safety.filter 之后执行。停顿时间 70.2 s → 4.9 s。
- **诊断法（可复用）**：报"卡住/悬停/抖动"先写症状探测器（低速持续时长 + 同时刻 touching/note/brake/半径），别凭直觉改物理。本次直觉（碰撞箱）全错。
- 视网膜线索的正/副作用：`expL/expR`（每半侧仿射斜率）**无法**判断 looming 在哪一侧（对真实障碍方位只有 ~50–60% 一致）；每半侧取 max divergence 也**错**（max 恒为地板→全部判成 head-on）。只有"每半侧平均 divergence"能定位（~70–83%）。
- loomFloor 与"何时 flinch"不是单调关系：5 种子 ×45 s 实测 0.15 → 46 碰撞 / 0.08 → 71 碰撞（逃逸次数 3 倍、长时间停在高处）。调 loomFloor 必须连 escClimbHold 一起看。
- 无头验证：`node %TEMP%/fly_esc_soak.mjs`（临时目录的 harness，仓库内不留 .mjs）+ 本地 server **必须在同一次工具调用里启动并跑完**；A/B 参数用 CDP `Fetch.enable` 拦截 `*app.js*` 改写响应体，不用改仓库文件。沙箱 `http_proxy` 存在，curl 探活要 `--noproxy '*'`。

## Python 侧（serial 后端 / autoplay）—— 2026-09-29

- **Python 环境**：pytest 只装在 `C:\Users\xx.mu\.workbuddy\binaries\python\envs\flydrone\Scripts\python.exe`（managed 3.13.12 **没装**）。**2026-09-29 已 `pip install -e .`**（editable），所以现在 `import flydrones` 任意目录可用、且有了 `Scripts/flydrones.exe` 控制台命令；**不再需要 `PYTHONPATH=src`**。跑测试直接 `python -m pytest`。当前基线 **74 passed, 1 skipped**（v22 同步后）。
- **★ soak 必须后台跑 + 落盘**：6 seed × 45 s × 4 子步 ≈ 2 分钟，前台会吃到命令墙钟上限被 SIGTERM（现象：stdout/stderr **全空** + Exit 1）。用 `run_in_background` + `> log 2>&1`，再 Read log。
- **写 harness 的参数结构坑**：`decoder.cruise` 在 `decoder` 下，`cruise_floor` 在 `decoder.escape` 下——写错层级会让 cruise=0，飞机压根不前进，整轮数据废掉。做法：`yaml.safe_load("src/flydrones/defaults.yaml")` 再改，别手搓 dict。
- **API 备忘**：`Telemetry` **没有** `collisions`（在 `SimDrone.collisions`）；`SimDrone` 没有 `ground`；`EscapePicker` 属性名是 `esc_p`（不是 `pick_p`）；当前选择在 `decoder.picker.esc_pick`，`decoder` 自身没有这个属性。
- `tools/check_web_engine.mjs` 的 `FAIL: fist holds` 是**既有失败**（旧 v17 demo 上同样失败，属手势幻视高度漂移检查），与 v22 同步无关。
- **报错 `No module named 'flydrones'` 的成因**：①在仓库目录之外执行；②Windows PowerShell 里 `PYTHONPATH=src cmd` 这种前缀赋值不成立（PowerShell 语法不同）。根治办法就是 `pip install -e .`。给用户的命令优先用 `Scripts/flydrones.exe ...`，不要再教 `PYTHONPATH=src`（bash 专用，用户很可能在 PowerShell 里敲）。
- **★ 修"方向/符号"类 bug 后必须做 mutation test**：临时把符号反过来，确认测试**会 fail**。`test_escape.py` 的 ground-truth 测试前三版全是**假通过**（对准障碍中心→侧向偏移恒 0；测 divergence 层而非 picker 层→符号无关；断言"膨胀侧==障碍侧"→大箱子应判 head-on）。有效断言是**不变量**："一旦规则决定转向，转向必须背离障碍"（正确 3/3 away/0 into，反符号 0/3 away/3/3 into）。"命中率 xx%" 类断言抓不住符号反转。
- **转向符号（已实测，勿凭直觉改）**：图像**左**半侧膨胀 → 障碍在左 → **向右转**。`FlightCommand.yaw` +1 = CW/右；`SimDrone` `yr_t = -c.yaw*yr_max`；body 轴实测 `f=(cos,sin)`、`r=(sin,-cos)`。"镜像不变" 的推理是**错的**（镜面同时翻转视网膜特征左右划分，命令符号不动，两者不抵消）。
- **移植后的实测收益**（6 seed×45 s，真 Pilot，同 seed）：旧随机逃逸 collisions 399 / nocue 116 → 移植+正确符号 **231 / 29**（少 ~42% 碰撞）。`nocue`（无视觉线索次数）是关键 sanity 指标：它降下来才证明眼睛读数真在生效，没 fallback 到骰子。碰撞数对参数高度混沌，只当方向性证据。
- **serial 链路手册**：`run_sim` 断言 `isinstance(p.drone, SimDrone)` → serial 要手写 loop 用 `Pilot.tick(t, dt)`；`Pilot.warmup(sec, dt)` 才会发 arming 帧；`SerialDrone` 无 `port_obj`，注入用 `d.serial=Fake; d._open=lambda: ...`；`decode_frames` **只吃 bytearray**（传 bytes → TypeError）；rate limiter 用 `time.monotonic()`，喂合成时间戳必须**同时 monkeypatch monotonic**，否则只出几帧。
- serial 的 `--send` 不影响线上字节（两种模式写的字节完全一样），真正挡无人机的是飞控自己的解锁；文档措辞已按此修正。
