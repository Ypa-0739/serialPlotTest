# 项目交接 Prompt（比赛物流小车 · 上位机 + 固件）

> 用法：把本文件全文复制给新的 AI 会话（Claude Code / Codex 均可）作为初始 prompt，
> 配合下方工作区路径，即可无缝继续本项目。本文件是截至 2026-08-18 的完整状态快照。

---

## 0. 项目概况

**任务**：比赛物流小车。STM32 单航点底盘闭环已经可用，上位机基础通信、自检和单航点导航已完成 PC 仿真测试，当前处于树莓派 5 上位机继续迭代及后续实车联调阶段。

**硬件**：
- STM32F407VET6 主控 + 4×ZDT_X42S 闭环步进电机
- 麦克纳姆轮底盘（运动学：`V_bl=-Vx+Vy-Vz*L; V_fl=Vx+Vy-Vz*L; V_fr=Vx-Vy-Vz*L; V_br=-Vx-Vy-Vz*L`，`L=(ROBOT_H/2)+(ROBOT_W/2)`）
- OPS9 定位传感器（串口接入 STM32，输出 X/Y/YAW，中心偏移 +Y 25mm）
- 树莓派 5 上位机（串口 115200 控制 STM32，未来接摄像头视觉）

**工作区**：`c:\Users\steph\STM32CubeIDE\workspace_1.19.0\serialPlotTest`
**Git 分支**：`agent/stm32-mecanum-pid`（**所有改动保留在工作树，一律不提交**）
**当前基线提交**：`8b786e120e7822c9307f4b7f06bcb900611eb945`（`8b786e1 add mecanum chassis PID control and tuning`）
**Python 环境**：`C:\Users\steph\sjtu-agent\.venv\Scripts\python.exe`（Windows 测试用）

## 1. 工作约束（必须遵守，来自用户明确指示）

1. **不提交**：所有改动只保留在工作树；除非用户明确要求，不做 git add/commit
2. **中文注释**：关键控制逻辑添加中文注释；代码风格与周围代码一致
3. **Codex 接管模式**：本会话不调用 DeepSeek，不要求 OpenAI API Key，全部本地推理
4. **保留物理急停**；软件层急停 = `emergency_stop()` 原子锁存 + 清队 + 仅保留最高优先级 STOP
5. **PING 不是硬实时**：Linux+Python 无法保证硬实时；真正的硬安全边界在 STM32（1.5s HOST LOST 看门狗）。Pi 侧用"线程隔离 + 充足余量"（400~500ms 周期）满足
6. **唯一写者**：只有 `SerialBridgeThread` 能调 `serial.write()`；外部经 `send()` 投递队列；`readline()` 绝不无限阻塞（timeout=0.05, write_timeout=0.1）
7. **重连绝不自动恢复运动**：断线重连只发 `SerialConnected`，绝不重发旧命令；恢复运动必须重新自检（STOP→MODE WORK→STATUS→OPS→CAN→PID 重载→READY→release）
8. **测试不依赖 Windows 虚拟串口软件**：FakeSerial 注入 + loop:// 回环集成
9. **追加不覆盖**：`codex_advisor_history.jsonl` 只追加

## 2. 已完成的开发阶段（全部通过验证）

| 阶段 | 内容 | 状态 |
|---|---|---|
| 0 | STM32 固件：PID 封装、调参模块 llm_tuner、4 项安全修复 | CubeIDE 编译通过 |
| 1 | 串口桥 serial_bridge.py：唯一写者/心跳/重连/急停门禁 | 45/45 测试 |
| 2 | 启动自检状态机 state_machine.py：STOP→…→READY | 60/60 测试 |
| 3 | 主循环 main.py + FakeFirmware demo.py + 集成测试 | 66/66 测试 |
| 4 | 单航点导航 navigator.py + models.py + event_router.py | 85/85 测试 |

**当前：85/85 测试全部通过**，`git diff --check` 与 `py_compile` 通过。**尚未实车串口验证**。

## 3. 关键文件与路径

### 固件（STM32，工作区根目录）
- `Core/Src/main.c` — 协议实现（命令解析/响应 printf/安全看门狗/制动限幅）。**协议唯一权威来源**
- `Core/Inc/llm_tuner.h` + `Core/Src/llm_tuner.c` — 调参模块封装（ROUND START/STOP、17 列 CSV 遥测、OPS 中心偏移补偿）
- `Core/Src/dma.c` — DMA2_Stream7 中断优先级=5（必须低于 OPS9(USART2=1)/主机RX(USART1=2)/CAN(3)）
- `Core/Src/tim.c` — TIM3/TIM4 未使用勿启动（注释说明）
- `serialPlotTest.ioc` — CubeMX 配置（与 dma.c 同步）
- `llm-pid-tuner-main/tuner.py` — 调参脚本（含 `_heartbeat_sleep` 心跳空窗修复：分步睡眠每 0.4s 补发 PING，把最坏静默窗口从 1.1s 压到 0.4s）

### 上位机（`pi-brain/`，Python，未提交）
- `pi-brain/app/protocol.py` — 纯协议层：命令编码（encode_pose_set 等）+ 行解析（parse_line）+ 事件类（PoseStarted/PoseReached/SafetyFault/…）+ `is_motion_command` + 优先级常量（PRIORITY_STOP=0/PING=10/MOTION=20/CONFIG=30）。**新增运动命令时必须同步 `_MOTION_COMMAND_PREFIXES` 与测试**
- `pi-brain/app/serial_bridge.py` — 单线程四职责（RX/PING/TX/重连）：PriorityQueue(priority, seq, text)、PING 按 monotonic deadline 先于 TX、tx_drain_limit=8、断线自动锁存急停+清队、BACKOFF 退避重连、`is_emergency_stopped`/`release_emergency_stop`/`MotionCommandRejected`
- `pi-brain/app/state_machine.py` — StartupStateMachine：STOP→MODE WORK→STATUS→OPS STATUS→CAN STATUS→PID SET X/Y/YAW→PID LIMIT X/Y/YAW→PID STATUS ALL→READY；每步 1.5s 超时；OPS 必须 LINK=OK+有效帧；CAN 必须 STATE=2+ERROR=0；PID isclose abs_tol=1e-7；FAULT 再 STOP；重连重自检；全过才 release
- `pi-brain/app/navigator.py` — **单航点导航控制器（第四阶段核心）**：IDLE→WAIT_POSE_START→MOVING→REACHED；WAIT_POSE_START/MOVING→CANCELLING→CANCELLED；任意运动状态→FAILED。安全规则：门禁双检查（set_ready + is_emergency_stopped）、单航点、未确认启动就到位=协议异常 FAILED(UNEXPECTED_REACHED)、**START_TIMEOUT 与 MOTION_TIMEOUT 都 emergency_stop() 锁存**（不能假设固件未动，可能只是回复丢失）、取消确认超时升级 STOP 再超时锁存、绝不自动重发、终态后下一次 goto_pose 才清理（串口串行化）
- `pi-brain/app/models.py` — Pose / PoseGoal(name,x_mm,y_mm,yaw_deg,timeout_s) / NavState / GotoReason / GotoResult(goal_id,success,reason,final_pose,position_error_mm,yaw_error_deg,elapsed_s)
- `pi-brain/app/event_router.py` — 同步 EventRouter：主循环唯一事件源→订阅顺序广播；各模块绝不自己 get_event()
- `pi-brain/app/demo.py` — FakeFirmware：响应式固件仿真（printf 精度一致）；POSE 仿真：延迟 START/TARGET、注入误差、pose_start_ok/reached_ok/safety_after_s/stopped_delay_s/ack_stop 开关、新 POSE SET 覆盖旧目标 pending 响应、POSE STOP 清 pending
- `pi-brain/app/main.py` — 主入口：事件泵（get_event(0.05)→router.publish→startup.tick→navigator.tick）、READY/FAULT 门禁切换、demo 模式 READY 后自动提交演示航点；`--port`/`--baud`/`--demo`
- `pi-brain/tests/` — test_protocol.py(26)/test_serial_bridge.py/test_stress.py/test_integration.py(6)/test_navigator.py(19)/fakes.py

### 测试框架要点
- 测试全部走 FakeFirmware/FakeSerial 注入，无真实硬件
- 事件泵模式：`pump()` 循环 `bridge.get_event` → `handle_event` → `tick()`
- 注意：入队与写出异步，断言前先 pump 等待（如 STOP 实际写出）

### 文档、参考资料和无关文件
- `README.md` — 当前 STM32 固件功能、WORK/TUNE/PLOT 三模式、串口命令、SerialPlot 通道和构建说明；理解固件时先读
- `HANDOFF_PROMPT.md` — 本交接快照；新会话应先核对实际 Git 状态再使用，不能把快照当成永远正确
- `llm-pid-tuner-main/config.json` — LLM 调参程序实际配置；不要误用工程根目录的旧 `config.json`
- `llm-pid-tuner-main/logs/pid_results.jsonl` — PID 调参结果历史
- `llm-pid-tuner-main/logs/codex_advisor_history.jsonl` — AI 调参过程与评价历史，只追加、不覆盖
- `pi-brain/requirements.txt` — 树莓派上位机 Python 依赖
- `pi-brain/sample_x.txt`、`pi-brain/sample_check.txt` — 本地样本/检查资料，不是运行时必需模块
- `pi-brain/pdf-pages/`、`pi-brain/ocr_lines/`、`pi-brain/ocr_run.py`、`pi-brain/assemble_md.py` 以及根目录 `read.md` — 另一项 OCR/文档处理产生的资料，与物流小车控制无关；不要纳入架构、测试或提交，也不要擅自删除
- `.claude/` — 用户已有未跟踪目录，与本项目此次改动无关；不要删除、覆盖或误提交

## 4. 固件协议速查（与 main.c 一致）

**命令（Pi→STM32，一行 \n 结尾）**：
- `PING` → `# PONG`
- `STOP` → `# STOP MODE=WORK`（或 TUNE 中 `# ROUND STOP HOST`）
- `MODE WORK` → `# MODE WORK PLOT=0 CHANGED=0|1`
- `STATUS` → `# STATUS MODE=... HOST_PROTO=2 AXIS=... P=%.7f I=%.8f D=%.7f MAX_OUT=... STATE=0 PLOT=0 MOTOR_PROTO=EMM OPS_FRAMES=... UART_TX_OK=... UART_TX_ERR=...`
- `OPS STATUS` → `# OPS LINK=OK|STALE|NO_DATA|BYTES_NO_FRAME X=... Y=... YAW=... FRAMES=...`（含 CENTER_X/Y、OFFSET_Y=25.00 等）
- `CAN STATUS` → `# CAN STATE=2 ERROR=0x... FREE=... TX_OK=... TX_ERR=...`
- `PID SET <AXIS> <p> <i> <d>` → `# PID LOADED AXIS=... P=... I=... D=...`
- `PID LIMIT <AXIS> <out>` → `# PID LIMIT AXIS=... OUTPUT=... UNIT=MPS`
- `PID STATUS ALL` → `# PID ALL X=p,i,d Y=p,i,d YAW=p,i,d`
- `POSE SET <x_mm> <y_mm> <yaw_deg>`（OPS 原始坐标）→ `# POSE START X=... Y=... YAW=... CENTER_X=... CENTER_Y=... TOL_MM=5.00 TOL_YAW=1.00` → 到位 → `# POSE TARGET X=... Y=... YAW=... ERROR_MM=... ERROR_YAW=...`
- `POSE STOP` → `# POSE STOP`；安全停车 → `# POSE STOP SAFETY`
- 调参轮次：`# ROUND START <n> AXIS=<a> DIR <d> X=... Y=... YAW=...` / `# ROUND STOP <REASON> AXIS=... X=... Y=... YAW=...`（REASON∈TARGET/TIMEOUT/OPS LOST/HOST LOST/OVERTRAVEL/CROSS TRACK/TRANSLATION LIMIT/WRONG DIR/YAW LIMIT/AXIS/HOST/RESET）
- 调参启动 4 格式（全部=运动命令）：`SET P:%f I:%f D:%f`、`SET KP:%f KI:%f KD:%f`、`PID %f %f %f`、`P:%f,I:%f,D:%f`
- 运动命令族（is_motion_command）：`MOVE `(除 MOVE STOP)、`TURN `、`POSE SET `、`MOTOR RUN `、`NAV GOTO `、`CMD VEL `、`SET P:`、`SET KP:`、`P:`；`PID SET/LIMIT/STATUS` 豁免
- 17 列 CSV 遥测：timestamp,setpoint,input,output,error,p,i,d,ops_x,ops_y,yaw,cross,yaw_delta,hold_cross,hold_yaw,center_x,center_y

**解析优先级**（protocol.py parse_line）：PONG → POSE STOP(==) → POSE STOP SAFETY(startswith) → POSE START/TARGET → ROUND → STATUS → OPS LINK → CAN → ERROR；`0x...` 分支在 `_KV_RE` 最前（避免 0x00000004 误判为 0）

## 5. 当前状态与下一步

**已验证**：串口桥（唯一写者/心跳/重连/急停门禁）→ 启动状态机（READY）→ 主循环骨架 → 单航点导航闭环。demo 模式可在 PC 上完整演示：自检→READY→提交航点→MOVING→REACHED→[NAV RESULT]。

**实车接入前未验证的边界**：USB 串口实际断开行为、STM32 回复延迟、Linux 设备名变化、OPS 首次有效帧时间、CAN 状态稳定性、长时间缓冲与日志性能。

**推荐开发顺序**（用户已确认，按序推进，每步完成后向用户汇报并等指示）：
1. **第六阶段 JSONL 结构化日志**：每行一个事件（ts/monotonic_s/task_state/nav_state/goal_id/target/event/raw/elapsed_s）；独立低优先级线程或队列写入，避免磁盘卡顿影响事件泵与心跳；订阅 EventRouter
2. 实车低速单航点验证 → 连续多航点
3. 第五阶段比赛任务状态机：WAIT_START→MOVE_TO_PICK→ALIGN_PICK→PICK→MOVE_TO_DROP→ALIGN_DROP→DROP→RETURN→FINISHED（+PAUSED/ABORTED/RECOVERY/EMERGENCY_STOP）；只能调用 navigator.goto_pose() 与机构接口，不能直接发 POSE SET；第一版用预标定航点序列，不上 A* / 样条 / 动态避障
4. 视觉进程（NCNN 优先后端抽象、Picamera2、假视觉桩）、FastAPI debug 面板（可选）

**维护性提醒**：`is_motion_command` 前缀表必须与固件新增运动命令同步；新增协议行时同步 parse_line 与 test_protocol.py。

## 6. 运行与验证

```bash
# 全量测试（工作区根目录下）
cd pi-brain && C:/Users/steph/sjtu-agent/.venv/Scripts/python.exe -m unittest discover -s tests -t .

# demo 演示（无硬件，验证自检+导航闭环；Windows 控制台中文乱码为 GBK 编码问题，Pi 上无影响）
cd pi-brain && C:/Users/steph/sjtu-agent/.venv/Scripts/python.exe -u -m app.main --demo

# 真串口
python -m app.main --port COM3        # Windows
python -m app.main --port /dev/ttyUSB0  # Pi（自动枚举：不带 --port）

# 检查
git diff --check
```
