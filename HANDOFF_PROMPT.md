# 项目交接说明：STM32 底盘 + 树莓派导航/视觉 + PID 调参

> 更新日期：2026-09-09。以当前工作树为准，不再沿用 2026-08-18 的阶段快照。
> 已完成停车/通信修复、四项控制层优化及架构拆分。本轮按用户要求更新 README，并将主仓库修改提交、推送至 origin/main；发布结果以 Git 记录和本轮最终回复为准。未烧录，驱动器通信超时配置和实车验收仍待完成。

## 1. 工作区与约束

- 仓库：https://github.com/stephenchiao/serialPlotTest
- 工作区：`C:\Users\steph\STM32CubeIDE\workspace_1.19.0\serialPlotTest`
- 架构重构起点：`main`，`741100f`（feat: harden Raspberry Pi binary control protocol）。后续会话先运行 `git status` 和 `git log -1`，不要把起点当作当前提交。
- 根工程已有多项本地修改；`llm-pid-tuner-main` 是带自身本地修改的子模块。不要覆盖、回退或一并提交用户原有工作。
- 用户已授权本轮主仓库 commit/push；调参子模块、IDE 本机文件、根配置和个人笔记不纳入本次提交。后续未得到要求不自动发布。关键控制逻辑使用中文注释。
- 保留物理急停。软件确认不能替代电机零速反馈，更不能证明 CAN 故障下电机已经停车。
- 不调用外部 LLM 或要求 API Key；本轮修复与测试完全本地完成。
- `read.md`、OCR 样本及工具、`.claude/` 等不是小车运行模块，不清理、不纳入本轮提交。
- 日志历史只追加，不覆盖。根目录旧 `config.json` 与调参工具实际配置应明确区分。

## 2. 当前架构和已实现功能

硬件：STM32F407VET6，四个 ZDT X42S 闭环步进电机，OPS9 位姿传感器，树莓派 5，DM-G6220 云台。

- CAN1：500 kbit/s，PB8 RX/PB9 TX，底盘电机 ID1 左后、ID2 左前、ID3 右前、ID4 右后。
- CAN2：1 Mbit/s，PB12 RX/PB13 TX，G6220；需独立收发器。
- USART1：主机链路，115200；USART2：OPS9，115200，PA2/PA3。
- 车体 +X 向右，+Y 向前；位置 mm，航向 degree；运动学输入 m/s 和 rad/s。
- OPS 安装偏移为车体中心前方 25 mm。协议 POSE 目标是 OPS 原始坐标；地图航点为车体中心坐标，由 `map_frame.py` 转换，不能混用。

STM32 已有三轴 PID、中心偏移补偿、20 ms POSE 控制、二维速度斜坡、制动限速和先平移后转向的两阶段航点控制。当前到位条件为 2 mm / 0.5 degree，并连续稳定 25 个控制周期。

Pi 已有串口桥、启动自检、单航点导航、路线执行、Mission、JSONL 异步日志、地图启动锚定、双速度档、独立视觉线程与人工视觉运动联调入口。这些模块不再是“待实现阶段”。真实机构仍需接入，Mission 默认空机构只适合仿真。

职责链：`Mission / RouteRunner -> Navigator -> SerialBridgeThread -> STM32`。视觉发布观察结果，由任务层检查类型、来源、编号、置信度和新鲜度；不能直接访问串口或四轮控制。

## 3. 主机门禁和协议基线

- 上电进入 HOST WAIT，四轮零速使能保持，G6220 失能；等待超时不自动运动。
- `HOST LINK COM` / `HOST LINK RPI` 声明主机类型；每次声明停车并清旧运动，不是多主机互斥锁。
- COM 调试不要求心跳；RPI 的 WORK/PLOT 使用 1.5 秒主机失联监督，Pi 通常每 0.4 秒发 PING。
- TUNE 不要求 PING，但保留其他故障保护：每轮最长 5 秒、每会话最多 20 轮，返回目标 POSE 最长 15 秒。
- HOST 文本协议版本 **4**；二进制 wire 版本 **2**；能力位 **0x0000003F**。本轮没有修改版本或帧布局。
- 数值常量唯一来源：`protocol/rpi_binary_protocol.json`，通过 `tools/generate_rpi_protocol.py` 生成 C/Python 常量和数值文档。
- Pi 自检包括主机声明、STOP、MODE WORK、STATUS、OPS/CAN、PID 重载/校验；完成 ASCII 自检后，经 `HOST BINARY START` 和 VERSION/CAPS 回复显式切换。
- 正式目标使用带 `goal_id` 的二进制事务，限速与目标原子提交；不自动重发运动。丢事件通过 QUERY 核对，不据此重放旧目标。
- 重连/冷重启不得恢复旧运动；先探测遗留会话、停车，等待退回 ASCII，再重新自检。

协议细节见 `docs/binary-pose-protocol.md`、`docs/binary-protocol-reference.generated.md`。

## 4. 前轮五项修复（保留）

### 4.1 Pi 急停与出队竞态

`serial_bridge.py` 增加清队序号边界 `_discard_before_seq`。清队不仅删除队列中的项，还使已出队但未写出的旧对象失效。实际写出前在安全锁内校验序号及二进制运动门禁，因此急停后即使重新 release，旧目标也不能发出。

### 4.2 STM32 二进制 STOP 撤销旧命令

`RpiProtocol_QueuePush(..., urgent=1)` 清空普通 RX FIFO，并保留独立 STOP 槽。STOP 尚未出队时普通帧拒绝入队，计入 dropped；STOP 出队后允许新的显式命令。

STOP 仍是正常停车命令，不改变协议版本、不强制重新握手。客户端必须等待 STOP 回复后再提交新业务。Pi 的 `emergency_stop()` 另有锁存门禁，只有重新完成自检/显式允许后才能解除。

### 4.3 STM32 文本接收不再忙时丢 STOP

新增 `Core/Inc/host_command_rx.h`、`Core/Src/host_command_rx.c`：

- 四项有界命令队列，每行最多 63 个字符（另加 NUL）；持续组行，不因普通命令待处理而停止接收。
- 独立 STOP 锁存，可抢占满队列并清旧命令；STOP 待处理时丢弃普通命令。
- 超长行、非法控制字符行整行丢弃，不执行合法截断前缀；停车出队时已开始接收的半行也作废。
- 主循环出队保存/恢复 PRIMASK；UART 故障重置接收状态。
- `HOST RX STATUS` 返回 `# HOST RX DROPPED=<n> INVALID=<n>`；DROPPED 含队列溢出及 STOP 撤销项，INVALID 为无效行。UART 接收重置会清零计数。

新增源文件必须在 CubeIDE 刷新后进入构建；不要只重新链接旧对象。

### 4.4 应用退出执行有界停车交接

新增 `SerialBridgeThread.shutdown(timeout=0.75)`：锁存急停并禁止新业务，由串口线程写出 STOP，限时等待回复，再关闭串口线程。返回 `ShutdownResult(stop_written, stop_acknowledged, thread_stopped)`；主入口及绘图入口打印这些状态，不再依赖固定 sleep。

- `stop_written` 只表示串口 write 接受完整字节；短写也按失败处理。
- `stop_acknowledged` 表示收到协议停车回复，二进制沿用请求序号匹配；ASCII 本身没有请求序号，只观察本次写出后的 STOP 回复；TUNE 接受 ROUND STOP HOST，不把 TARGET 等自然结束当作确认。
- 两项都不代表电机已停。未连接、写失败、无 ACK 都不能报告成功。
- STOP 等待预算默认 0.75 秒，随后底层线程 join 最多 2 秒；遵循串口有界读写前提。退出过程中链路失败不尝试重连。
- `stop()` 保留为底层关闭/测试清理接口，应用退出使用 `shutdown()`。业务事件仍留在原事件队列，不会被关闭流程抢走。

### 4.5 调参工具兼容当前握手回复

`llm-pid-tuner-main/hw/bridge.py` 先等待 STOP 回复，再发送 HOST LINK COM；按完整 token 匹配 `# HOST LINK COM OK`，兼容追加的 `HEARTBEAT=OFF` 等字段，同时拒绝 RPI、BUSY、OKAY 等错误回复。

## 5. 本轮四项控制层优化

### 5.1 UART/CAN 有界异步发送与周期统计

- `host_uart_tx.c` 接管 USART1 文本输出：512 字节行缓冲，32 行普通 FIFO、4 行停车优先 FIFO；W/P/调参 CSV 各保留最新一行。DMA 使用独立静态缓冲，完成前不得覆盖，二进制切换只丢弃未发送文本。
- 主机文本与二进制共享 USART1 DMA，发送忙超过 100 ms 触发原有 UART 故障停车及会话清理。文本超长行/控制队列溢出也触发故障；遥测覆盖只计数。OPS ZERO 改用 USART2 中断发送，失败明确报错。
- CAN1 改为 16 项普通 FIFO、每轮一个最新速度槽和每轮一个 STOP 槽；一次调度最多提交 3 帧。普通帧等待超过 50 ms 丢弃并锁存故障；STOP 超时仍保留重试。STOP 清普通队列并申请撤销旧邮箱，四轮零速都离开邮箱后才恢复普通发送。已经上总线的旧帧不能撤回。
- 移除主机命令中的逐轮 `HAL_Delay`，使能后的 5 ms 间隔由调度器按电机记录；启动等待期间仍调度 CAN 和轮速查询。CAN1/CAN2 接收中断每次最多取 3 帧。
- `CONTROL STATUS` 返回主循环/控制步最大间隔、超过 25 ms 的控制步数、超过 100 ms 的主循环间隔数、故障及 IWDG 复位来源；第二行返回 UART/CAN 拥塞统计。计时单位为 HAL tick 的 ms，属于间隔统计，不是示波器测得的执行时间。
- **CAN 发送函数返回 0、TX_EN/TX_RUN=0 和 MOTOR_EN 均只表示软件提交/请求状态，不表示驱动器已执行。**
- `serialPlot.c` 的旧 `SerialPlot_SendFloats` 仍保留有限阻塞发送，但当前业务没有调用；不要直接重新接入 USART1，以免绕过文本/二进制 DMA 的所有权管理。

### 5.2 PID 显式 dt、微分滤波及下游抗积分饱和

- POSE 和 TUNE 三轴统一使用 `PID_CalcDt` / `PID_CalcErrorDt`；保留旧 API 作为 20 ms 包装。
- 参数继续使用旧版 20 ms 基准：积分增量为 `error * dt / 0.020`，微分原始量为 `delta_error * 0.020 / dt`。**不批量换算现有 Ki/Kd，不修改 JSON 或协议参数单位。**
- 微分默认一阶低通 `tau=0.020 s`，重置后的首步不产生微分冲击。实际输出会因滤波及抗饱和而改变，旧参数仍需低速复测。
- `PID_ApplyOutput` 接收经过矢量、制动、斜坡、保持轴及四轮比例限幅的最终软件指令，同向受限时撤销本步积分。POSE 的 X/Y 反馈先从车体系转回全局系；TUNE 主轴恢复归一化方向。该值不是电机实测速度，也没有建模驱动器内部加速度曲线。

### 5.3 OPS 完整位姿快照

`OPS9_GetSnapshot()` 在保存/恢复 PRIMASK 的短临界区内一次复制 X/Y/YAW、帧序号和更新时间。POSE、TUNE、状态及遥测不再直接拼接中断共享的位姿字段；新鲜度判断在快照后获取当前 tick。解析器仍拒绝非有限浮点帧。

### 5.4 电机反馈新鲜度、停止确认与独立看门狗

- 四轮速度始终按 10 ms 一轮轮询，每台约 25 Hz，与 HOST/TELEM 开关无关。反馈必须满足地址、单包索引、长度、方向和末字节格式；记录 RPM、接收时刻和序号。
- 任一所需电机超过 300 ms 无有效轮速时拒绝非零速度；运动中触发 `MOTOR FEEDBACK LOST`。单电机台架只要求该轮新鲜；底盘运动要求四轮。二进制沿用 CAN_FAULT 原因码，wire 布局不变。
- `MOTOR STOP STATUS` 显示 REQUESTED / WAIT_FEEDBACK / CONFIRMED / UNCONFIRMED。四轮零速离开 CAN 邮箱后建立反馈序号基线，必须每轮再收到至少两个新样本，且连续两次 `abs(rpm)<=1`、四轮均新鲜，才能 CONFIRMED。超过 600 ms 未确认则 UNCONFIRMED，每 100 ms 重发零速，不刷新原始超时起点；迟到的有效反馈仍可转为 CONFIRMED。已确认后非零或过期反馈会撤销确认。
- `MOTOR FEEDBACK` 返回各轮 VALID/RPM/AGE_MS/SEQ；VALID 表示曾收到有效反馈，新鲜度须结合 AGE_MS 判断。
- `control_runtime.c` 在启动等待完成后直接配置 F407 IWDG 寄存器，仅健康主循环末尾喂狗；256 分频、重载 249，按 32 kHz LSI 名义约 2 s。超过 100 ms 主循环间隔锁存控制故障，运动停车；重新声明 HOST LINK 才清该故障。未加入 HAL IWDG 模块，也未修改 CubeMX 外设配置；保留 USER CODE 中的初始化调用。
- **IWDG 复位不能保证外置驱动器停转。当前没有该批 X42S 固件的通信超时配置证据，未发送任何猜测的参数写入命令。** 必须按对应固件手册配置、读回并验证驱动器断通信停机行为，再关闭这一验收项。调试暂停默认可能触发 IWDG，不能把调试冻结行为当作看门狗验证。

详细设计、命令含义及故障注入步骤见 [控制层验证说明](docs/control-layer-validation.md)。

### 5.5 架构拆分与构建入口

- `main.c` 从 2681 行缩减至 194 行，保留 CubeMX 启动和 `RobotApp_Init/Process` 调用。业务状态改为 `robot_app.c` 私有；板级 HAL 回调集中到 `board_events.c`。
- `host_rx_router.c` 独立处理文本/二进制分流，不依赖 HAL，不读取应用会话状态。ISR/主循环同步由应用保存/恢复 PRIMASK。
- `motion_math.c` 统一 POSE/TUNE 的安装偏移、角度、坐标、斜坡计算。业务协议与 POSE 状态机仍在应用模块，后续拆分计划见 `docs/architecture.md`。
- `tools/build_firmware.py` 替代本机临时构建脚本，不依赖 Debug 清单或 CubeIDE 路径；`.github/workflows/ci.yml` 在 push/PR 时运行主仓库回归和 ARM 构建。
- 调参子模块的前轮本地修复保留在子模块工作树，未更新主仓库 gitlink。17 项调参测试是本地结果，不表示所引用远端子模块已包含这些修复。

## 6. 关键文件

| 范围 | 文件 |
| --- | --- |
| CubeMX 启动 / 应用 / HAL 适配 | `Core/Src/main.c`、`robot_app.c`、`board_events.c` |
| 接收路由 / 共享运动计算 | `Core/Src/host_rx_router.c`、`motion_math.c` |
| 文本组行、队列及 STOP | `Core/Src/host_command_rx.c` |
| 文本 DMA、CAN 调度 | `Core/Src/host_uart_tx.c`、`zdtCan.c` |
| 轮速监督、IWDG/周期 | `Core/Src/motor_monitor.c`、`control_runtime.c` |
| 位姿快照 | `Core/Src/ops9.c` |
| 二进制编解码与队列 | `Core/Src/rpi_protocol.c` |
| PID、运动学、调参 | `Core/Src/pid.c`、`mecanum_chassis.c`、`llm_tuner.c` |
| 唯一串口写者、停车交接 | `pi-brain/app/serial_bridge.py` |
| 自检与导航 | `pi-brain/app/state_machine.py`、`navigator.py` |
| 任务、路线与地图 | `mission.py`、`route_runner.py`、`map_frame.py`、`speed_profile.py` |
| 事件泵与日志 | `pi-brain/app/main.py`、`event_router.py`、`jsonl_logger.py` |
| 视觉与联调 | `pi-brain/app/vision/`、`pi-brain/tools/vision_motion_debug.py` |
| PC 调参握手 | `llm-pid-tuner-main/hw/bridge.py` |

## 7. 本轮验证基线

- Pi 全量 unittest：**192 项通过**。
- 调参工具 `test_hw_bridge.py`：**9 项通过**；`test_hardware_tui.py`：**8 项通过**。
- 五套便携 C 测试通过，编译参数 `-std=c11 -Wall -Wextra -Werror`；新增接收路由与共享运动计算测试，保留控制层 9 组 HAL 替身场景。
- 协议生成一致性检查通过，版本/能力位未变。
- ARM GCC 13.3.rel1：按 Debug 目标参数从源码编译链接 **46 个单元**，包含新增架构模块；**零警告**。
- 本轮构建产物：`tmp/firmware/serialPlotTest.elf`；日志、map 在同目录。`text=100580`、`data=512`、`bss=29072` 字节。这是仓库构建脚本的独立 ARM 结果，不能冒充 CubeIDE 已刷新过的 Debug ELF。
- 未烧录，未完成实车故障注入。自动测试不运行真实 HAL/DMA/IWDG，也未验证电机实际停车延迟或 PID 实车改善幅度。

Windows 测试环境：`C:\Users\steph\sjtu-agent\.venv\Scripts\python.exe`（本轮可直接使用 python）。

```powershell
# 项目根目录
python tools/generate_rpi_protocol.py --check
Push-Location pi-brain
python -B -m unittest discover -s tests -t .
Pop-Location
Push-Location llm-pid-tuner-main
python -B -m unittest discover -s tests -p test_hw_bridge.py
python -B -m unittest discover -s tests -p test_hardware_tui.py
Pop-Location
python -B tools/run_c_tests.py
python -B tools/build_firmware.py --toolchain-bin "<ARM GCC tools/bin>"
git diff --check
git -C llm-pid-tuner-main diff --check
```

CubeIDE 1.19.0 中刷新工程并完整构建，确认 `Debug/Core/Src/subdir.mk` 和 `Debug/objects.list` 包含全部 Core/Src 文件，尤其是 `robot_app`、`board_events`、`host_rx_router`、`motion_math` 和前轮控制模块。本机完整 ARM 工具链位于 `E:/STMCubeIDE/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins/` 下的 GNU tools 插件；当前使用入库脚本 `tools/build_firmware.py`，临时目录不入库。

## 8. 下一步：先上板验收，再扩展任务

1. 台架验证 ASCII 突发/满队列 STOP、二进制旧目标+STOP、退出与 USB 断开、Pi 冷重启；确认旧目标不恢复，并测量实际停车延迟。
2. 依照控制层验证说明注入 USART1 ORE/FE、DMA 饱和、单轮失联、CAN 故障和主循环卡死，记录 CONTROL STATUS、反馈新鲜度和停车确认状态。先核实并验证 X42S 通信超时停机配置。
3. 低速复测现有 PID，比较控制周期分布、超调和到位时间；重点验证非零 Ki/Kd、四轮限幅及 POSE 平移转旋转阶段，不凭主机测试宣称实车性能提升。
4. 低速固定路线重复验证后，再接真实机构异步完成/超时接口及视觉精对准；道路和障碍投影仍需现场标定。
5. 按 `docs/architecture.md` 从应用模块抽取类型明确的统一命令和 POSE 控制器；同步维护真实协议回复样本，避免 FakeFirmware 与固件分叉。CI 执行结果以 GitHub Actions 为准。
