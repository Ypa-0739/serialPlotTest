# STM32F407 麦克纳姆轮物流小车底层控制

基于 STM32F407VET6、四轮麦克纳姆底盘和 4 个 ZDT X42S 闭环步进电机的底层电控工程，包含 CAN 电机控制、OPS9 位姿反馈、X/Y/YAW PID 闭环以及串口调参与安全保护。

## 硬件与接口

- 主控：STM32F407VET6
- 电机：4 × ZDT X42S，CAN1 500 kbit/s，ID 1～4
- CAN1：PB8 RX、PB9 TX
- 云台：DM-G6220，CAN2 1 Mbit/s 标准帧，PB12 RX、PB13 TX（需独立CAN收发器）
- OPS9：USART2，PA2 TX、PA3 RX
- PC 调试：USART1，115200 baud
- 电机布局：ID1 左后、ID2 左前、ID3 右前、ID4 右后

## 坐标系

- `+X`：车体右侧
- `+Y`：车体前方
- `YAW`：OPS 航向角，单位 degree
- 线速度：m/s；角速度：rad/s

OPS9 安装在车体中心前方 25 mm。固件保留原始 OPS 坐标，同时计算补偿后的车体中心坐标，避免原地旋转时把传感器圆弧位移误判为底盘平移。

## 主要功能

- 麦克纳姆轮 X/Y/YAW 运动学逆解
- 三轴位置/航向 PID 闭环
- X 轴调参时自动保持 Y 位移和起始航向
- OPS 丢失、主机心跳丢失、越界、交叉漂移等安全停车
- `POSE SET` 自动回到指定 OPS 位姿
- `POSE SET` 二维平移矢量梯形加减速、YAW 独立角加减速和制动距离限速
- 运行时 PID 参数及速度上限设置
- Python 串口采集、心跳、安全裁剪和结果记录

常用命令：

```text
HOST LINK COM
HOST LINK RPI
HOST STATUS
HOST RX STATUS
CONTROL STATUS
MOTOR FEEDBACK
MOTOR STOP STATUS
MODE WORK
MODE TUNE
MODE PLOT
MODE STATUS
PROTO VERSION
STATUS
PING
STOP
TELEM OFF|WHEEL|POSE|BOTH
TELEM STATUS
OPS STATUS
CAN STATUS
PID STATUS ALL
PID SET X|Y|YAW <p> <i> <d>
PID LIMIT X|Y|YAW <value>
TUNE AXIS X|Y|YAW
SET P:<p> I:<i> D:<d>
POSE SET <x_mm> <y_mm> <yaw_deg>
POSE STOP
G6220 STATUS
G6220 ENABLE
G6220 DISABLE
```

固件完成 UART、CAN、OPS 和执行器通信初始化后进入 `HOST WAIT`。四轮闭环
步进电机保持使能且目标速度为零，利用保持力矩防止外力造成车轮旋转或车体偏移；
G6220 保持失能。等待 60 秒只会输出一次超时提示，之后仍保持安全等待，绝不
自动进入运行态。PC 工具必须先发送 `HOST LINK COM`，树莓派必须先发送
`HOST LINK RPI`；握手成功后才开放原有命令和执行器。

`HOST LINK` 用于声明当前 USB 接入端类型，不做多主机争抢锁定。由于实际只有
一根 USB 线，后续 `HOST LINK COM/RPI` 可以安全切换类型；每次声明都会先停车、
清除旧运动状态并回到 `WORK`，绝不恢复上一连接的目标。`COM` 用于有人看护的
电脑调试，不要求周期心跳；`RPI` 用于比赛自主运行，`WORK/PLOT` 运动仍受
1.5 秒 `HOST LOST` 保护。

`TUNE` 模式不要求发送 `PING`：自动调参单轮最多运行 5 秒，每个调参会话最多
20 轮；用于返回起点的 `POSE SET` 最多运行 15 秒。主循环持续判定这些上限，
不使用阻塞 `while` 或 `HAL_Delay`；CAN 故障、OPS 丢失、方向错误、横向漂移、航向异常和越界保护
仍会立即停车。正式 `WORK`/`PLOT` 运动要求主机心跳，超过 1.5 秒未收到命令会触发安全停车。COM 串口同一时间只能由一个
程序占用，实车测试时必须保留物理急停。

G6220 只使用电机内部闭环的“位置-速度模式”，STM32 不实现其底层 PID。
固件启动后会初始化 CAN2 并发送使能命令；全局 `STOP`、底盘位姿安全故障
或离开 `WORK` 模式时会发送失能命令，重新执行 `MODE WORK` 时再次使能，
但不会恢复急停前的旧位置目标。`G6220_CAN_ID` 和 `G6220_MASTER_ID` 必须
与达妙上位机写入电机的参数一致。

## 独立运行模式

固件上电预置 `WORK`，但在 `HOST LINK` 成功前由独立等待门禁禁止所有业务和运动命令。三种模式通过同一套安全切换函数管理。每次切换都会先停止四轮、取消当前 POSE/调参/调试运动并清空 PID 内部历史，避免不同用途互相串扰：

- `MODE WORK`：正常工作模式。用于正式 `POSE SET`、PID 加载和状态查询；不允许 `MOTOR RUN`、`MOVE`、`TURN` 调试命令。
- `MODE TUNE`：PID 调参模式。允许调参轮次、返回起点的 `POSE SET`及调试运动；不要求心跳，但不豁免其他安全故障。
- `MODE PLOT`：曲线测试模式。允许 `MOTOR RUN`、`MOVE`、`TURN` 和 `POSE SET`，并要求心跳；不能启动调参轮次。

`MOTOR RUN` 用于底盘悬空时诊断单个电机，可在 `TUNE/PLOT` 使用。它不依赖 OPS 位姿，但仍受 STOP、CAN 发送/总线故障、模式切换、自身最长 10 秒超时以及 `PLOT` 心跳监督。其他底盘运动必须有有效 OPS。

为兼容现有命令，`TUNE AXIS X|Y|YAW`会自动切换到 `TUNE`；`PLOT ON`等价于进入 `PLOT`并开启两组遥测，`PLOT OFF`关闭遥测并返回 `WORK`。`STOP`只停止运动，不改变当前模式。正式运行前建议显式发送 `MODE WORK`。

当前主机文本协议版本为 `4`，可发送 `PROTO VERSION` 查询。V4在既有
`HOST LINK COM|RPI` 所有权握手之上增加二进制版本和能力协商。
`PID SET`和`PID LIMIT`是跨模式的非运动配置命令，便于WORK启动自检和TUNE装载参数；`TUNE LIMIT`以及会启动调参轮次的`SET P:... I:... D:...`仅允许在TUNE模式执行。`STATUS`保留 `MODE`、`HOST_PROTO`、`PLOT`和USART1发送统计字段，并增加当前 `HOST`。

正式 Raspberry Pi 位姿事务还可使用带 CRC、请求序号和 `goal_id` 的
[二进制协议](docs/binary-pose-protocol.md)。PC/TUNE 文本命令保持不变；RPI
必须先完成现有 ASCII 启动自检，再通过 `HOST BINARY START`和带
`VERSION/CAPS`的READY响应显式切换。切换后 `SerialBridgeThread` 仍是串口唯一
写者；速度档与位姿作为一条原子事务发送，另有QUERY、二进制遥测和冷启动会话
恢复。Mission、RouteRunner 与 Navigator 不直接接触串口。
链路丢失会停车、清除旧目标并退回 ASCII 自检状态，绝不恢复旧运动。

二进制协议的数值常量只有一个权威来源：
`protocol/rpi_binary_protocol.json`。修改版本、能力位、命令、事件或故障码后，运行：

```powershell
python tools/generate_rpi_protocol.py
python tools/generate_rpi_protocol.py --check
```

生成器同步维护 STM32 头文件、树莓派 Python 常量和 Markdown 数值表，避免两端
手工维护产生协议漂移。当前基线为 HOST protocol 4、binary wire 2、能力位
`0x0000003F`。

## 主机接收与退出停车

文本接收现在使用四项有界命令队列与独立 STOP 锁存，每行最多 63 字符。
STOP 清除旧命令；STOP 尚未出队时普通命令会被丢弃，客户端须等待停车回复后
再提交新业务。超长或含非法控制字符的行整行拒绝，不执行截断前缀。
`HOST RX STATUS` 可查看撤销/丢弃项及无效行计数。二进制 STOP 同样清除旧 RX 队列。

Pi 应用退出使用 `SerialBridgeThread.shutdown()`，由唯一串口线程发送 STOP，
限时等待回复后再关闭；分别报告写出、固件确认和线程退出状态。无回复或写失败
不能报告确认成功，固件确认也不等同于电机零速反馈。

## POSE 梯形速度规划

`POSE SET` 的控制链为：车体中心位姿误差 → X/Y/YAW PID → 主动制动速度上限 →
二维平移/YAW 加减速规划 → 全局速度转车体速度 → 麦克纳姆逆解 → 四轮 ZDT 电机。
控制周期为 20 ms，完全依赖 `HAL_GetTick()`推进，不使用阻塞延时生成斜坡。

每个 `POSE SET <x> <y> <yaw>` 在 STM32 内部严格分成两个阶段：

1. `TRANSLATE`：保持收到命令时的起始航向，只闭环移动到目标车体中心；
2. `ROTATE`：平移到位且规划线速度接近零后先发送四轮零速，再在目标坐标原地转到目标航向。

只有两个阶段都稳定完成后才发送一次 `# POSE TARGET`。因此树莓派仍只提交一个航点，
收到该终态后才能提交下一航点；无需连续发送速度，也不需要增加新的串口命令。

- 平移最大速度默认 `0.15 m/s`，可通过现有 `PID LIMIT X/Y` 调整；有效范围
  `0.02～0.30 m/s`，实际二维矢量模长取 X/Y 两轴上限的较小值。
- 平移加速度默认 `0.20 m/s²`，安全范围 `0.05～0.80 m/s²`；平移减速度默认
  `0.40 m/s²`，安全范围 `0.05～1.20 m/s²`。
- YAW 最大角速度默认 `0.30 rad/s`，可通过 `PID LIMIT YAW` 调整；有效范围
  `0.02～0.80 rad/s`。
- YAW 角加速度默认 `0.50 rad/s²`，安全范围 `0.10～2.00 rad/s²`；角减速度默认
  `0.80 rad/s²`，安全范围 `0.10～3.00 rad/s²`。
- 单航点相对当前车体中心的硬行程上限为 `10000 mm`；运行中位置误差超过
  `10500 mm` 视为越界并立即停车。

平移斜坡对完整的 `Vx/Vy` 差矢量限幅，不分别裁剪两轴，因此尽量保持 PID
要求的运动方向。接近目标时使用 `v = sqrt(2*a_decel*s_remaining)` 主动降低平移
速度；转向阶段使用同样的制动关系限制 YAW 角速度。到达容差且规划速度接近零后
才开始最终到位稳定计数。

正常 `POSE STOP` 为保持树莓派现有取消确认语义而立即零速。全局 `STOP`、模式切换、
OPS/HOST 丢失、CAN1 故障、非有限数值或行程越界同样直接清除规划状态并输出四轮零速，
不会经过减速斜坡，也不会在故障恢复后继续旧目标。PID 状态清理不会修改 Kp/Ki/Kd。

USART1 的 `printf` 使用有界行队列和 DMA：停车回复优先，普通回复 FIFO，W/P/调参遥测各保留最新值；DMA 缓冲在完成前保持不变。CAN1 每次最多提交 3 帧，四轮速度合并更新，STOP 清旧队列并优先提交零速。主机命令不再逐轮阻塞等待；UART DMA 忙超过 100 ms、CAN 普通帧排队超过 50 ms 均进入故障处理。

POSE/TUNE 的 PID 和斜坡统一使用实际 dt；PID 保留旧版 20 ms 参数基准，增加 20 ms 微分低通和下游限幅抗积分饱和。OPS 读取使用包含位姿、帧号及更新时间的完整快照。`CONTROL STATUS` 可检查循环间隔和发送拥塞。

四轮反馈超过 300 ms 未更新时禁止底盘非零速度。`MOTOR STOP STATUS` 将“零速已请求”与“收到新的四轮零速反馈”分开；600 ms 未确认则报告 UNCONFIRMED 并重试。启动完成后启用独立看门狗，名义约 2 秒未喂狗复位；**MCU 复位不保证外置驱动器停转，X42S 通信超时配置尚待核实并上板验证。** 设计及验收步骤见 [控制层验证说明](docs/control-layer-validation.md)。

## 树莓派视觉联调

树莓派可通过 SSH 运行视觉只读调试，不连接 STM32、也不产生运动命令：

```bash
cd pi-brain
python3 -m tools.vision_debug
```

需要验证“识别结果 → 地图航点 → 底盘闭环”时，使用联调入口：

```bash
python3 -m tools.vision_motion_debug --port /dev/serial/by-id/<STM32设备>
python3 -m tools.vision_motion_debug --port /dev/serial/by-id/<STM32设备> --armed
```

地图标准起点、车体中心目标和视觉条件位于
`pi-brain/config/vision_motion_debug.json`。程序用启动自检读到的实际 OPS 位姿
锚定固定地图；默认目标与标准起点相同，因此默认配置不会产生位移。未指定
`--armed` 时只能观察；指定后也不会自动发车，仍须自检进入 `READY`、视觉结果
满足类型/编号/来源/置信度/新鲜度门禁，并由现场人员输入 `go`。交互命令为：

- `status`：查看自检、路线和视觉健康状态；
- `go`：经 `RouteRunner → Navigator → SerialBridgeThread` 提交一个地图目标；
- `cancel`：取消当前路线并等待 STM32 停止确认；
- `estop`：锁存软件急停，必须重启程序并重新自检后才能再运动；
- `quit`：退出前先发送 STOP，再关闭相机和串口线程。

联调进程使用 `HOST LINK RPI` 和 1.5 秒固件心跳监督，且必须是摄像头和 STM32
串口的唯一所有者；不要同时运行另一份 `pi-brain`、绘图工具或串口助手。视觉
只选择固定地图目标，不直接发送 `POSE SET`，也不发送四轮速度。

## Python 两组 8 通道实时波形

项目不再依赖 SerialPlot 的单组二进制格式。COM/PLOT调试继续输出下面两类
ASCII行；RPI二进制会话使用CRC遥测帧，但映射为相同的Python事件类型：

```text
@W,1,tick,seq,target1,target2,target3,target4,actual1,actual2,actual3,actual4
@P,1,tick,seq,ops_x,ops_y,ops_yaw,center_x,center_y,plan_vx,plan_vy,plan_vz
```

- W8：四轮目标 RPM + 四轮反馈 RPM。后台始终每 10 ms 轮询一台电机，每台约 25 Hz，与遥测开关无关；W8 开启后输出 20 Hz。
- P8：OPS X/Y/YAW、车体中心 X/Y、规划 Vx/Vy/Vz；输出 20 Hz。
- `TELEM BOTH` 时 W8/P8 相差 25 ms 交错发送，避免同一控制周期连续发送两条长帧。
- 桥接器把遥测放入独立的有界“保留最新值”队列；STOP、故障、到位和心跳事件走原有控制队列，遥测积压不能阻塞安全事件。

安装依赖后，Windows 和树莓派使用同一绘图程序：

```powershell
cd pi-brain
python -m pip install -r requirements.txt
python -m tools.telemetry_plotter --port COM3 --mode work
```

树莓派将端口改为 `/dev/ttyUSB0`（以实际设备名为准）。工具同时显示两组曲线并分别保存 `wheel-*.csv`、`pose-*.csv`。命令行可直接输入 `POSE SET`、`MOTOR RUN` 等固件命令；输入 `STOP` 使用最高优先级急停发送。

绘图程序是串口唯一所有者，接收、普通命令、STOP 和心跳都通过同一发送队列串行化；不能与另一份 `pi-brain` 主程序或串口助手同时打开同一端口。`--mode work`和`--mode plot`自动发送心跳；`--mode tune`按固件设计不发送心跳，但仍保留 STOP/CAN/OPS/越界/轮次保护。

`TELEM OFF|WHEEL|POSE|BOTH`可在任意模式独立选择通道，`TELEM STATUS`查看序号及发送统计。旧的 `MODE PLOT`、`PLOT ON/OFF/STATUS`仍保留为文本协议兼容入口，但不再输出 SerialPlot 二进制帧。

## 固件架构

`main.c` 仅保留 CubeMX 外设初始化、时钟配置和应用入口，由 2681 行缩减到 194 行。业务状态收归 `robot_app.c` 私有，HAL 回调集中路由，接收协议与运动计算可独立在主机测试。

| 模块 | 职责与边界 |
| --- | --- |
| `main.c` | 初始化硬件，调用 `RobotApp_Init()` / `RobotApp_Process()` |
| `board_events.c` | HAL 中断及标准输出适配；不保存业务状态 |
| `robot_app.c` | 会话、模式、目标和安全调度；持有私有 PID/目标状态 |
| `host_rx_router.c` | ASCII/二进制接收分流、合法帧通知；不调用 HAL 或电机 |
| `host_command_rx.c` / `rpi_protocol.c` | 文本组行、帧校验、有界队列及 STOP 抢占 |
| `motion_math.c` | POSE/TUNE 共用坐标补偿、角度差、标量/二维斜坡；安装偏移只有一份 |
| `pid.c` / `llm_tuner.c` | 闭环计算与调参轮次 |
| `host_uart_tx.c` / `zdtCan.c` | 有界异步发送和拥塞监督 |
| `ops9.c` / `zdtEmm.c` / `motor_monitor.c` | 传感器与电机反馈、停车确认 |
| `control_runtime.c` | 周期间隔及 IWDG |

协议业务分派和 POSE 状态机暂留应用模块，后续通过类型明确的命令/状态接口进一步分离。模块所有权、ISR 规则和扩展步骤见 [架构说明](docs/architecture.md)。

## 构建

使用 STM32CubeIDE 1.19.0 打开工程并构建 `Debug` 配置。主要入口为：

- `Core/Src/main.c`
- `Core/Src/robot_app.c`
- `Core/Src/board_events.c`
- `Core/Src/host_rx_router.c`
- `Core/Src/motion_math.c`
- `Core/Src/rpi_protocol.c`
- `Core/Src/host_command_rx.c`
- `Core/Inc/rpi_protocol.h`
- `Core/Inc/rpi_protocol_generated.h`
- `Core/Src/mecanum_chassis.c`
- `Core/Src/pid.c`
- `Core/Src/zdtEmm.c`
- `Core/Src/zdtCan.c`
- `serialPlotTest.ioc`

新增源文件后，先在 STM32CubeIDE 中刷新工程并执行一次完整 clean build，确认
构建日志和 `Debug/Core/Src/subdir.mk` 已包含 `Core/Src/` 中全部源文件，尤其是
新增应用、接收路由、共享计算与控制监督模块，并检查 `Debug/objects.list` 中对应对象。构建完成后用
`arm-none-eabi-size Debug/serialPlotTest.elf` 记录固件 `text/data/bss` 以及
Flash/RAM 基线；旧 ELF 或未包含协议源文件的构建结果不能作为尺寸基线。

也可以在已安装 ARM GCC 与 newlib 的主机上直接构建，无须 CubeIDE 工作区或 `Debug/*.mk`：

```powershell
# arm-none-eabi-gcc 已在 PATH 中
python -B tools/build_firmware.py
# 或指定 CubeIDE 内置 GNU 工具链的 tools/bin 目录
python -B tools/build_firmware.py --toolchain-bin "<GNU工具链目录>/bin"
```

默认输出为 `tmp/firmware/serialPlotTest.elf`、map 和 `build.log`，目标为 F407 Cortex-M4 硬浮点、Debug `-O0`，C 编译警告视为错误。此命令仅构建，不烧录。

Python 工具位于 `llm-pid-tuner-main/`，建议使用其 `.venv` 环境运行测试：

```powershell
.\llm-pid-tuner-main\.venv\Scripts\python.exe -m unittest `
  llm-pid-tuner-main.tests.test_hw_bridge `
  llm-pid-tuner-main.tests.test_hardware_tui
```

树莓派主控、协议和视觉测试：

```powershell
cd pi-brain
python -m unittest discover -s tests -v
cd ..
python tools/generate_rpi_protocol.py --check
```

便携式 C 测试可在具有 GCC 的主机上运行，包含二进制协议、文本接收、接收路由、共享运动计算和控制层五套测试：

```powershell
python -B tools/run_c_tests.py
```

2026-09-09 验证基线为 Pi Python `192/192`、调参桥/界面 `17/17` 通过，
协议生成一致性及五套便携 C 测试通过；控制层包含 9 组场景，HAL 使用测试替身。调参工具 17 项是本地子模块验证结果，该子模块的未提交修改不在本次主仓库发布范围。

本轮通过可复现构建脚本编译链接 46 个单元、零警告，`text=100580`、`data=512`、`bss=29072` 字节；产物位于
`tmp/firmware/`，未烧录。CubeIDE 需刷新并完整构建。
真实 DMA/ISR、驱动器失联停车、IWDG 复位及 PID 效果仍须上板验证，详见
[控制层验证说明](docs/control-layer-validation.md) 和 [二进制协议](docs/binary-pose-protocol.md)。
`functions_reference.html` 为旧版函数索引，新控制模块以源码与交接文档为准。

`.github/workflows/ci.yml` 在 push/PR 时分别运行主仓库 C/Pi 回归和 ARM 全源码构建，不依赖本地 IDE 文件或调参子模块。远端执行状态以仓库 Actions 页面为准。

## 当前验证参数

```text
X/Y:  P=0.0033, I=0, D=0
YAW:  P=0.02,   I=0, D=0
低速上限: X/Y=0.10 m/s, YAW=0.15 rad/s
高速上限: X/Y=0.20 m/s, YAW=0.25 rad/s
```

此前低速实车验证中，X 横移约 195.81 mm，补偿后的前向漂移约 0.69 mm、偏航约 0.15°；YAW 转动约 29.03°，原始 OPS 位移约 13.87 mm，补偿后的车体中心位移约 2.62 mm。这些是本轮控制层修改之前的数据，不能作为新版固件验收结果。
