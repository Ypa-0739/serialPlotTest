# 树莓派—STM32 原生 USB CDC（队友协议 v2）

本目录是树莓派端代码。连接方案：树莓派 USB 主机口 → 数据线 → STM32 Type-C 数据口。
不使用 CP2102/FT232，不接 UART TX/RX；STM32 Type-C 必须实际连接 MCU USB D+/D-，
并运行 USB Device CDC ACM 固件。115200 仅为 CDC 串口 API 的逻辑参数。

本次适配依据队友提供的 `serialPlotTest-main (1).zip`：ASCII 主机协议版本4，
二进制协议版本2。队友负责将 UART 底层换为 USB CDC，同时保留本文协议。
仓库里的 `stm32_firmware/Comm` 是旧 v1 示例，本次没有修改它，不能与本目录 v2 混用。

## 树莓派联调

在项目根目录运行（先关闭其他占用串口的程序）：

```bash
sudo apt install python3-serial
python3 -m tools.stm32_link_test --list-ports
python3 -m tools.stm32_link_test --port /dev/ttyACM0 --count 10
```

第二条列出设备；第三条默认只用二进制 SESSION_PROBE，不发送 HOST LINK RPI，
不使能新的运动会话。全部返回 `PROBE ... OK`、最后出现 `PASS` 才证明 USB 数据传输
和 v2 请求/应答都正常。只出现 ttyACM0 不足以证明协议正常。
若检测到遗留活动会话，默认探测也会先 STOP_ALL 停车并静默等待恢复。

完整会话测试可能通过 HOST LINK RPI 使能电机，必须架空车轮、预留急停。
它不发送运动目标，但仍须先保证 CAN、电机和 OPS9 状态正常：

```bash
python3 -m tools.stm32_link_test --port /dev/ttyACM0 --handshake --count 50
python3 -m tools.stm32_link_test --port /dev/ttyACM0 --stop
```

完整测试出现连续 `PING ... OK` 与 `PASS` 表示握手及二进制 PING 成功。
`HOST BINARY NOT READY` 不一定是 USB 故障：默认探测通过而握手失败时，
优先检查 STM32 的 RPI 主机模式、WORK 模式、底盘使能、CAN 与 OPS9 就绪条件。

把实际设备路径写入 [config/stm32.json](../../config/stm32.json) 的 port；
推荐 `/dev/serial/by-id/实际设备名称`，不要保留“替换”占位符。
无权限时用 `sudo usermod -aG dialout "$USER"` 后重新登录。
没有 ttyACM 设备时检查数据线、D+/D-、USB CDC 固件和内核日志，不要修改树莓派 GPIO UART。

## 会话和安全语义

`SerialLink.open()` 默认执行：

1. 打开 CDC 设备，发 SESSION_PROBE，验证 VERSION=2 和能力位 0x3F。
2. 如果存在旧 active/armed 会话：STOP_ALL → 静默3秒 → 再探测确认已退出。
3. ASCII（每条 CRLF）：PROTO VERSION → HOST LINK RPI → STOP → MODE WORK → HOST BINARY START。
4. 验证 `# HOST BINARY READY VERSION=2 CAPS=0x0000003F`，完成二进制 PING，再确认 active/armed、host_link=2。
5. 启动唯一接收线程，100ms 间隔二进制 PING 心跳；未在500ms内收到对应 OK，应判链路失效。

`connected=True` 表示握手后的会话就绪，不仅是串口已打开。
`negotiate=False` 仅用于安全诊断；`port_open` 可以为 True，而 connected 仍为 False。
恢复等待必须超过 STM32 独立看门狗（队友代码为1.5秒）。
所有有效命令，包括 SESSION_PROBE，都会喂狗，所以恢复期间不能轮询或发送心跳。

USB 断开/心跳失败后自动重新探测和握手，绝不重放目标。
未完成的请求立即失败，旧位姿事务失效，目标 ID 使用随机起点避免进程重启碰撞。
运动恢复必须由上层明确决策。关闭链路停止心跳；物理停车必须由 STM32 独立看门狗完成，
不能把树莓派关闭设备当作已确认停车。本次没有更改 PID 参数和自动标定流程。

```python
import json
from robot_hardware.stm32 import SerialLink

with open("config/stm32.json", encoding="utf-8") as file:
    config = json.load(file)

# 该默认用法会完整握手，可能使能电机。
with SerialLink.from_config(config) as link:
    print(link.connected, f"RTT={link.ping() * 1000:.2f} ms")
```

## 二进制接口：交给 STM32 队友

字节流格式：`A5 5A | version:u8 | type:u8 | sequence:u8 | length:u16LE | payload | CRC:u16LE`。
version=2，payload 最大64字节，整帧最大73字节。
CRC16/CCITT-FALSE（poly=0x1021、init=0xFFFF），覆盖 version 到 payload；`123456789` 的 CRC=0x29B1。
USB 接收可能任意分片或多帧合并，不能把一个 USB 包当作一帧。

| 消息 | 类型 | payload |
|---|---|---|
| COMMAND | 0x10 | opcode:u8 + data |
| RESPONSE | 0x11 | request_sequence:u8 + opcode:u8 + status:u8 + data |
| EVENT | 0x22 | event_code:u8 + event_data |
| TELEMETRY | 0x23 | kind:u8 + telemetry_data |

没有 HEARTBEAT(0x21)；心跳是无参数 COMMAND PING(0x01)。
RESPONSE 的帧头 sequence 不用于匹配请求；必须使用 payload 里的 request_sequence 和 opcode。
状态码：0 OK、1 UNKNOWN_COMMAND、2 INVALID_LENGTH、3 INVALID_ARGUMENT、4 BUSY、5 INTERNAL_ERROR。

| 命令 | opcode | data / 响应 data |
|---|---|---|
| PING | 0x01 | 空 / 空 |
| STOP_ALL | 0x02 | 空 / 空；握手前也允许 |
| SESSION_PROBE | 0x03 | 空 / `<BBBBIIB>` 共13字节：active、armed、host_link、pose_state、goal_id、caps、version |
| SET_POSE_GOAL | 0x80 | `<IiiiI>`：goal_id、x_mm、y_mm、yaw_mrad、timeout_ms |
| CANCEL_POSE_GOAL | 0x81 | goal_id:u32 |
| QUERY_POSE_GOAL | 0x82 | 空 / `<IBiiiHBB>` 共21字节：goal_id、state、x/y/yaw、fault、robot_mode、host_link |
| SET_SPEED_LIMITS | 0x83 | `<ii>`：线速度µm/s、角速度µrad/s |
| SET_POSE_GOAL_WITH_LIMITS | 0x84 | `<IiiiIii>`：目标20字节 + 限幅8字节 |

导航发送原子命令0x84，速度上限不超过300mm/s、800mrad/s，目标超时1~60000ms。
0x10速度、0x20舵机、0x30状态、0x40任务码在队友当前 v2 固件未实现，
Python 接口明确拒绝，不能继续使用 legacy_velocity 模式。
caps 必须含 0x3F：状态查询、限幅、遥测、异步发送、会话恢复、原子目标+限幅。

事件：0x10 STARTED（goal_id）、0x11 REACHED（goal_id+5个i32）、
0x12 CANCELLED（goal_id）、0x13 MOTION_FAULT（goal_id+reason:u16）。
goal_id 必须匹配当前目标。故障6沿用队友 UART_FAULT 编号，USB适配后表示底层链路故障；
Python 本地 CANCEL_TIMEOUT 使用0x0100，避免与固件冲突。

### 遥测布局及限制

kind1轮速、kind2位姿均为39字节：kind:u8 + `<IH8i>`。
kind1字段为 tick、序号、4个目标RPM×10、4个实际RPM×10。
kind2字段为 tick、序号、OPS x/y/yaw、车体中心x/y、计划vx/vy/vz。
坐标mm、航向mrad，计划速度为µm/s、µrad/s。
kind3链路统计为25字节：kind:u8 + `<6I>`（tick、RX丢包、TX丢包、遥测替换、CRC错误、底层错误）。

`decode_telemetry` 可以解析三类遥测；`Stm32Ops9Receiver` 使用 OPS 坐标，不混用中心坐标。
v2没有质量、标定位和 OPS9 传感器更新时间，因此 quality=None，
状态标记 FIRMWARE_MONITORED 只表示依赖固件看门狗，不宣称传感器质量或标定已完成。
250ms新鲜度只验证遥测链路；不能据此证明 OPS9 数据本身持续更新。
STM32 必须独立检测 OPS9 失联、CAN 故障、越界及主机掉线并停车，拒绝无效运动目标。
树莓派收到运动故障后锁定位姿不可用于导航，直到新会话；现场标定保护仍保留。

```bash
# 也会完整握手/可能使能电机，请架空车轮，并先填写配置端口
python3 -m tools.stm32_ops9_monitor
```

STM32 USB 适配需把 ASCII 应答、二进制响应/事件/遥测统一送到同一个 CDC IN，
CDC OUT 同时支持 ASCII 启动阶段和 v2 流解析。TX 缓冲区在发送完成前保持有效，
处理 USBD_BUSY、排队和完成回调，不能丢 ACK，也不能让遥测堵塞紧急停车/响应。
USB拔掉后独立停车；必须实际测试“拔线”和“主机程序退出”，不能只测枚举。

## 无硬件验证

```bash
python3 -m unittest discover -s tests -v
```

模拟覆盖握手、任意分片、旧会话静默恢复、心跳超时、重连不重放、
响应匹配、协议版本/能力拒绝、原子目标、v2遥测和旧数据隔离。
这些测试不等价于真实 USB 硬件联调。
