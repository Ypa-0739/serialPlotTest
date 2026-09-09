# 控制层优化与上板验收（2026-09-09）

控制层修改涵盖异步发送、PID 时间处理、OPS 快照、电机反馈监督与 IWDG。随后完成的架构拆分见 [架构说明](architecture.md)，主机回归仍通过，尚未烧录；下表中的硬件结果均待填写。

## 运行规则

| 项目 | 实现及阈值 | 观察入口 |
| --- | --- | --- |
| USART1 | DMA 独立缓冲；普通 32 行、停车 4 行；W/P/CSV 三类最新值槽；单行最多 511 字节（含换行）；忙超过 100 ms 故障 | `CONTROL STATUS` 第二行 |
| CAN1 | 普通 FIFO 16 项，四轮最新速度槽、四轮 STOP 槽；每次最多提交 3 帧；普通等待超过 50 ms 故障 | `CONTROL STATUS`、`CAN STATUS` |
| 启动与使能 | 启动延时持续服务 CAN；每轮使能提交后至少间隔 5 ms 才发送其后续普通帧；STOP 可抢占 | CAN 抓包 |
| PID | 目标周期 20 ms，实际 dt；微分低通 tau=20 ms；软件下游同向受限时撤销本步积分 | 低速 POSE/TUNE 记录 |
| OPS | X/Y/YAW、帧号、更新时间在同一短临界区复制，恢复调用者 PRIMASK | `OPS STATUS`，便携 C 测试 |
| 轮速新鲜度 | 10 ms 查询一个电机；每台约 40 ms；超过 300 ms 无有效回复拒绝相应运动 | `MOTOR FEEDBACK` |
| 停止确认 | 零速离开邮箱后，每轮至少两个新样本，连续两次绝对转速不超过 1 RPM，且四轮均新鲜 | `MOTOR STOP STATUS` |
| 停止未确认 | 请求后超过 600 ms，标记 UNCONFIRMED；每 100 ms 重发零速；重试不刷新原请求计时 | `MOTOR STOP STATUS`、CAN 抓包 |
| 周期监督 | 主循环起点间隔超过 100 ms 锁存故障并停车；控制步间隔超过 25 ms 计入 LATE | `CONTROL STATUS` |
| IWDG | 启动等待完成后启用；仅耗时不超过 100 ms 的主循环末尾喂狗；名义约 2 s 复位 | `IWDG_RESET`，复位引脚/启动日志 |

以上诊断命令用于 ASCII 会话，包括 HOST WAIT。二进制会话内不插入文本；需要诊断时先停止并正常退出二进制会话，再查询。未更改 HOST v4、binary wire v2 或帧字段。电机反馈失联沿用二进制 CAN_FAULT，控制循环超时沿用 INTERNAL_ERROR。

`VALID=1` 表示曾收到合法电机反馈；结合 AGE_MS 判断是否新鲜。`MOTOR STOP STATUS` 的 FRESH 是四轮整体新鲜标志。新非零命令使上次停止状态回到 IDLE，IDLE 的 ELAPSED_MS 为 0；停止确认后如果反馈变为非零或过期，立即撤销 CONFIRMED。

软件发送成功、STOP 协议 ACK、POSE_REACHED 事件和电机反馈停止确认是不同证据。此次未把 STOP ACK 或 POSE_REACHED 改成物理停车确认；CONFIRMED 也仅证明最近驱动器报告零速，不能证明车体没有滑动或驱动器传感器没有故障。

## 参数与发送接口兼容性

现有 `PID SET`、配置文件中的 Kp/Ki/Kd 数值不迁移。保持 20 ms 基准计算：

```text
integral_candidate = integral + error * dt / 0.020
derivative_raw = (error - last_error) * 0.020 / dt
alpha = dt / (0.020 + dt)
derivative_filtered += alpha * (derivative_raw - derivative_filtered)
```

重置后第一步没有微分冲击；dt 非有限、非正或大于 100 ms 时 PID 返回零并清历史。主控制循环另有超时监督。只增加滤波和抗饱和，不改变物理输入输出单位。非零 Ki/Kd 的响应会改变，必须重新低速比较；默认滤波时间常数目前是代码参数，没有新增串口配置命令。

`PID_ApplyOutput()` 必须在同一步最终限幅后调用，传入与 PID 同坐标系、同正方向的指令；POSE 转回全局 X/Y，TUNE 主轴乘回方向符号。四轮等比限幅也计入反馈。反馈值不等于实测电机输出，内部驱动器斜坡没有纳入此模型。

`ZDT_CAN_Send_ExtId()` 等返回 0 的含义改为“入队成功”。异步故障通过 `ZDT_CAN_ConsumeFault()` 汇入底盘安全监督。CAN1 发送 API 仅由主循环调用；接收 ISR 只更新反馈和事件。STOP 申请撤销旧邮箱，但已经发到总线的帧无法撤回。只有确认四轮 STOP 请求均不再等待邮箱发送后，才开始等待新的零速反馈。

USART1 文本输出需以换行结束，支持一次 printf 被拆为多个 `_write` 片段。活动 DMA 缓冲不能重置或覆盖；必须先 HAL abort，再调用 `HostUartTx_Reset()`。二进制切换使用 `DiscardPending()`，保留活动缓冲，并持续监督该笔 DMA 是否超时。

## 验收步骤

先在架空台架执行失联和卡死测试，保留可独立切断驱动器的急停。实车试验使用此前低速上限；这里没有自动连接、烧录或发车脚本。

| 场景 | 操作和必须记录的结果 | 当前状态 |
| --- | --- | --- |
| 基础启动 | CubeIDE 刷新后完整编译；上电保持零速，检查四轮反馈 AGE_MS；HOST WAIT/TELEM OFF 下 SEQ 仍递增 | 待上板 |
| UART 突发 | 连续 HELP/状态查询叠加遥测；插入 STOP；普通范围内 HELP 完整，STOP 优先，DMA 字节不混帧；超出队列容量应故障停车 | 待上板 |
| 文本/二进制切换 | ASCII 回复 DMA 尚未完成时开始二进制握手；检查没有混写，强制延迟完成回调时能超时清会话 | 待上板 |
| CAN 邮箱拥塞 | 制造无 ACK/总线拥塞，记录主循环最大间隔及 CAN_WAIT_MAX_MS；确认主循环不轮询等待邮箱 | 待上板 |
| STOP 撤旧速度 | CAN 抓包记录旧速度与 STOP 边界；普通软件队列中的旧目标不再发送，四轮零速优先；不能要求撤回已上总线的帧 | 待上板 |
| 单轮失联 | 仅停止一轮速度回复，其余三轮正常；超过 300 ms 时底盘运动停止，不能因 CAN 外设 LISTENING 而继续 | 待上板 |
| 真实停车 | 记录 STOP 请求、四个零速 CAN 帧、两轮新反馈及外部测得的车体停车时刻；软件 ACK 不能冒充测量结果 | 待上板 |
| 停止未确认 | 一轮持续回报非零或不回报；600 ms 后必须 UNCONFIRMED、持续重试；补回两次新零速后允许确认 | 待上板 |
| 反馈失效 | 已 CONFIRMED 后注入非零/过期反馈；确认状态必须撤销 | 待上板 |
| OPS 连续更新 | OPS 高频输入时重复状态/目标操作；记录帧号、坐标及更新时间对应关系；坏浮点帧不更新位姿 | 待上板 |
| PID 限幅 | X/Y/YAW 分别测试 Ki=0 与非零、Kd 非零；触发制动、矢量/四轮限幅，比较释放限幅后的超调、到位时间和周期分布 | 待上板 |
| IWDG | 专用测试固件在正常进入主循环后人为卡死，不在 ISR 喂狗；记录复位时间及下次 IWDG_RESET=1；不要用会冻结看门狗的调试暂停代替 | 待上板 |
| 驱动器断通信 | 核对 X42S 固件版本与对应手册；配置并读回通信超时停机参数；让 MCU 停止 CAN 发送，实测四轮停机 | **配置未知，待核实** |

最后一项是故障闭环的必要条件：IWDG 只能复位 MCU，CAN1 中断/收发器/线缆异常时，软件没有路径保证外部驱动器收到零速。当前未找到这批驱动器的通信超时配置证据，也未发送推测的持久化配置命令。

IWDG 使用 F407 寄存器接口（PR=6、RLR=249），32 kHz LSI 时名义为 `256 * 250 / 32000 = 2 s`；实际时间随 LSI 偏差变化。配置与启动依据 [ST RM0090 的 IWDG 章节](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-407-415-417-437-455-469-application-note-stmicroelectronics.pdf)。初始化位于自定义模块，调用留在 main 的 USER CODE 区域，不依赖生成 HAL IWDG 文件。

## 已运行的本地验证

```powershell
python -B tools/run_c_tests.py
python -B tools/generate_rpi_protocol.py --check
```

五套 C 测试均通过，使用 `-std=c11 -Wall -Wextra -Werror`，包括架构重构新增的接收路由和共享计算套件。控制层套件直接编译八个真实模块，包含 9 组测试：PID dt/限幅；停止确认；CAN 排队/抢占；CAN 超时/使能等待/RX 预算；UART 缓冲所有权/优先级；UART 突发/错误；电机与底盘集成；OPS 快照/坏帧；周期超时。覆盖 32 位 tick 回绕、旧零速样本不得确认新停车、查询不得清空反馈、W/P 遥测不互相覆盖等回归场景。

Pi unittest 192 项、调参桥 9 项、调参界面 8 项通过。架构重构后 ARM GCC 13.3.rel1 按 Debug 参数全源码编译链接 46 单元、零警告，`text=100580`、`data=512`、`bss=29072`。产物和日志在 `tmp/firmware/`，未烧录。调参测试基于本地子模块工作树；主机替身不验证真实 DMA、中断调度或 IWDG 寄存器时序，上表不能据此改为已通过。
