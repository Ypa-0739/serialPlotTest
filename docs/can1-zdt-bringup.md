# CAN1 / ZDT_X42S 通讯排查记录

## 已核对的依据

- 本地 X42S V1.0.2 手册第 15、17、27、40–41、67、69 页：必须是 CAN 硬件版本，P_Serial=CAN1_MAP，500000 bit/s，固定校验 6B，扩展帧 ID=(地址<<8)|包序号。
- 本地厂家 F407 速度模式例程：Prescaler=14、BS1=4TQ、BS2=1TQ、SJW=1TQ；厂家关闭自动重发，本工程保留自动重发并限制邮箱驻留时间。
- 本地开发板 v5.3 原理图：板载 TJA1050、5V 供电，PB8/PB9 需通过板上的 CAN_RX/CAN_TX 跳线接通；U14 的 1/2 脚分别是 CANH/CANL，R6=120Ω。LED1 是 STM32 短路指示灯，另有可控红色 LED，不能仅凭颜色判断。
- 本工程 HAL 源码：AddTxMessage 只提交邮箱；TxMailboxXCompleteCallback 对应 TXOK；HAL ErrorCode 累积错误位，不能把它重复出现当作新的 ACK 错误次数。
- ST 的 [bxCAN 正常模式说明](https://community.st.com/stm32-mcus-60/using-can-bxcan-in-normal-mode-with-stm32-microcontrollers-part-1-151183)说明：正常模式发送需要另一节点提供 ACK；缺失 ACK 会产生确认错误。

## 时钟与改动

HSE=25 MHz，PLLM=25、PLLN=336、PLLP=2，SYSCLK=168 MHz；AHB=/1、APB1=/4，因此 CAN 时钟为 42 MHz。CAN 不使用 APB 定时器的倍频规则。

CAN1：42 MHz / [14 × (1+4+1)] = 500 kbit/s；采样点=(1+4)/6=83.33%。开始本次工作时 can.c 已有该时序，现补齐 .ioc 中显式 BS2/SJW 和诊断中断。CAN2 仍为 42 MHz / [3 × (1+11+2)] = 1 Mbit/s。

CAN1 开启 TX、SCE 中断及 HAL 通知，三个完成回调只统计 hcan1。保留原有全帧接收过滤器用于诊断，但只有扩展数据帧、DLC<=8 才交给电机解析器。

每个硬件邮箱提交后计时；超过 50 ms 仍 pending 就申请撤销，并丢弃软件普通命令和速度队列，报告故障，保留停车队列优先级。无软件待发任务时也检查邮箱；没有阻塞等待、反复重启 CAN 或修改 CAN2。撤销是异步申请，不能保证线上的帧必定未发出，也不等于电机已停车，停车仍由真实反馈确认。

保留 AutoBusOff 自动恢复。运动许可统一使用 `ZDT_CAN_IsReady()`，TUNE 启动前也检查 CAN 就绪。HAL 历史错误不再要求重启 MCU 才能解除：仅在没有活动运动、四轮反馈均新鲜且停车已确认时，观察连续 500 ms；期间硬件 TEC/REC=0、无告警/被动错误/Bus-Off、无新增错误回调/提交失败/邮箱超时/丢弃，并观察到新的发送完成和接收，才调用 HAL_CAN_ResetError 并解除传输故障锁存。配置/参数类 HAL 错误不会自动清除。恢复后输出 `# CAN RECOVERED MOTION=STOPPED`，不自动重启先前轮次或运动。

累计 ERR_LATCH、ACK_SEEN、BOFF_SEEN 和计数继续保留；ERROR 表示尚未解除的 HAL 错误，READY 表示当前总线许可，不包含 OPS 或电机反馈许可。单纯无 ACK 不保证一定进入 Bus-Off，须同时观察 ACK_SEEN 和 TEC/REC。若启动时过早发起调参而 OPS 尚未就绪，仍会拒绝，需要确认 OPS 后重新发起调参。

## CAN STATUS 字段

|字段|含义|
|---|---|
|READY / RECOVERIES|统一 CAN 就绪结果 / 静止条件下解除旧故障的次数；READY=1不代表所有运动条件都满足|
|STATE=2|HAL_CAN_STATE_LISTENING，已启动并可收发，不是 CAN_MODE_SILENT|
|TX_QUEUED|HAL 成功装入硬件邮箱次数|
|TX_OK|发送完成回调次数；总线 ACK 不证明指定电机执行了命令|
|TX_ERR|AddTxMessage 提交失败次数；不是所有总线错误总数|
|TX_ABORT|硬件撤销完成回调次数，包含 STOP 引起的撤销|
|TX_TIMEOUT|超时后成功申请撤销的邮箱数，不是撤销已完成数|
|ERR_CB / ERR_LATCH|CAN1 错误回调数 / 累积 HAL 错误位|
|ACK_SEEN / BOFF_SEEN|本次启动以来是否观察到 ACK / Bus-Off 错误，非事件次数|
|ESR / TSR|读取状态时的硬件寄存器快照|
|TEC / REC / BOFF / EPVF / EWGF|当前收发错误计数和总线错误状态|
|RX|HAL 收到的所有帧数量，仍需用反馈 VALID/SEQ 判断协议有效性|
|LAST|0=最近发送完成，2=提交失败，4=邮箱超时，5=总线错误；初值0不代表已发送|

## 单电机最小测试

1. **先排除共地后红灯异常。** 若为 LED1，断电检查供电和 GND；未确认原因前不带电测试。本次没有连接串口、烧录或驱动电机。
2. 断电后仅连接地址 1 的电机：板 CANH→R/A/H，CANL→T/B/L，板 GND→电机电源负极/GND。确认 PB8/PB9 的 CAN 跳线接通。电机使用自己的额定电源，电源正极不能接板 5V/3.3V。确认 CAN 硬件版本而非只有串口版本。
3. 确认总线只有两端终端电阻；板载已含 120Ω，不要再在板端并联一只。按实际电路核对断电 H/L 电阻，两只 120Ω 并联时约 60Ω。
4. 电机配置 P_Serial=CAN1_MAP、CAN_Baud=500000、ID_Addr=1、Checksum=固定6B；确认固件是 EMM 还是 X，保存并按厂家要求重启。仅能通过 USB-TTL/RS232/RS485 控制电机不能证明 CAN 可用。
5. 排除供电问题后烧录本次固件，沿用原串口参数。连接主机后使用下列只读查询，不发送 RUN/START。PROTO 根据实际固件选择 EMM 或 X；换协议会触发既有停止处理。

   ```text
   HOST LINK COM
   PROTO EMM
   CAN STATUS
   MOTOR GET 1
   MOTOR FEEDBACK
   ```

   `HOST LINK COM` 沿用项目既有使能行为；测试前使电机及机构处于安全静止状态。每隔约 1 秒重复 `CAN STATUS`、`MOTOR GET 1` 和 `MOTOR FEEDBACK`，比较计数。

6. `MOTOR GET 1` 发出速度与状态两条查询：

   ```text
   TX ExtID=0x100 IDE=EXT RTR=DATA DLC=2 DATA=35 6B
   RX ExtID=0x100 IDE=EXT RTR=DATA DLC=5 DATA=35 00/01 speed_H speed_L 6B
   TX ExtID=0x100 IDE=EXT RTR=DATA DLC=2 DATA=3A 6B
   RX ExtID=0x100 IDE=EXT RTR=DATA DLC=3 DATA=3A flags 6B
   ```

   固件仍后台轮询地址 1–4，并非总线上只有这两条帧；只接 1 号时，2–4 无应用回复是预期现象。一个正常 CAN 接收节点也可能 ACK 发给其他地址的帧，所以必须检查 1 号真实回复。

7. 成功标准：TX_OK 与 RX 持续增长，FREE 不持续为0，1号 VALID=1、SEQ持续增加；`MOTOR GET 1` 可打印 SPEED 和 STATE。EMM 速度单位 RPM，X 为0.1 RPM，解析器已作换算。VALID 只表示速度反馈新鲜，状态回复还需检查 STATE 输出。
8. 若只有 TX_QUEUED 增加而 TX_OK 不增加、ACK_SEEN=1：优先查电机 CAN 硬件版本、CAN1_MAP、供电、跳线、H/L、共地、终端和波特率。若 TX_OK 增加但没有有效回复：查地址、校验、实际帧和电机配置。FREE 恢复也可能只是超时撤销，不能单独作为连通证据。
9. 单机通过后逐台接入地址 2、3、4，分别 `MOTOR GET n`，四台 VALID 均为1且 SEQ持续增长后，再进行原 PID TUNE 流程。未放宽四轮反馈、OPS、停止确认和非零运动保护；单电机测试期间 TUNE 运行应继续被拒绝。

## 验证及文件清单

- `python tools/run_c_tests.py`：5 个测试套件全部通过，控制层11组；新增覆盖入邮箱不算完成、三个完成回调、CAN2回调隔离、ACK/BOF锁存、空队列邮箱超时、tick回绕及故障时丢弃运动队列。增加恢复窗口测试：无收发不恢复、新错误或失去停车条件重置观察窗口、硬件错误/非零TEC/控制器未启动/参数错误禁止恢复、tick回绕以及历史诊断保留。
- `python tools/build_firmware.py --toolchain-bin <CubeIDE ARM工具链目录>`：46个编译单元编译和链接通过，使用 `-Wall -Werror`。产物为 `tmp/firmware/serialPlotTest.elf`；不是已有 Debug 目录中的旧 ELF。
- `git diff --check` 通过。尚未进行实机 CAN 验证，不能认定硬件故障已解决。

本次修改：`Core/Src/can.c`、`Core/Src/zdtCan.c`、`Core/Inc/zdtCan.h`、`Core/Src/robot_app.c`、`Core/Src/llm_tuner.c`、`Core/Src/stm32f4xx_it.c`、`Core/Inc/stm32f4xx_it.h`、`serialPlotTest.ioc`、`tests/c/control_test_hal.h`、`tests/c/test_control_layer.c`、本文档。`tmp/` 下另有提取资料、检查图片、辅助脚本和编译测试产物。保留工作区原有修改，未执行破坏性 Git 操作。


## 实机日志复核

用户反馈：TX_OK=64912、RX=64321、FREE=3、TEC=REC=0，四轮 VALID=1，反馈年龄2–32ms；这证明采样时总线正在正常收发并已取得四轮速度反馈。ERROR/ERR_LATCH=0xA8EF 包含历史告警、被动错误、Bus-Off、填充、ACK、位错误和三个邮箱的仲裁丢失位。ERR_CB=246356、TX_TIMEOUT=293 为累计值，单张快照不能判断错误是否仍在增长，亦不能把全部历史异常归因为软件保护。

烧录本版后先发 STOP，等待停车及恢复条件满足，再间隔1秒查询两次 CAN STATUS 和 MOTOR FEEDBACK。预期 ERROR=0、READY=1，ERR_LATCH仍可非零，错误计数不再增长，TX_OK/RX/SEQ增长。确认 OPS LINK=OK 后，人工重新发起 TUNE。若仍有新错误或不能确认停车，保持禁止运动并继续查接线/供电/总线质量。

本轮新增/调整范围为 CAN 驱动就绪与恢复、robot_app 的统一调用、llm_tuner 启动与运行检查、测试 HAL 及恢复测试和本文档。未改串口命令协议、树莓派所有权/心跳、OPS解析、CAN2参数和反馈保护。

## PID tuner 启动竞态修复

实机曾出现 `ROUND STOP CAN FAULT` 在线路上先于同一轮的 `ROUND START`。原因有两层：启动轮次先调用四轮停车并撤销旧 CAN 邮箱，却立即进入 RUN；同时主机串口把 STOP 放入高优先级队列而 START 使用普通队列，破坏了事件顺序。Python tuner 又只在 `round_active=True` 时处理 STOP，因此忽略故障停止、接受后到的 START，最终误报“4秒未收到CSV”。

修复后 MCU 使用 WAIT→ARMING→RUN：参数更新只进入 ARMING，等待四轮停车确认、反馈新鲜、CAN 当前就绪后才输出 START；两秒仍无法满足时输出 `ROUND STOP CAN NOT READY` 或 `STOP NOT CONFIRMED`。ARMING 不产生非零速度。START 与 STOP 都进入同一个紧急串口 FIFO，保持产生顺序。Python tuner 下发每轮参数前清除旧串口回复，并用 `round_requested` 追踪待启动轮次；START 前到达的 STOP 现在立即以主控故障结束，不再伪装成 CSV 超时。

后续实机确认 ARMING 和 START 均能完成，但进入 RUN 后立即 CAN FAULT。原因是 HAL 的 ErrorCode 为累计位：总线仲裁丢失和孤立的 ACK/位/CRC/TX 错误也会留在其中；旧判断要求整个累计值为0，而且任意 ErrorCallback 都置停车故障。多节点 CAN 的仲裁丢失可由硬件自动重试，不能单独作为停车条件。

当前实现将错误分为两类：所有事件继续累积到 ERR_CB/ERR_LATCH；只有 Error Passive、Bus-Off、RX FIFO 溢出、HAL 超时/未初始化/参数/内部错误立即置故障。瞬时仲裁及协议错误由当前 ESR、50ms邮箱超时和300ms电机反馈新鲜度共同兜底。`ZDT_CAN_IsReady()` 不再被历史协议位永久阻塞，但仍检查控制器状态、当前 EWGF/EPVF/BOFF、致命锁存和配置错误。CAN 引起的 ROUND STOP 会附带 ERROR、ESR、FATAL_CB 和 TX_TIMEOUT 快照。

OPS 启动也存在相同的瞬时窗口：旧版 `LLM_TunerStartRound()` 在进入 ARMING 前立即检查100ms新鲜度，串口打开后若 OPS 接收刚恢复便直接输出 `ERROR OPS NOT READY`。现在 StartRound 先进入 ARMING，最多等待2秒；期间保持零速，只有 OPS、四轮反馈、停车确认和CAN全部就绪才输出 START。真实串口的 Python tuner 另外在任何 TUNE/PID 命令前重复请求 `OPS STATUS`，最多等待3秒拿到 `LINK=OK`；失败时不启动轮次。

## 掩码单电机测试下的 CAN NOT READY 与立即停车

现象：`CAN STATE=2 ERROR=0x00000000 FREE=3 TX_OK=19390 RX=6451`、`CAN ESR=0x00000000 TSR=0x1C000000 TEC=0 REC=0 BOFF=0 EPVF=0 EWGF=0`，即采样瞬间状态、寄存器、邮箱全部健康，且 `MOTOR FEEDBACK ID=1 VALID=1 SEQ` 持续增长；但 `MOTOR RUN` 报 `TX_EN=0 TX_RUN=0`（两条命令都成功入队、`0xF3` 也收到 `02` ACK）之后立刻出现 `MOTION STOP SAFETY TYPE=SINGLE_MOTOR REASON=CAN FAULT`，随后永久 `ERROR MOTOR RUN CAN NOT READY`。

按数据流逐段核对后确认协议侧没有缺陷：CAN1 位时序与厂家例程逐位一致；`F3 AB 01 00 6B`(DLC=5)、Emm `F6 dir velH velL acc snF 6B`(DLC=7)、`35 6B`/`3A 6B` 查询与手册 5.3.2 / 5.3.7 / 5.5.11 / 5.5.15 及厂家 `can_SendCmd` 的拆包规则完全一致（注意 X 固件是"方向 加速度 速度"，Emm 固件是"方向 速度 加速度"，顺序相反，两者均已正确）；`0x3A` 的 bit0/1/2/3 分别对应 `Ens_TF/Prf_TF/Cgi_TF/Cgp_TF`，与 `MOTOR STATE` 打印一致；CAN2 使用 bank 14、CAN1 使用 bank 0，无过滤器冲突。因此故障在软件保护逻辑，不在协议或位时序。

三处叠加造成永久锁死：

1. `tx_fault` 是粘滞标志，`ZDT_CAN_IsReady()` 把它当成"当前不可用"，而唯一清除路径是 `ZDT_CAN_RecoverWhenIdle()` —— 需要无运动、反馈新鲜、停车已确认，并连续 500ms 内计数不变且同时观察到新的发送完成与接收。任何一次历史上的邮箱超时、队列等待超时、`AddTxMessage` 失败或 `BOF/EPV` 错误回调都会把它置位，此后只有苛刻条件才能解锁。
2. `ChassisSafety_Process()` 写成 `if (!ChassisSafety_CanReady() || Mecanum_ConsumeCanTxFault())`。`CanReady()` 为假时右侧因短路不求值，故障既不消费也不上报；而 `motion == CHASSIS_MOTION_NONE` 时又提前 return。于是空闲期故障静默累积，一开始运动就集中爆发成 `CAN FAULT`，用零速命令把刚下发的转速覆盖掉 —— 这正是"指令发出去了但电机不转"的直接原因。
3. `Mecanum_ClearCanTxFault()` 只清底盘层 `can_tx_fault_latched`，不清 `ZDT_CAN` 内部的 `tx_fault`；所以 `MOTOR RUN` 和 `PID TUNE` 里那两处"先清故障再启动"实际没有生效。

另外 `HAL_CAN_ERROR_PARAM` 也被 `ZDT_CAN_IsReady()` 当作硬故障：`hcan1.ErrorCode` 是 `|=` 累积、只有 `HAL_CAN_ResetError` 才清，而 HAL 在正常运行路径上也会置这一位（`HAL_CAN_AddTxMessage` 命中 `TSR.CODE` 与 `TME` 位竞态、`HAL_CAN_GetRxMessage` 读到已空的 RX FIFO）。它同时让 `ZDT_CAN_RecoverWhenIdle()` 拒绝自动恢复，于是同样表现为"看不到任何 ESR/TEC 异常却永久 NOT READY"。

本轮改动：

- `zdtCan.c`：新增 `CanAutoClearFault()`，在 `ZDT_CAN_Process()` 入口执行自愈 —— 观察到 TXOK 计数增长（证明总线上确实有帧被 ACK 成功发送）且控制器状态干净，并保持 100ms 后自动清 `tx_fault`。判据用 TXOK 增长，因此总线真的断开时不会误清除。新增 `ZDT_CAN_ClearFault()` 供启动新运动前显式清除；`CAN_BLOCKING_HAL_ERRORS` 取代原掩码并从就绪判据中移除 `PARAM`（该位仍在 `CAN STATUS` 的 `ERROR/ERR_LATCH` 中完整可见）；`ZDT_CAN_IsReady()` 复用同一个健康判定；统计新增 `auto_recoveries` 与 `tx_fault`。
- `mecanum_chassis.c`：`Mecanum_ClearCanTxFault()` 现在同时清 `ZDT_CAN` 故障；`SetAllMotorsSpeed()` 的反馈检查与下发都跟随 `required_motor_mask`（`MASK=0x0F` 时与四轮行为完全等价）；后台轮询改用新增的 `Mecanum_ReportPollResult()`，入队失败不再升级为会触发停车的运动故障。
- `robot_app.c`：`ChassisSafety_Process()` 无条件先消费故障标志再判断，空闲期也清理；TUNE 轮次的反馈要求跟随掩码，POSE/整车仍要求四轮；`MOTOR RUN` 在判定前后各清一次历史发送故障（硬件级判据仍由 `ChassisSafety_CanReady()` 把关），并打印 `STATE/ERR/ESR/TX_TIMEOUT`；新增 `# CAN SAFETY TX_FAULT=... READY=... ERR=... ESR=...` 诊断；`MOTOR RUN` 后允许每个电机打印一次 `0xF6` 的 ACK，用于区分"电机没收到命令"和"收到命令但没转"；`SET P:I:D` 在非整车掩码下改为警告而非拒绝。
- `llm_tuner.c`：ARMING 阶段的反馈要求由硬编码 `0x0F` 改为 `Mecanum_GetRequiredMotorMask()`（这正是单电机下 TUNE 永远卡在 ARMING 的直接原因）；`MOTOR FEEDBACK NOT READY` 现在附带 `MASK/FRESH` 快照；`LLM_TunerStartRound()` 在停车后清一次历史发送故障，避免 ARMING 被瞬时故障挡住。

注意：本次放宽了"单电机掩码下 TUNE 是否可运行"。此前约定单电机测试期间 TUNE 应被拒绝；现在 ARMING 跟随掩码，使单电机台架验证可以进入 RUN。**整车 PID 调参仍必须使用 `MOTOR MASK 0x0F`**，因为单轮驱动下的运动学行为与四轮完全不同。

验证：`python tools/run_c_tests.py` 5 个套件全部通过，控制层 12 组；新增覆盖显式清除、TXOK 增长后 100ms 自愈、观察窗口内不放行、无成功发送不误清除、控制器不干净不累计证据、以及 `PARAM` 不再阻塞就绪。全部 `Core/Src` 编译单元通过 `-Wall -Wextra -Werror` 的 C 层检查（本机无 ARM 工具链，仅主机 gcc 语义检查，未做固件链接与实机验证）。

## PID 轮次中途 CAN FAULT 与 CAN RECOVERED 刷屏

四轮全部上电后的实机表现分两段。好消息在前：手动 `PID 0.0033 0.0 0.0` 之后 `ROUND START` 成功，CSV 连续输出 2.34 秒，input 从 0 升到 181mm、output 饱和在 0.15 后按刹车限幅回落，说明 CAN 通信、四轮驱动与 PID 闭环都已正常。随后在 2400ms 处被打断：

```text
# CAN SAFETY TX_FAULT=1 READY=1 ERR=0x00000058 ESR=0x12000000 TX_TIMEOUT=10 AUTO_REC=0
# ROUND STOP CAN FAULT ERROR=0x00000058 ESR=0x12000000 FATAL_CB=0 TX_TIMEOUT=10
```

同时空闲期 `# CAN RECOVERED MOTION=STOPPED MASK=0x0F` 持续刷屏。两件事根因不同：

**CAN RECOVERED 刷屏**是 `ZDT_CAN_RecoverWhenIdle()` 的进入条件写成 `(!HAL_CAN_GetError(&hcan1) && !tx_fault)`，只要 `ErrorCode` 非零就走恢复窗口。而 `ERR=0x58` 是纯粹的接收侧事件（bit3 STF 位填充、bit4 FOR 格式、bit6 BR 位隐性），`ESR=0x12000000` 显示 `REC=18 / TEC=0 / EWGF=EPVF=BOFF=0 / LEC=0`——总线在正常收发，只是偶发接收错误让 `ErrorCode` 反复累积，然后每 500ms 被 `HAL_CAN_ResetError` 清一次。这一条同时说明 `HAL_CAN_ErrorCallback` 并未参与本次 `tx_fault` 的置位（`0x58` 不含任何 `CAN_FATAL_HAL_ERRORS` 位）。

**`TX_FAULT=1 READY=1`** 说明硬件判据全部通过，`tx_fault` 只能来自 `ZDT_CAN_Process()`——主循环一次抖动就足以让在途邮箱或等待队列被判 50ms 超时，置位一次 `tx_fault`，随后被 `ChassisSafety_Process()` 消费并立即停车。`TX_TIMEOUT=10` 说明这类事件是孤立的，不是持续故障。

三处修改：

- `robot_app.c`：`ChassisSafety_Process()` 对 `tx_fault` 引入 `CAN_FAULT_GRACE_MS`(100ms) 宽限期——只有故障每轮都被重新置位、持续满 100ms 才停车；单次被下一轮清掉的孤立事件不再打断正在进行的轮次。真正的断线/无 ACK 会持续置位，照样按时停车。空闲期(`motion==NONE`)复位该状态，避免历史故障在运动刚开始时爆发；`ChassisSafety_Stop()` 也复位它。`# CAN SAFETY` 增加 `STREAK_MS` 字段，用于区分离散事件与持续故障。
- `zdtCan.c`：`ZDT_CAN_RecoverWhenIdle()` 的进入条件改为 `(!tx_fault && !(HAL_CAN_GetError(&hcan1) & CAN_FATAL_HAL_ERRORS))`，纯接收错误不再触发 500ms 恢复窗口，消除刷屏。
- `zdtCan.c`：`HAL_CAN_ErrorCallback()` 把 `EPV`/`BOF` 从累积的 `ErrorCode` 中剥离，改由实时 `ESR` 的 `EPVF`/`BOFF` 重建。这与上一节移除 `PARAM` 是同一类缺陷——用累积值判断当前状态。本次它未参与置位，但早期日志里历史 `EPV` 确实让每一次普通错误回调都锁存 `tx_fault`。

上位机侧另外修了 `llm-pid-tuner-main/tuner.py:737`：`important_status` 转发白名单缺 `# CAN`，导致 `# CAN SAFETY`（CAN 故障的具体判据）与 `# CAN RECOVERED`（自愈频率）被静默丢弃，现场只能看到 `# ROUND STOP CAN FAULT` 这个结论而看不到原因。补上后即可直接判读。该改动只影响上位机，不需要重新烧录固件。

另有两个非阻塞观察项：`REC=18` 提示总线上存在偶发接收错误（远低于 error warning 的 128，建议观察是否继续增长，若稳定可忽略）；`ZDT_CAN_Process()` 的 50ms 超时与 CAN1 的 `AutoRetransmission=ENABLE`（厂家例程为 `DISABLE`）叠加仍偏敏感，宽限期只是缓解而非根治。

验证：`python tools/run_c_tests.py` 5 个套件全部通过、控制层 12 组；`test_can_diagnostics_and_mailbox_expiry` 与 `test_can_stopped_recovery` 已同步为"累积 BOF 不锁存 / 实时 ESR 的 BOFF 才锁存"的新语义。全部 `Core/Src` 编译单元通过 `-Wall -Wextra -Werror` 的 C 层检查（本机无 ARM 工具链，未做固件链接）。

**实机验证通过**：烧录后 PID 自动调参连续三轮全部正常结束，未再出现 `ROUND STOP CAN FAULT`，`CAN_FAULT_GRACE_MS` 宽限期、`RecoverWhenIdle` 判据收紧、`HAL_CAN_ErrorCallback` 实时判据三项修改均在实机上得到确认。至此 `MOTOR RUN` 单电机、四轮反馈、整车轮次调参三条路径全部走通。

## 2026-09-23 X 轮次反馈超时

最新 X 正向轮次在约 600 ms、位移约 21.5 mm 时以 `ROUND STOP MOTOR FEEDBACK LOST` 结束；原始 CSV 保存在 `llm-pid-tuner-main/logs/round_csv/20260923_182927_048284_X/round_001.csv`。停止前主轴输出为 0.120 m/s，航向变化约 +0.95°，YAW hold 输出约 -0.0192 rad/s。本轮尚未达到 0.2 m/s 主轴限幅，也没有 YAW hold 饱和证据。

停车后间隔约 1 秒的两组只读状态显示：四轮 `AGE_MS` 均为 6–36 ms、`SEQ` 各增长约 29–30，`TX_OK/RX` 各增长 118；`TX_TIMEOUT=140`、`STALL_REC=8`、`CAN_DROP=270` 没有继续增长，当前 `ESR=0`、`TEC=REC=0`。这只能证明停车后反馈恢复且持续新鲜，不能反推出停止瞬间是哪一路超过 300 ms。手动前后左右运动成功同样不能排除调参时的瞬时反馈中断。

为定位下一次故障，在 `ChassisSafety_Process()` 的反馈失效分支增加 `# CAN FEEDBACK LOST MASK=... FRESH=... AGE_MS=ID1,ID2,ID3,ID4 SEQ=...`，随后仍按原有规则停车。诊断行和 `ROUND STOP` 都走串口紧急队列，正常情况下主机先收到故障快照再收到停止事件。`FRESH` 中清零的位对应失去新鲜反馈的电机（ID1=BL、2=FL、3=FR、4=BR）。未改 300 ms 门槛、PID 参数、CAN2 或恢复判据。该诊断目前仅完成主机 GCC 语法检查和便携 C 测试；须在 STM32CubeIDE 编译、烧录后才会出现在实机日志中。
