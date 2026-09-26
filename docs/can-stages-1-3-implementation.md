# CAN 与调参上位机前三阶段实施记录

本次在已有工作区修改基础上实施。未连接设备、发送运动命令或烧录。此前审查记录中“故障消费及多套解锁仍待处理”的描述，现由本文取代。

## 第一阶段：通信与轮次

- pi-brain 汇集 CAN STATE、READY、ESR 后判断就绪；HAL 累积 ERROR 保留作诊断。缺字段保持等待，不把 LISTENING 单独视为健康。模拟设备同步返回四行 CAN 状态。
- 串口控制台等到 CAN ESR 结束行，避免第一行返回后剩余状态混入下一条命令。
- 调参串口桥接统一写入与错误传播，设置 1 秒写超时并检查短写；读异常不再作为“暂时无数据”吞掉。
- 模式、协议版本、调参轴、PID 和限幅等待匹配应答；当前支持文本协议版本 4。初始化读取三轴 PID 快照，同步 YAW 基准。
- 事务等待期间的其他消息保留在接收队列，轮次切换不再清空串口缓冲。移除开始后固定 1 秒不读串口的等待。
- 所有轮次分支共用启动准入检查，覆盖初次、普通、回滚、VERIFY：最大轮数、停止和暂停请求都在发送前检查。
- 等待 ROUND START 使用独立 3 秒截止时间，持续 CSV 不会延长该等待。运行中的最大时长仍为 7 秒。
- 暂停发送 STOP 并确认停车，继续处理接收；恢复后清除已中断轮次的数据，重新请求新轮。LLM 返回后也重新检查准入。
- 动作验收检查用户中断，发生安全错误、无法确认停车或动作不合格时终止后续动作。

## 第二阶段：数据与结果

- CSV 至少具备 8 个基本字段且全部数值有限；本轮时间戳必须递增，PID 不得在轮内改变。缺失 PID 不再被默认值补成“设备参数”。
- `suggested_pid`、`loaded_pid`、`tested_result`、`verified_pid` 分别记录建议、装载、测试证据及验证结果。
- `final_pid` 与 `final_metrics` 来自同一完成轮次；`tested_result` 同时包含轴、轮次、PID、YAW 参数、停止原因及验证标记。
- 读取不到设备 PID 快照时保留缺失，不使用本地值冒充实机快照。
- 结果日志升级为版本 3。自动续调仅接受验证通过、PID/指标/轴绑定一致、反馈确认停车、动作验收无失败的记录。版本 1/2 日志保留但不再自动装载；用户可人工检查后显式提供初始参数。
- 退出记录 `stop_confirmation`，区分命令已发出与反馈确认停车；通信失败不能伪装成已确认停车。

## 第三阶段：故障与恢复

- `fault_event` 是可消费通知；`tx_fault` 是运动许可锁存；历史统计独立保存。
- `ConsumeFault` 只读取并清除事件。生产接口删除无条件 ClearFault，MOTOR RUN/TUNE/POSE 不再清除锁存后自行放行。
- 公共安全层读取当前故障；硬件不健康立即停车，软件传输故障沿用 100 ms 宽限，但不再被无事件循环清零。调参运行层不重复处理 CAN 门禁。
- 100 ms 的 TX_OK 观察仅显示链路恢复迹象，不解除运动锁存。
- 唯一解除入口为 `RecoverWhenIdle`：无活动运动、所需电机反馈新鲜、停车确认、TEC/REC=0、硬件无警告/被动错误/Bus-Off，且连续 500 ms 无新故障并有 TX_OK/RX 进展。
- 每次故障增加 `fault_generation`，新故障重置恢复观察。恢复不会自动重启旧运动或旧调参轮。
- CAN STATUS 保留原字段，增加 `GEN` 和 `REC_PHASE`。阶段 0 表示无锁存，1 表示故障待恢复，2 表示观察到链路进展，阶段 2 不代表运动许可。兼容字段 AUTO_REC 保留为 0。
- 测试专用清锁存入口仅存在于 CONTROL_HOST_TEST 构建，不暴露给固件。

## 未改变与验证边界

保留 PID 配置、300 ms 反馈门槛、停车确认、EWGF/EPVF/BOFF 门禁、CAN1 位时序、自动重发、50 ms 邮箱超时和 CAN2/G6220。第四阶段的查询调度拆分未实施。

软件测试覆盖启动回复缺失、最大轮数、分析期间停止、配置拒绝、短写、非有限 CSV、参数指标错配、暂停停车、CAN 多行解析、历史 ERROR、故障消费不解锁、新故障使恢复计时失效及停车槽保留。运行命令：

```text
python tools/run_c_tests.py
llm-pid-tuner-main/.venv/Scripts/python.exe -m unittest discover -s tests
python -m unittest discover -s tests
python tools/build_firmware.py --toolchain-bin <STM32 GCC bin>
```

两个 unittest 命令分别在调参上位机和 pi-brain 目录执行。固件产物在 `tmp/firmware/serialPlotTest.elf`。尚未证明实机 CAN 故障根因消除；烧录后需先验证停车与低速通信，再按原条件复测，并在复位前保存成对的 CAN STATUS/MOTOR FEEDBACK。
