# 本地视觉组件

视觉代码在独立工作线程中采集和检测，通过 `VisionService` 的有界队列发布
`VisionObservation`。本包不得导入 `serial_bridge`、`protocol`、`navigator`、
`route_runner` 或 `mission`。

任务层读取结果时应使用 `VisionObservationGate` 检查目标类型、来源、置信度和
单调时间戳有效期。分类结果只能选择预标定 `PoseGoal`；任何运动仍必须通过本地
`Mission`/`RouteRunner`/`Navigator` 链路提交。

`gripper.require_global_ready=false` 的含义是：夹爪近距离画面可能只看到一件
目标物料，因此允许“目标颜色多帧确认 + 已对准 + 画面不歧义”成为候选抓取结果；
它不表示视觉层可以自动抓取。最终动作仍由任务层和机构层决定。

障碍轨迹只做 0.5 秒短时保留，并保留最后一次真实观察的 timestamp。即使调用方
调大轨迹保留期，也必须用 `VisionObservationGate` 拒绝过期障碍。

树莓派 5 使用 Raspberry Pi OS 提供的 Picamera2。不要运行上游安装脚本，也不要
修改全局 Python 环境。道路阈值和障碍单应矩阵在现场标定前保持禁用。

只读调试（默认启用二维码和物料识别）：

```bash
python3 -m tools.vision_debug
python3 -m tools.vision_debug --material --target-code 4
python3 -m tools.vision_debug --qr --line
```

视觉—底盘人工联调使用独立入口。电脑可以通过 SSH 操作树莓派，但树莓派
进程仍以 `HOST LINK RPI` 独占 STM32 串口。`config/vision_motion_debug.json`
保存固定图纸中的标准发车位姿和地图车体中心航点；程序用启动自检读到的
实际 OPS 位姿自动建立本轮坐标变换，不要求现场逐点标定。默认模式只观察，
允许发车时显式使用 `--armed`，并在程序内输入 `go`：

```bash
python3 -m tools.vision_motion_debug --port /dev/serial/by-id/<STM32设备>
python3 -m tools.vision_motion_debug --port /dev/serial/by-id/<STM32设备> --armed
```

交互命令为 `status | go | cancel | estop | help | quit`。其中 `go` 只会在视觉
类型、目标编号、来源、置信度和新鲜度均合格时，把地图航点转换为本轮 OPS
目标，再经 `RouteRunner -> Navigator -> SerialBridgeThread` 提交；视觉模块
不会直接发送 `POSE SET` 或四轮速度。启动区人工放置误差会成为地图整体误差，
粗导航后仍应使用视觉精对准。进程退出会先请求 STOP，再关闭相机和串口线程。
