# 智能搬运机器人底层状态机

该目录提供可由 systemd 开机启动的任务编排层。它不会直接假定 GPIO、
电机板、机械臂或显示屏型号；真实硬件通过 `ComponentBundle` 注入。

## 安全边界

- 开机只初始化、自检并进入 `WAITING_FOR_START`，不会自动驱动车辆。
- 必须先观察到实体按钮释放，再检测到一次按下，防止开机时按钮卡住误启动。
- 比赛开始后持续检查急停、边界、堵转/电源等安全汇总。
- 默认在连续 14 秒没有检测到底盘或机械臂物理动作时提前安全停车。
- 所有长动作都有超时、有限重试和恢复；重试耗尽进入 `SAFE_STOP`。
- 任意未处理异常都会先取消导航、停止底盘并停止机械臂。
- 遥测接口只用于日志和观测，不能绕过唯一实体 Start 按钮控制比赛流程。

## 主状态流程

```text
BOOTING -> SELF_CHECK -> WAITING_FOR_START -> READING_TASK_CODE
  -> 第一批：转盘导航 -> 依序定位/抓取三件 -> 加工区依序放置
            -> 暂存区依序放置
  -> 第二批：转盘导航 -> 依序定位/抓取三件 -> 加工区依序放置
            -> 暂存区按物料编号对应堆叠
  -> REPORTING -> COMPLETED
```

任何阶段均可因安全条件、任务码超时、动作失败或长期无动作进入
`SAFE_STOP`。

## 在开发机运行模拟任务

从项目根目录运行：

```bash
python3 -m robot_runtime \
  --simulate \
  --auto-start \
  --exit-on-terminal \
  --task-code "452+321+254+312"
```

这里的 `--auto-start` 只用于无硬件模拟测试。树莓派正式配置不得使用它，
应当由 `StartButton` 的 GPIO 适配器提供实体按钮状态。

运行测试：

```bash
python3 -m unittest \
  discover -s tests -v
```

## 接入真实硬件

所有接口位于 `interfaces.py`：

- `StartButton`：GPIO 实体按钮与消抖。
- `TaskCodeReader`：摄像头二维码识别，返回完整任务码文本。
- `Display`：持续显示任务码、当前状态和最终统计。
- `MotionController`：电机/PWM/编码器底层停车与活动状态。
- `Navigator`：巡线、定位、路径规划、避障和逻辑区域导航。
- `MaterialPerception`：颜色、形状、物料位置和转盘目标定位。
- `Manipulator`：抓取到车载槽、加工放置、暂存和对应堆叠。
- `SafetyMonitor`：急停、边界、堵转、姿态和电池状态汇总。
- `StatisticsRecorder`：抓取、放置、堆叠和最终正确数。
- `Telemetry`：本地日志或只读遥测。
- `LightingController`：垂直向下照明控制。
- `RecoveryController`：丢线、丢目标和动作超时后的有界恢复。

创建自己的模块，例如：

```text
robot_hardware/
├── __init__.py
├── factory.py
├── gpio_button.py
├── motor_driver.py
├── navigation.py
├── perception.py
├── manipulator.py
└── display.py
```

在 `robot_hardware.factory` 中提供：

```python
def build_components(config):
    return ComponentBundle(
        start_button=...,
        task_code_reader=...,
        display=...,
        motion=...,
        navigator=...,
        material_perception=...,
        manipulator=...,
        safety=...,
        statistics=...,
        telemetry=...,
        lighting=...,
        recovery=...,
    )
```

然后修改 `robot_config.json`：

```json
{
  "component_factory": "robot_hardware.factory:build_components"
}
```

实际文件中应保留其他配置项。组件动作必须快速返回：动作未完成时返回
`ActionResult.running()`，完成时返回 `ActionResult.done()`，可恢复错误返回
`ActionResult.retryable()`，不可恢复错误返回 `ActionResult.fatal()`。同一方法会在
状态机循环中重复调用，硬件适配器必须保证幂等。

颜色识别算法位于 `robot_perception/color`，巡线识别位于
`robot_perception/line`，方向决策位于 `robot_control/line_navigation.py`。
真实硬件组件应在 `robot_hardware.factory` 中组装，状态机不直接导入调试脚本。

## 树莓派开机启动

确认项目位于树莓派本地目录并已完成真实硬件工厂配置，然后执行：

```bash
cd ~/python
sudo bash deployment/systemd/install_service.sh
```

安装脚本会根据当前项目路径、普通用户名和 `python3` 路径生成 systemd 服务，
并立即启用。服务默认只在异常退出时重启；正常完成后进程保持最终统计显示，
直到关机或手动停止。

常用命令：

```bash
systemctl status robot-runtime.service
journalctl -u robot-runtime.service -f
sudo systemctl restart robot-runtime.service
sudo systemctl stop robot-runtime.service
```

当前默认 `component_factory` 是 `null`，因此启动的是安全模拟组件且不会自动
按下 Start。完成真实硬件适配并配置工厂之前，它不会控制实际电机。
