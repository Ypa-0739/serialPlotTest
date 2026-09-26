# 导航运行链路与接入说明

## 已接通的数据流

```text
OPS9
  -> STM32解析私有协议
  -> OPS9_POSE遥测
  -> Stm32Ops9Receiver
  -> 地图位姿/物理运动判断
                        \
前视摄像头单帧 ----------> 灰色可行域与黄白禁入检测
                        -> 黑色圆柱候选与三帧确认
                        -> 动态道路封闭
                        -> A*改道
                        -> Stm32PoseMapNavigator
                        -> SET_POSE_GOAL(goal_id, OPS9绝对位姿)
                        -> STM32 20 ms位姿闭环/PID/麦轮解算

未确认近障碍、越界、道路失效、OPS9失效、相机冻结或STM32断联
  -> NavigationSafetyMonitor
  -> SAFE_STOP
```

导航阶段使用 `DualCameraVisionController.observe_navigation()`，一帧只采集一次，
不会运行旧的黑线巡线算法。二维码阶段仍可用同一个前摄像头调用
`scan_task_code()`，夹爪相机逻辑不受影响。

## 真实组件注入

完成标定后：

```python
from robot_hardware.navigation import build_real_navigation

navigation = build_real_navigation()

components = ComponentBundle(
    navigator=navigation,
    motion=navigation.stack.chassis,
    safety=navigation.safety,
    # 以下组件继续填写你们真实的按钮、显示、机械臂等适配器
    start_button=...,
    task_code_reader=...,
    display=...,
    material_perception=...,
    manipulator=...,
    statistics=...,
    telemetry=...,
    lighting=...,
    recovery=...,
)
```

`NavigationRuntime` 负责统一打开前/夹爪摄像头和 STM32 USB CDC 设备、订阅 OPS9、位姿
事件、执行自检及关闭资源。`Stm32PoseGoalController.is_active()` 仍使用 OPS9
实际位姿变化，而不是“存在活动 goal”，因此堵转不会被误认为仍在运动。

`config/navigation.json` 的 `planner.control_mode` 默认是 `stm32_pose_goal`。只有在
台架回退调试时才改为 `legacy_velocity`；两种模式不能同时运行。

## 启动锁

以下配置任意一项仍是占位值，`build_real_navigation()` 都会拒绝启动：

- `config/stm32.json` 的真实 STM32 CDC `/dev/serial/by-id/...`；
- `config/navigation.json` 的实测点位、道路和车体包络；
- `config/ops9.json` 的起始坐标变换；
- `config/obstacle.json` 的前视相机地面单应矩阵；
- `config/road.json` 的现场灰、黄、白 HSV 阈值。

完成对应标定后才能将各文件的 `calibration_required`，以及地图中的
`nominal_map_requires_field_calibration` 改为 `false`。

## 制动公式

独立安全监控使用：

```text
d_stop = d_blind + v * t_reaction + v^2 / (2 * a_brake) + margin
```

参数在 `config/navigation_safety.json`。其中盲区、总延迟和满载最差制动减速度
必须实测，不能直接使用当前初值参加比赛。
