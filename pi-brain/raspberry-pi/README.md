# 智能搬运小车树莓派程序

项目按运行调度、比赛任务、硬件驱动、视觉算法、动作控制、公共服务和模拟测试
分层。详细边界见 `docs/ARCHITECTURE.md`。

## 目录

```text
robot_runtime/       主入口、状态机、接口和运行模型
robot_mission/       二维码任务文本解析与比赛规则
robot_hardware/      摄像头、GPIO、显示器和STM32通信
robot_perception/    二维码、颜色、巡线、物料和色环识别
robot_control/       导航、视觉对准、机械臂协调和恢复
robot_services/      安全、日志、遥测和统计
robot_simulation/    无硬件模拟组件
config/              运行、颜色和巡线配置
tests/               无硬件自动测试
tools/               人工调试、标定和离线评测入口
deployment/          systemd等部署脚本
docs/                架构与标定文档
references/          往届参考代码，不参与正式运行
```

## 无硬件模拟

```bash
cd ~/python
python3 -m robot_runtime \
  --simulate \
  --auto-start \
  --exit-on-terminal \
  --task-code "452+321+254+312"
```

`--auto-start` 只能用于模拟；正式比赛必须由唯一实体 Start 按钮触发。

## 自动测试

```bash
cd ~/python
python3 -m unittest discover -s tests -v
```

## 颜色识别调试

```bash
cd ~/python
python3 -m tools.debug_color --preview
```

颜色参数位于 `config/color.json`，离线标定见 `docs/COLOR_TUNING.md`。

## 树莓派5双摄像头

双摄像头统一配置位于 `config/cameras.json`：CAM/DISP0 上的夹爪相机
（`camera_num: 0`）用于物料颜色与中心定位；CAM/DISP1 上的前置相机
（`camera_num: 1`）用于巡线、导航和二维码。摄像头型号未确定时保持 `model: null`。

抓取相机对准调试：

```bash
python3 -m tools.debug_gripper --target-code 4
```

两路相机联合调试（默认无窗口，适合SSH）：

```bash
python3 -m tools.debug_dual_camera --mode all --target-code 4
```

这里的“摄像头1”是夹爪相机，“摄像头2”是车头导航相机；它们和 Picamera2
打印的 `camera_num` 不是同一个概念。联合程序让摄像头2的巡线与二维码算法
复用同一帧，避免重复打开或重复采集设备。

安装、编号核对和标定流程见 `docs/DUAL_CAMERA.md`。

## 巡线调试

```bash
cd ~/python
python3 -m tools.debug_line
```

无桌面环境时使用 `--no-preview`。巡线参数位于 `config/line.json`。

## OPS9 地图导航

OPS9 接在 STM32 上，由 STM32 转发统一位姿遥测；树莓派不再占用第二个 OPS9
串口。初始田字路网和点位在 `config/navigation.json`，OPS9 健康门限在
`config/ops9.json`，前视黑色障碍参数在 `config/obstacle.json`。

正式导航由树莓派规划航点、STM32 执行 20 ms 位姿闭环。每个航点使用带 CRC、
请求序号和 `goal_id` 的 v2 二进制事务。当前队友固件不支持旧连续速度命令，
必须使用 `stm32_pose_goal` 模式。

先测试 USB CDC 双向通信，不使能新的运动会话（遗留活动会话会先停车）：

```bash
python3 -m tools.stm32_link_test --port /dev/ttyACM0 --count 10
```

完整握手和位姿监视会进入 RPI 模式，可能使能底盘，必须架空车轮、准备急停。
设置 `config/stm32.json` 的实际设备路径后可运行 `python3 -m tools.stm32_ops9_monitor`。
USB CDC 配置、队友固件接口要求和测试步骤见
[`robot_hardware/stm32/README.md`](robot_hardware/stm32/README.md)。

地图点位、车体包络和相机单应矩阵都是待现场测量的初值；未完成标定前禁止启用
自动行驶。导航阶段已改为灰色可行域、黄白禁入区域和黑色圆柱检测，不再依赖
旧黑线巡线。实现与协议说明见 `docs/NAVIGATION_OPS9_DESIGN.md`，真实组件接线见
`docs/NAVIGATION_RUNTIME.md`。

## 树莓派开机启动

真实硬件组件接入完成后执行：

```bash
cd ~/python
sudo bash deployment/systemd/install_service.sh
```

硬件组件接口和安全约束见 `robot_runtime/README.md`。
