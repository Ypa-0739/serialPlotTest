# STM32F407 麦克纳姆轮物流小车底层控制

基于 STM32F407VET6、四轮麦克纳姆底盘和 4 个 ZDT X42S 闭环步进电机的底层电控工程，包含 CAN 电机控制、OPS9 位姿反馈、X/Y/YAW PID 闭环以及串口调参与安全保护。

## 硬件与接口

- 主控：STM32F407VET6
- 电机：4 × ZDT X42S，CAN1 500 kbit/s，ID 1～4
- CAN1：PB8 RX、PB9 TX
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
- 运行时 PID 参数及速度上限设置
- Python 串口采集、心跳、安全裁剪和结果记录

常用命令：

```text
STATUS
PING
STOP
OPS STATUS
CAN STATUS
PID STATUS ALL
PID SET X|Y|YAW <p> <i> <d>
PID LIMIT X|Y|YAW <value>
TUNE AXIS X|Y|YAW
SET P:<p> I:<i> D:<d>
POSE SET <x_mm> <y_mm> <yaw_deg>
POSE STOP
```

调参过程中必须持续发送 `PING`，超过 1.5 秒未收到主机命令会触发安全停车。COM 串口同一时间只能由一个程序占用，实车测试时必须保留物理急停。

## 构建

使用 STM32CubeIDE 1.19.0 打开工程并构建 `Debug` 配置。主要入口为：

- `Core/Src/main.c`
- `Core/Src/mecanum_chassis.c`
- `Core/Src/pid.c`
- `Core/Src/zdtEmm.c`
- `Core/Src/zdtCan.c`
- `serialPlotTest.ioc`

Python 工具位于 `llm-pid-tuner-main/`，建议使用其 `.venv` 环境运行测试：

```powershell
.\llm-pid-tuner-main\.venv\Scripts\python.exe -m unittest `
  llm-pid-tuner-main.tests.test_hw_bridge `
  llm-pid-tuner-main.tests.test_hardware_tui
```

## 当前验证参数

```text
X/Y:  P=0.0033, I=0, D=0
YAW:  P=0.02,   I=0, D=0
低速上限: X/Y=0.10 m/s, YAW=0.15 rad/s
高速上限: X/Y=0.20 m/s, YAW=0.25 rad/s
```

低速实车验证中，X 横移约 195.81 mm，补偿后的前向漂移约 0.69 mm、偏航约 0.15°；YAW 转动约 29.03°，原始 OPS 位移约 13.87 mm，补偿后的车体中心位移约 2.62 mm。

