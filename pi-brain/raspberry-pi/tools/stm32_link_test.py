"""测试树莓派 USB 主机口到 STM32 原生 USB CDC 的 v2 协议。"""

import argparse
import json
from pathlib import Path
import time

from robot_hardware.stm32 import Command, SerialLink, SerialLinkError
from robot_hardware.stm32.messages import SessionInfo


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/stm32.json")
    parser.add_argument("--port", help="覆盖配置端口，例如 /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, help="CDC 逻辑波特率，默认 115200")
    parser.add_argument("--list-ports", action="store_true", help="列出 USB CDC 设备后退出")
    parser.add_argument("--count", type=int, default=10, help="连续探测次数，默认 10")
    actions = parser.add_mutually_exclusive_group()
    actions.add_argument("--handshake", action="store_true", help="进入 RPI 二进制会话；可能使能底盘，请架空车轮")
    actions.add_argument("--stop", action="store_true", help="探测并发送 STOP_ALL，不使能新的底盘会话")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.count <= 0:
        raise SystemExit("--count 必须大于 0")
    try:
        if args.list_ports:
            from serial.tools import list_ports
            ports = list(list_ports.comports())
            for port in ports:
                print(f"{port.device}: {port.description} [{port.hwid}]")
            if not ports:
                print("未发现串口：检查数据线及 STM32 CDC 固件")
            return 0
        config = json.loads(Path(args.config).read_text(encoding="utf-8"))
        if args.port:
            config["port"] = args.port
        if args.baudrate:
            config["baudrate"] = args.baudrate
        if "替换" in config["port"]:
            raise SerialLinkError("请先设置 config/stm32.json 的 port，或指定 --port /dev/ttyACM0")
        with SerialLink.from_config(config, negotiate=args.handshake) as link:
            print(f"USB CDC 已打开：{link.port}")
            print(f"VERSION={link.session_info.version} CAPS=0x{link.session_info.capabilities:08X}")
            if args.stop:
                link.request(Command.STOP_ALL)
                print("STOP_ALL 已确认")
            else:
                for index in range(args.count):
                    if args.handshake:
                        latency = link.ping(timeout=float(config.get("command_timeout_seconds", 0.5)))
                        print(f"PING {index + 1}/{args.count}: OK，RTT={latency * 1000:.2f} ms")
                    else:
                        response = link.request(Command.SESSION_PROBE)
                        info = SessionInfo.decode(response.data)
                        link.validate_session(info)
                        if info.active or info.armed:
                            raise SerialLinkError("探测期间会话被其他程序占用；请关闭其他串口程序")
                        print(f"PROBE {index + 1}/{args.count}: OK，active={info.active} armed={info.armed}")
                    time.sleep(0.1)
                if args.handshake:
                    print("PASS：USB 双向通信、v2 握手和二进制 PING 均通过；未发送运动目标")
                else:
                    print("PASS：USB CDC 双向通信及 v2 协议探测通过；未使能新的运动会话")
            print(f"统计：{link.statistics()}")
            if args.handshake:
                link.request(Command.STOP_ALL)
        return 0
    except (SerialLinkError, OSError, ValueError, ImportError) as error:
        print(f"FAIL：{error}")
        return 1
    except KeyboardInterrupt:
        print("\n已退出；STM32 必须通过独立看门狗停车")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
