"""Build and run portable firmware tests with a host GCC (no hardware access)."""
from pathlib import Path
import argparse
import os
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default="gcc", help="Host C compiler executable")
    args = parser.parse_args()
    output = ROOT / "tmp" / "c-tests"
    output.mkdir(parents=True, exist_ok=True)
    suites = {
        "rpi_protocol": ["rpi_protocol"],
        "host_command_rx": ["host_command_rx"],
        "host_rx_router": ["host_rx_router", "host_command_rx", "rpi_protocol"],
        "motion_math": ["motion_math"],
        "control_layer": ["pid", "motor_monitor", "zdtCan", "zdtEmm",
                          "mecanum_chassis", "host_uart_tx", "ops9", "control_runtime"],
    }
    for name, modules in suites.items():
        executable = output / (name + (".exe" if os.name == "nt" else ""))
        command = [args.cc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-ICore/Inc"]
        if name == "control_layer":
            command += ["-DCONTROL_HOST_TEST", "-Itests/c"]
        command += [f"tests/c/test_{name}.c"]
        command += [f"Core/Src/{module}.c" for module in modules]
        command += ["-lm", "-o", str(executable)]
        subprocess.run(command, cwd=ROOT, check=True)
        subprocess.run([str(executable)], cwd=ROOT, check=True)
    print(f"All {len(suites)} portable C suites passed.")


if __name__ == "__main__":
    main()
