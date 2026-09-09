"""Build all firmware sources without CubeIDE, Debug/*.mk, or a local workspace."""
from pathlib import Path
import argparse
import os
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--toolchain-bin", type=Path, help="Directory containing arm-none-eabi-gcc")
    parser.add_argument("--output", type=Path, default=ROOT / "tmp" / "firmware")
    args = parser.parse_args()
    suffix = ".exe" if os.name == "nt" else ""

    def tool(name: str) -> str:
        basename = "arm-none-eabi-" + name + suffix
        if args.toolchain_bin:
            return str(args.toolchain_bin.resolve() / basename)
        resolved = shutil.which(basename)
        if not resolved:
            parser.error(f"{basename} not found; use --toolchain-bin or add it to PATH")
        return resolved

    compiler, size_tool = tool("gcc"), tool("size")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    # Explicit source roots; never compile stale generated object lists or tests.
    sources = sorted((ROOT / "Core/Src").glob("*.c"))
    sources += sorted((ROOT / "Drivers/STM32F4xx_HAL_Driver/Src").glob("*.c"))
    sources += sorted((ROOT / "Core/Startup").glob("*.s"))
    includes = ["Core/Inc", "Drivers/STM32F4xx_HAL_Driver/Inc",
                "Drivers/STM32F4xx_HAL_Driver/Inc/Legacy",
                "Drivers/CMSIS/Device/ST/STM32F4xx/Include", "Drivers/CMSIS/Include"]
    target = ["-mcpu=cortex-m4", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard", "-mthumb"]
    objects = []
    with (output / "build.log").open("w", encoding="utf-8") as log:
        def run(command: list[str]) -> None:
            result = subprocess.run(command, cwd=ROOT, text=True, errors="replace",
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            log.write(result.stdout)
            log.flush()
            if result.returncode:
                print(result.stdout)
                raise SystemExit(result.returncode)

        for source in sources:
            # Preserve source paths to avoid basename collisions across modules.
            obj = output / "objects" / source.relative_to(ROOT).with_suffix(".o")
            obj.parent.mkdir(parents=True, exist_ok=True)
            flags = ["-g3", "-DDEBUG", "--specs=nano.specs"]
            if source.suffix == ".s":
                flags += ["-x", "assembler-with-cpp"]
            else:
                flags += ["-std=gnu11", "-DUSE_HAL_DRIVER", "-DSTM32F407xx", "-O0",
                          "-ffunction-sections", "-fdata-sections", "-Wall", "-Werror",
                          "-fstack-usage"]
                flags += ["-I" + str(ROOT / inc) for inc in includes]
            run([compiler, *target, *flags, "-c", str(source), "-o", str(obj)])
            objects.append(str(obj))
        elf = output / "serialPlotTest.elf"
        run([compiler, *target, "-g3", "--specs=nano.specs", "--specs=nosys.specs",
             "-T" + str(ROOT / "STM32F407VETX_FLASH.ld"), "-static",
             "-Wl,--gc-sections", "-Wl,-Map=" + str(output / "serialPlotTest.map"),
             "-u", "_printf_float", "-u", "_scanf_float", *objects,
             "-Wl,--start-group", "-lc", "-lm", "-Wl,--end-group", "-o", str(elf)])
    subprocess.run([size_tool, str(elf)], check=True)
    print(f"Firmware build passed: {len(sources)} units; output: {output}")


if __name__ == "__main__":
    main()
