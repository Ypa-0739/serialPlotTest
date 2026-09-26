#!/usr/bin/env python3
"""Safe, manual serial bridge for Codex hardware PID work.

This module is deliberately transport-only: it never chooses PID values and
never starts a round on its own.  Motion commands are rejected unless the
process is started with --allow-motion after an explicit human go-ahead.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import re
import sys
import threading
import time
from pathlib import Path

import serial


CSV_HEADER = [
    "elapsed_ms",
    "target",
    "input",
    "output",
    "error",
    "p",
    "i",
    "d",
    "ops_x",
    "ops_y",
    "ops_yaw",
    "cross_track",
    "yaw_delta",
    "hold_cross_output",
    "hold_yaw_output",
    "center_x",
    "center_y",
]

def is_motion_command(command: str) -> bool:
    """Return True only for commands that can request nonzero motion."""
    upper = command.strip().upper()
    if upper.startswith("PID "):
        # PID STATUS/SET are configuration or read-only commands; a numeric
        # PID triplet is the round-start form.
        rest = upper[4:].lstrip()
        return bool(rest) and (rest[0].isdigit() or rest[0] in "+-.")
    return upper.startswith(("SET P:", "MOTOR RUN", "MOVE ", "TURN ", "POSE SET"))


def now_stamp() -> str:
    return dt.datetime.now().astimezone().isoformat(timespec="milliseconds")


def is_csv_telemetry(line: str) -> bool:
    if line.startswith("#"):
        return False
    fields = [item.strip() for item in line.split(",")]
    if len(fields) != 17:
        return False
    try:
        for item in fields:
            float(item)
    except ValueError:
        return False
    return True


class Console:
    def __init__(self, port: str, baud: int, root: Path, allow_motion: bool) -> None:
        self.port_name = port
        self.baud = baud
        self.root = root
        self.allow_motion = allow_motion
        self.serial: serial.Serial | None = None
        self.stop_event = threading.Event()
        self.reader_thread: threading.Thread | None = None
        self.log_lock = threading.Lock()
        self.raw_file = None
        self.round_file = None
        self.round_writer = None
        self.round_number = 0
        self.session_dir: Path | None = None

    def open(self) -> None:
        self.session_dir = self.root / "logs" / "codex_serial" / dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.raw_file = (self.session_dir / "raw.log").open("a", encoding="utf-8", buffering=1)
        self.serial = serial.Serial(
            self.port_name,
            self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
            timeout=0.20,
            write_timeout=1.0,
        )
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()
        self.log("# LOCAL OPEN port=%s baud=%d" % (self.port_name, self.baud))

    def log(self, line: str, direction: str = "RX") -> None:
        record = f"{now_stamp()} [{direction}] {line}"
        with self.log_lock:
            if self.raw_file:
                self.raw_file.write(record + "\n")
                self.raw_file.flush()
        print(record, flush=True)

    def send(self, command: str) -> bool:
        command = command.strip()
        if not command:
            return False
        if not self.allow_motion and is_motion_command(command):
            self.log(f"BLOCKED motion command until --allow-motion: {command}", "LOCAL")
            return False
        if not self.serial or not self.serial.is_open:
            raise RuntimeError("serial port is not open")
        self.serial.write((command + "\n").encode("utf-8"))
        self.serial.flush()
        self.log(command, "TX")
        return True

    def read_one(self, timeout: float = 0.20) -> str | None:
        if not self.serial:
            return None
        old_timeout = self.serial.timeout
        self.serial.timeout = timeout
        try:
            payload = self.serial.readline()
        finally:
            self.serial.timeout = old_timeout
        if not payload:
            return None
        line = payload.decode("utf-8", errors="replace").rstrip("\r\n")
        self.log(line)
        return line

    def drain(self, duration: float = 1.0) -> None:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.read_one(timeout=min(0.20, max(0.01, deadline - time.monotonic())))

    @staticmethod
    def matches(command: str, line: str) -> bool:
        if command == "STOP":
            return line.startswith("# STOP MODE=") or line.startswith("# ROUND STOP HOST")
        if command == "HOST LINK COM":
            return line.split()[:5] == ["#", "HOST", "LINK", "COM", "OK"]
        if command == "PROTO VERSION":
            return line.startswith("# PROTO VERSION=")
        if command == "STATUS":
            return line.startswith("# STATUS ")
        if command == "OPS STATUS":
            return line.startswith("# OPS ")
        if command == "CAN STATUS":
            return line.startswith("# CAN ESR=")
        if command == "MOTOR MASK STATUS":
            return line.startswith("# MOTOR MASK=")
        if command == "PID STATUS ALL":
            return line.startswith("# PID ALL ")
        if command == "MOTOR STOP STATUS":
            return line.startswith("# MOTOR STOP STATE=")
        return False

    def command_and_wait(self, command: str, timeout: float = 3.0) -> list[str]:
        self.send(command)
        lines: list[str] = []
        deadline = time.monotonic() + timeout
        feedback_ids: set[int] = set()
        while time.monotonic() < deadline:
            line = self.read_one(timeout=min(0.20, max(0.01, deadline - time.monotonic())))
            if line is None:
                continue
            lines.append(line)
            if command == "MOTOR FEEDBACK":
                match = re.match(r"# MOTOR FEEDBACK ID=(\d+)\b", line)
                if match:
                    feedback_ids.add(int(match.group(1)))
                if feedback_ids >= {1, 2, 3, 4}:
                    return lines
            elif self.matches(command, line):
                return lines
        raise TimeoutError(f"timeout waiting for {command!r}")

    def preflight(self) -> bool:
        self.log("# LOCAL DRAIN START", "LOCAL")
        self.drain(1.0)
        required = [
            "STOP",
            "HOST LINK COM",
            "PROTO VERSION",
            "STATUS",
            "OPS STATUS",
            "CAN STATUS",
            "MOTOR FEEDBACK",
        ]
        try:
            for command in required:
                self.command_and_wait(command)
            self.command_and_wait("MOTOR MASK STATUS")
            self.command_and_wait("PID STATUS ALL")
            self.log("# LOCAL PREFLIGHT COMMANDS COMPLETE", "LOCAL")
            return True
        except Exception as exc:
            self.log(f"# LOCAL PREFLIGHT FAILED {exc!r}", "LOCAL")
            try:
                self.command_and_wait("STOP", timeout=2.0)
            except Exception as stop_exc:
                self.log(f"# LOCAL STOP ACK FAILED {stop_exc!r}", "LOCAL")
                self.safe_stop()
            for command in ("STATUS", "OPS STATUS", "CAN STATUS", "MOTOR FEEDBACK", "MOTOR STOP STATUS"):
                try:
                    self.command_and_wait(command, timeout=2.0)
                except Exception as diag_exc:
                    self.log(f"# LOCAL DIAGNOSTIC FAILED {command}: {diag_exc!r}", "LOCAL")
            return False

    def safe_stop(self) -> None:
        if not self.serial or not self.serial.is_open:
            return
        try:
            self.serial.write(b"STOP\n")
            self.serial.flush()
            self.log("STOP", "TX-SAFE")
        except Exception as exc:
            self.log(f"# LOCAL STOP WRITE FAILED {exc!r}", "LOCAL")

    def start_reader(self) -> None:
        self.reader_thread = threading.Thread(target=self._reader_loop, name="serial-reader", daemon=True)
        self.reader_thread.start()

    def _reader_loop(self) -> None:
        while not self.stop_event.is_set():
            line = self.read_one(timeout=0.20)
            if line is None:
                continue
            if line.startswith("# ROUND START"):
                self._open_round_csv()
            elif is_csv_telemetry(line):
                self._write_csv(line)
            elif line.startswith("# ROUND STOP"):
                self._close_round_csv()

    def _open_round_csv(self) -> None:
        self._close_round_csv()
        self.round_number += 1
        assert self.session_dir is not None
        self.round_file = (self.session_dir / f"round_{self.round_number:03d}.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.round_writer = csv.writer(self.round_file)
        self.round_writer.writerow(CSV_HEADER)
        self.round_file.flush()
        self.log(f"# LOCAL CSV OPEN round={self.round_number}", "LOCAL")

    def _write_csv(self, line: str) -> None:
        if self.round_writer is None or self.round_file is None:
            return
        self.round_writer.writerow([item.strip() for item in line.split(",")])
        self.round_file.flush()

    def _close_round_csv(self) -> None:
        if self.round_file:
            self.round_file.close()
            self.round_file = None
            self.round_writer = None

    def close(self) -> None:
        self.stop_event.set()
        self._close_round_csv()
        if self.serial and self.serial.is_open:
            try:
                self.safe_stop()
                time.sleep(0.25)
            except Exception:
                pass
            self.serial.close()
        if self.raw_file:
            self.raw_file.close()
            self.raw_file = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--skip-preflight", action="store_true")
    parser.add_argument(
        "--allow-motion",
        action="store_true",
        help="permit manually typed motion commands; requires prior human go-ahead",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    console = Console(args.port, args.baud, args.root, args.allow_motion)
    try:
        console.open()
        ok = args.skip_preflight or console.preflight()
        if not ok:
            print("PREFLIGHT FAILED; motion remains disabled.", file=sys.stderr, flush=True)
            return 2
        console.start_reader()
        print("READY: type one MCU command per line; Ctrl+C exits with STOP.", flush=True)
        if not console.allow_motion:
            print("MOTION LOCKED: PID/SET P:/MOVE/POSE SET are blocked.", flush=True)
        for raw in sys.stdin:
            command = raw.strip()
            if command:
                console.send(command)
    except KeyboardInterrupt:
        console.log("# LOCAL KEYBOARD INTERRUPT", "LOCAL")
    except Exception as exc:
        console.log(f"# LOCAL FATAL {exc!r}", "LOCAL")
        return 1
    finally:
        console.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
