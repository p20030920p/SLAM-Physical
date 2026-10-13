"""Stream native output and stop only the process group owned by this run."""

from __future__ import annotations

import os
import queue
import signal
import subprocess
import threading
from pathlib import Path

__all__ = ["check_stop", "run_logged"]


def check_stop(path: Path) -> None:
    if any((parent / "STOP").exists() for parent in [path, *path.parents[:3]]):
        raise InterruptedError("This run was stopped by 03_stop.cmd")


def run_logged(command: list[str], cwd: Path, log_path: Path, run: Path) -> int:
    check_stop(run)
    process = subprocess.Popen(
        command,
        cwd=cwd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    lines: queue.Queue = queue.Queue()

    def read_output():
        try:
            for line in process.stdout:
                lines.put(line)
        finally:
            lines.put(None)

    reader = threading.Thread(target=read_output, daemon=True)
    reader.start()
    try:
        with log_path.open("w") as log:
            while True:
                check_stop(run)
                try:
                    line = lines.get(timeout=0.2)
                except queue.Empty:
                    continue
                if line is None:
                    break
                print(line, end="", flush=True)
                log.write(line)
        return process.wait()
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        process.stdout.close()
        reader.join(timeout=1)
