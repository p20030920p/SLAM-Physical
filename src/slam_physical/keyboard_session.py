"""TTY wrapper around the installed ROS keyboard node, with session stop handling."""

import signal
import subprocess
import sys
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix


def main():
    run = Path(sys.argv[1])
    executable = (
        Path(get_package_prefix("teleop_twist_keyboard"))
        / "lib/teleop_twist_keyboard/teleop_twist_keyboard"
    )
    log = (run / "keyboard-native.log").open("w")
    process = subprocess.Popen(
        [
            str(executable),
            "--ros-args",
            "-r",
            "cmd_vel:=/teleop/cmd_vel",
            "-p",
            "speed:=0.25",
            "-p",
            "turn:=0.5",
        ],
        stderr=log,
    )
    try:
        while process.poll() is None and not (run / "STOP").exists():
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        (run / "STOP").touch()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5)
        log.close()


if __name__ == "__main__":
    main()
