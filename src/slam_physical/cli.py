"""Explicit acquisition, experiment and presentation commands."""

from __future__ import annotations

import argparse
from pathlib import Path

__all__ = ["main"]


def main():
    parser = argparse.ArgumentParser(
        description="BeautyMap historical-pose revision pilot"
    )
    commands = parser.add_subparsers(dest="command", required=True)
    acquisition = commands.add_parser("capture")
    acquisition.add_argument("--output", type=Path, required=True)
    acquisition.add_argument("--assets", type=Path, required=True)
    acquisition.add_argument("--frames", type=int, default=40)
    acquisition.add_argument("--gui", action="store_true")
    acquisition.add_argument(
        "--scenario", choices=["static", "departure"], default="static"
    )
    experiment = commands.add_parser("experiment")
    experiment.add_argument("--dataset", type=Path, required=True)
    experiment.add_argument("--output", type=Path, required=True)
    experiment.add_argument("--upstream", type=Path, required=True)
    experiment.add_argument("--revision-after", type=int, default=16)
    view = commands.add_parser("view")
    view.add_argument("--run", type=Path, required=True)
    reproduction = commands.add_parser("reproduce")
    reproduction.add_argument(
        "--method",
        choices=["beautymap", "dufomap", "conceptgraphs", "hovsg"],
        required=True,
    )
    reproduction.add_argument("--output", type=Path, required=True)
    reproduction.add_argument("--runtime", type=Path, required=True)
    reproduction.add_argument("--frames", type=int, default=8)
    args = parser.parse_args()
    if args.command == "capture":
        from slam_physical.capture import capture

        capture(
            args.output.resolve(),
            args.assets.resolve(),
            args.frames,
            args.gui,
            args.scenario,
        )
    elif args.command == "experiment":
        from slam_physical.experiment import run_experiment

        run_experiment(
            args.dataset.resolve(),
            args.output.resolve(),
            args.upstream.resolve(),
            args.revision_after,
        )
    elif args.command == "reproduce":
        from slam_physical.reproduce import reproduce

        reproduce(
            args.method, args.output.resolve(), args.runtime.resolve(), args.frames
        )
    else:
        from slam_physical.viewer import view

        view(args.run.resolve())


if __name__ == "__main__":
    main()
