#!/usr/bin/env python3
"""Command-line entry point for the UltraGPS simulator."""

import argparse
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.join(ROOT, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from ultragps_sim.messages import Waypoint
from ultragps_sim.sim_app import SimulationApp, SimulationConfig


def parse_waypoints(value: str) -> list[Waypoint]:
    """Parse 'x1,y1;x2,y2;...' into waypoint objects."""
    waypoints: list[Waypoint] = []
    for pair in value.split(";"):
        pair = pair.strip()
        if not pair:
            continue
        x_str, y_str = pair.split(",")
        waypoints.append(Waypoint(x=float(x_str), y=float(y_str)))
    if not waypoints:
        raise argparse.ArgumentTypeError("At least one waypoint must be provided")
    return waypoints


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="UltraGPS ROS-style simulator")
    parser.add_argument(
        "--mode",
        choices=["straight", "turn", "curve", "waypoint"],
        default="waypoint",
        help="Simulation scenario mode",
    )
    parser.add_argument("--dt", type=float, default=0.05, help="Integration time step")
    parser.add_argument("--steps", type=int, default=500, help="Maximum simulation steps")
    parser.add_argument(
        "--waypoints",
        type=parse_waypoints,
        default=parse_waypoints("2.0,2.0"),
        help="Semicolon-separated waypoints, e.g. '1.0,0.0;2.0,2.0'",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Show a 2D x-y trajectory plot at the end of simulation.",
    )
    parser.add_argument(
        "--plot-output",
        type=str,
        default=None,
        help="Optional output image path (e.g., trajectory.png).",
    )
    parser.add_argument(
        "--heading-stride",
        type=int,
        default=15,
        help="Spacing between heading arrows on trajectory plot.",
    )
    parser.add_argument(
        "--plot-headings",
        action="store_true",
        help="Compatibility flag: heading arrows are already plotted when --plot is used.",
    )
    parser.add_argument(
        "--log-output",
        type=str,
        default=None,
        help="Optional CSV output path for performance logging.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    app = SimulationApp(
        mode=args.mode,
        config=SimulationConfig(dt=args.dt, max_steps=args.steps),
        waypoints=args.waypoints,
    )
    app.run(
        plot=args.plot,
        plot_output=args.plot_output,
        heading_stride=args.heading_stride,
        log_output=args.log_output,
    )


if __name__ == "__main__":
    main()
