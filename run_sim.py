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
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    app = SimulationApp(
        mode=args.mode,
        config=SimulationConfig(dt=args.dt, max_steps=args.steps),
        waypoints=args.waypoints,
    )
    app.run()


if __name__ == "__main__":
    main()
