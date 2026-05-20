"""Simulation performance logging and summary metrics."""

import csv
from dataclasses import dataclass
from pathlib import Path


@dataclass
class StepMetrics:
    time_taken: float
    x: float
    y: float
    theta: float
    goal_x: float | None
    goal_y: float | None
    distance_error: float
    heading_error: float
    v: float
    omega: float
    waypoint_index: int
    goal_reached: bool


class SimulationLogger:
    """Collects per-step metrics and optionally writes them to CSV."""

    FIELDNAMES = [
        "time taken",
        "x",
        "y",
        "theta",
        "goal_x",
        "goal_y",
        "distance_error",
        "heading_error",
        "v",
        "omega",
        "waypoint_index",
        "goal_reached",
    ]

    def __init__(self, csv_path: str | None = None) -> None:
        self.csv_path = csv_path
        self.rows: list[StepMetrics] = []

    def record(self, metrics: StepMetrics) -> None:
        self.rows.append(metrics)

    def maybe_write_csv(self) -> None:
        if not self.csv_path:
            return
        path = Path(self.csv_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as file_obj:
            writer = csv.DictWriter(file_obj, fieldnames=self.FIELDNAMES)
            writer.writeheader()
            for row in self.rows:
                writer.writerow(
                    {
                        "time taken": row.time_taken,
                        "x": row.x,
                        "y": row.y,
                        "theta": row.theta,
                        "goal_x": row.goal_x,
                        "goal_y": row.goal_y,
                        "distance_error": row.distance_error,
                        "heading_error": row.heading_error,
                        "v": row.v,
                        "omega": row.omega,
                        "waypoint_index": row.waypoint_index,
                        "goal_reached": row.goal_reached,
                    }
                )

    def summary(self, final_position_error: float, waypoints_reached: int, total_waypoints: int) -> dict:
        if not self.rows:
            return {
                "final_position_error": final_position_error,
                "total_simulation_time": 0.0,
                "total_steps": 0,
                "waypoints_reached": waypoints_reached,
                "total_waypoints": total_waypoints,
                "max_distance_error": 0.0,
                "avg_distance_error": 0.0,
                "max_abs_heading_error": 0.0,
                "avg_abs_heading_error": 0.0,
            }

        distance_errors = [row.distance_error for row in self.rows]
        abs_heading_errors = [abs(row.heading_error) for row in self.rows]
        total_steps = len(self.rows)
        total_time = self.rows[-1].time_taken

        return {
            "final_position_error": final_position_error,
            "total_simulation_time": total_time,
            "total_steps": total_steps,
            "waypoints_reached": waypoints_reached,
            "total_waypoints": total_waypoints,
            "max_distance_error": max(distance_errors),
            "avg_distance_error": sum(distance_errors) / total_steps,
            "max_abs_heading_error": max(abs_heading_errors),
            "avg_abs_heading_error": sum(abs_heading_errors) / total_steps,
        }
