"""Simulation application wiring all ROS-style components together."""

from dataclasses import dataclass
from math import cos, sin
from typing import Literal

from ultragps_sim.bus import SimpleBus
from ultragps_sim.controller import GoToGoalController
from ultragps_sim.messages import Pose2D, Twist2D, Waypoint
from ultragps_sim.pose_publisher import PosePublisher
from ultragps_sim.vehicle_simulator import DifferentialDriveSimulator
from ultragps_sim.waypoint_manager import WaypointManager

ScenarioMode = Literal["straight", "turn", "curve", "waypoint"]


@dataclass
class SimulationConfig:
    dt: float = 0.05
    max_steps: int = 500
    log_every_n: int = 20


class SimulationApp:
    def __init__(
        self,
        mode: ScenarioMode,
        config: SimulationConfig | None = None,
        waypoints: list[Waypoint] | None = None,
    ) -> None:
        self.mode = mode
        self.config = config or SimulationConfig()

        self.bus = SimpleBus()
        self.simulator = DifferentialDriveSimulator(
            self.bus,
            initial_pose=Pose2D(x=0.0, y=0.0, theta=0.0),
        )
        self.pose_publisher = PosePublisher(self.bus)

        self.controller = GoToGoalController(self.bus)
        default_waypoints = [Waypoint(x=2.0, y=2.0)]
        self.waypoints = list(waypoints or default_waypoints)
        self.waypoint_manager = WaypointManager(self.bus, self.waypoints)
        self.trajectory: list[Pose2D] = [self.simulator.pose]

        self.bus.publish("/ground_truth_pose", self.simulator.pose)
        self.waypoint_manager.publish_goal()

    def _record_pose(self, pose: Pose2D) -> None:
        self.trajectory.append(pose)

    def _run_open_loop(self) -> None:
        mode_to_cmd = {
            "straight": Twist2D(v=0.5, omega=0.0),
            "turn": Twist2D(v=0.0, omega=1.0),
            "curve": Twist2D(v=0.5, omega=0.5),
        }
        cmd = mode_to_cmd[self.mode]
        self.bus.publish("/cmd_vel", cmd)

        for step in range(self.config.max_steps):
            pose = self.simulator.step(self.config.dt)
            self._record_pose(pose)
            if step % self.config.log_every_n == 0:
                print(
                    f"step={step:03d} cmd=(v={cmd.v:.2f}, w={cmd.omega:.2f}) "
                    f"pose=(x={pose.x:.2f}, y={pose.y:.2f}, th={pose.theta:.2f})"
                )

    def _run_waypoint_closed_loop(self) -> None:
        for step in range(self.config.max_steps):
            cmd = self.controller.step()
            pose = self.simulator.step(self.config.dt)
            self._record_pose(pose)

            if step % self.config.log_every_n == 0:
                active = self.waypoint_manager.current_goal
                goal_str = "None" if active is None else f"({active.x:.1f},{active.y:.1f})"
                print(
                    f"step={step:03d} goal={goal_str} cmd=(v={cmd.v:.2f}, w={cmd.omega:.2f}) "
                    f"pose=(x={pose.x:.2f}, y={pose.y:.2f}, th={pose.theta:.2f})"
                )

            if self.waypoint_manager.completed:
                print(f"All waypoints completed at step {step}.")
                break

    def _render_trajectory_plot(
        self,
        ax,
        *,
        show_headings: bool = False,
        heading_stride: int = 10,
    ) -> None:
        if not self.trajectory:
            raise ValueError("Trajectory is empty")
        if heading_stride <= 0:
            raise ValueError("heading_stride must be positive")

        xs = [pose.x for pose in self.trajectory]
        ys = [pose.y for pose in self.trajectory]
        ax.plot(xs, ys, label="Trajectory", color="tab:blue")

        start = self.trajectory[0]
        final = self.trajectory[-1]
        ax.scatter([start.x], [start.y], color="tab:green", marker="o", s=80, label="Start")
        ax.scatter([final.x], [final.y], color="tab:red", marker="X", s=90, label="Final")

        if self.waypoints:
            ax.scatter(
                [waypoint.x for waypoint in self.waypoints],
                [waypoint.y for waypoint in self.waypoints],
                color="tab:orange",
                marker="^",
                s=70,
                label="Waypoints",
            )

        if show_headings:
            samples = self.trajectory[::heading_stride]
            if samples[-1] is not self.trajectory[-1]:
                samples = samples + [self.trajectory[-1]]
            ax.quiver(
                [pose.x for pose in samples],
                [pose.y for pose in samples],
                [cos(pose.theta) for pose in samples],
                [sin(pose.theta) for pose in samples],
                angles="xy",
                scale_units="xy",
                scale=4.0,
                width=0.003,
                color="0.25",
                label="Heading",
            )

        ax.set_title(f"UltraGPS Trajectory ({self.mode})")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.axis("equal")
        ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.6)
        ax.legend()

    def plot_trajectory(self, *, show_headings: bool = False, heading_stride: int = 10) -> None:
        try:
            import matplotlib.pyplot as plt
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "matplotlib is required for --plot. Install it and rerun the simulator."
            ) from exc

        fig, ax = plt.subplots()
        self._render_trajectory_plot(
            ax,
            show_headings=show_headings,
            heading_stride=heading_stride,
        )
        fig.tight_layout()
        plt.show()

    def run(
        self,
        *,
        plot: bool = False,
        show_headings: bool = False,
        heading_stride: int = 10,
    ) -> Pose2D:
        if self.mode in {"straight", "turn", "curve"}:
            self._run_open_loop()
        elif self.mode == "waypoint":
            self._run_waypoint_closed_loop()
        else:
            raise ValueError(f"Unknown mode: {self.mode}")

        final_pose = self.simulator.pose
        print(
            "Final pose: "
            f"x={final_pose.x:.3f}, y={final_pose.y:.3f}, theta={final_pose.theta:.3f}"
        )
        if plot:
            self.plot_trajectory(show_headings=show_headings, heading_stride=heading_stride)
        return final_pose
