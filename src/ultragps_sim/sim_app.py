"""Simulation application wiring all ROS-style components together."""

from dataclasses import dataclass
from typing import Literal

from ultragps_sim.bus import SimpleBus
from ultragps_sim.controller import GoToGoalController
from ultragps_sim.messages import Pose2D, Twist2D, Waypoint
from ultragps_sim.plotting import plot_trajectory
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
        self.waypoints = waypoints or [Waypoint(x=2.0, y=2.0)]

        self.bus = SimpleBus()
        self.simulator = DifferentialDriveSimulator(
            self.bus,
            initial_pose=Pose2D(x=0.0, y=0.0, theta=0.0),
        )
        self.pose_publisher = PosePublisher(self.bus)

        self.controller = GoToGoalController(self.bus)
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

    def run(
        self,
        plot: bool = False,
        plot_output: str | None = None,
        heading_stride: int = 15,
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
            plot_trajectory(
                self.trajectory,
                waypoints=self.waypoints if self.mode == "waypoint" else None,
                title=f"UltraGPS Trajectory ({self.mode})",
                heading_stride=heading_stride,
                show=plot_output is None,
                save_path=plot_output,
            )
            if plot_output:
                print(f"Saved plot to: {plot_output}")

        return final_pose
