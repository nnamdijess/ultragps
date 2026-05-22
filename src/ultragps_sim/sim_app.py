"""Simulation application wiring all ROS-style components together."""

from dataclasses import dataclass
from math import atan2, sqrt
from typing import Literal

from ultragps_sim.bus import SimpleBus
from ultragps_sim.controller import GoToGoalController
from ultragps_sim.logger import SimulationLogger, StepMetrics
from ultragps_sim.messages import Pose2D, Twist2D, Waypoint
from ultragps_sim.plotting import plot_trajectory
from ultragps_sim.pose_publisher import PosePublisher
from ultragps_sim.ultragps_sensor import UltraGPSSensor
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
        use_estimated_pose: bool = False,
        position_noise_std: float = 0.0,
        heading_noise_std: float = 0.0,
    ) -> None:
        self.mode = mode
        self.config = config or SimulationConfig()
        self.waypoints = waypoints or [Waypoint(x=2.0, y=2.0)]

        self.use_estimated_pose = use_estimated_pose
        self.position_noise_std = position_noise_std
        self.heading_noise_std = heading_noise_std

        self.bus = SimpleBus()
        self.simulator = DifferentialDriveSimulator(
            self.bus,
            initial_pose=Pose2D(x=0.0, y=0.0, theta=0.0),
        )
        self.pose_publisher = PosePublisher(self.bus)

        self.ultragps_sensor: UltraGPSSensor | None = None
        if self.use_estimated_pose:
            self.ultragps_sensor = UltraGPSSensor(
                self.bus,
                position_noise_std=self.position_noise_std,
                heading_noise_std=self.heading_noise_std,
            )

        pose_topic = "/estimated_pose" if self.use_estimated_pose else "/pose"
        self.controller = GoToGoalController(self.bus, dt=self.config.dt, pose_topic=pose_topic)
        self.waypoint_manager = WaypointManager(self.bus, self.waypoints)

        self.trajectory: list[Pose2D] = [self.simulator.pose]
        self.logger = SimulationLogger()
        self.completed_steps = 0
        self.last_summary: dict | None = None

        self.bus.publish("/ground_truth_pose", self.simulator.pose)
        self.waypoint_manager.publish_goal()

    def _record_pose(self, pose: Pose2D) -> None:
        self.trajectory.append(pose)

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        while angle > 3.141592653589793:
            angle -= 2 * 3.141592653589793
        while angle < -3.141592653589793:
            angle += 2 * 3.141592653589793
        return angle

    def _record_step_metrics(self, step: int, cmd: Twist2D, pose: Pose2D) -> None:
        active = self.waypoint_manager.current_goal
        goal_x = active.x if active else None
        goal_y = active.y if active else None

        control_pose = self.controller.current_pose or pose

        if active:
            dx = active.x - control_pose.x
            dy = active.y - control_pose.y
            distance_error = sqrt(dx * dx + dy * dy)
            goal_heading = atan2(dy, dx)
            heading_error = self._wrap_to_pi(goal_heading - control_pose.theta)
        else:
            distance_error = 0.0
            heading_error = 0.0

        waypoint_index = len(self.waypoints) - 1 if self.waypoint_manager.completed else self.waypoint_manager._index

        estimated_pose = self.ultragps_sensor.latest_estimated_pose if self.ultragps_sensor else None

        self.logger.record(
            StepMetrics(
                time_taken=(step + 1) * self.config.dt,
                true_x=pose.x,
                true_y=pose.y,
                true_theta=pose.theta,
                estimated_x=estimated_pose.x if estimated_pose else None,
                estimated_y=estimated_pose.y if estimated_pose else None,
                estimated_theta=estimated_pose.theta if estimated_pose else None,
                goal_x=goal_x,
                goal_y=goal_y,
                distance_error=distance_error,
                heading_error=heading_error,
                v=cmd.v,
                omega=cmd.omega,
                waypoint_index=waypoint_index,
                goal_reached=self.controller.goal_reached,
            )
        )

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
            self._record_step_metrics(step, cmd, pose)
            self.completed_steps = step + 1
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
            self._record_step_metrics(step, cmd, pose)
            self.completed_steps = step + 1

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

    def _print_summary(self, summary: dict) -> None:
        print("Simulation Summary:")
        print(f"  final position error: {summary['final_position_error']:.4f}")
        print(f"  total simulation time: {summary['total_simulation_time']:.4f}")
        print(f"  total steps: {summary['total_steps']}")
        print(f"  waypoints reached: {summary['waypoints_reached']}")
        print(f"  total waypoints: {summary['total_waypoints']}")
        print(f"  maximum distance error: {summary['max_distance_error']:.4f}")
        print(f"  average distance error: {summary['avg_distance_error']:.4f}")
        print(f"  maximum absolute heading error: {summary['max_abs_heading_error']:.4f}")
        print(f"  average absolute heading error: {summary['avg_abs_heading_error']:.4f}")

    def run(
        self,
        plot: bool = False,
        plot_output: str | None = None,
        heading_stride: int = 15,
        log_output: str | None = None,
    ) -> Pose2D:
        self.logger = SimulationLogger(csv_path=log_output)

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

        final_goal = self.waypoints[-1] if self.waypoints else None
        if final_goal:
            dx = final_goal.x - final_pose.x
            dy = final_goal.y - final_pose.y
            final_position_error = sqrt(dx * dx + dy * dy)
        else:
            final_position_error = 0.0

        waypoints_reached = len(self.waypoints) if self.waypoint_manager.completed else self.waypoint_manager._index
        total_waypoints = len(self.waypoints)

        summary = self.logger.summary(final_position_error, waypoints_reached, total_waypoints)
        self.last_summary = summary

        self._print_summary(summary)

        self.logger.maybe_write_csv()
        if log_output:
            print(f"Saved log CSV to: {log_output}")

        return final_pose
