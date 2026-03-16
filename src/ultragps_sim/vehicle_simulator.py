"""Differential-drive kinematic simulation."""

from math import cos, sin

from ultragps_sim.messages import Pose2D, Twist2D


class DifferentialDriveSimulator:
    """Integrates x, y, theta from commanded linear/angular velocity."""

    def __init__(self, bus, initial_pose: Pose2D) -> None:
        self.bus = bus
        self.pose = initial_pose
        self.cmd = Twist2D(v=0.0, omega=0.0)

        self.bus.subscribe("/cmd_vel", self._on_cmd_vel)

    def _on_cmd_vel(self, cmd: Twist2D) -> None:
        self.cmd = cmd

    def step(self, dt: float) -> Pose2D:
        x_dot = self.cmd.v * cos(self.pose.theta)
        y_dot = self.cmd.v * sin(self.pose.theta)
        theta_dot = self.cmd.omega

        self.pose = Pose2D(
            x=self.pose.x + x_dot * dt,
            y=self.pose.y + y_dot * dt,
            theta=self.pose.theta + theta_dot * dt,
        )

        self.bus.publish("/ground_truth_pose", self.pose)
        return self.pose
