"""Simulated UltraGPS sensor producing noisy estimated pose."""

from random import gauss

from ultragps_sim.messages import Pose2D


class UltraGPSSensor:
    """Subscribes to true pose and publishes noisy estimated pose."""

    def __init__(
        self,
        bus,
        position_noise_std: float = 0.0,
        heading_noise_std: float = 0.0,
    ) -> None:
        self.bus = bus
        self.position_noise_std = position_noise_std
        self.heading_noise_std = heading_noise_std
        self.latest_estimated_pose: Pose2D | None = None

        self.bus.subscribe("/pose", self._on_true_pose)

    def _on_true_pose(self, true_pose: Pose2D) -> None:
        estimated = Pose2D(
            x=true_pose.x + gauss(0.0, self.position_noise_std),
            y=true_pose.y + gauss(0.0, self.position_noise_std),
            theta=true_pose.theta + gauss(0.0, self.heading_noise_std),
        )
        self.latest_estimated_pose = estimated
        self.bus.publish("/estimated_pose", estimated)
