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
        sim_dt: float = 0.05,
        update_rate_hz: float = 10.0,
    ) -> None:
        self.bus = bus
        self.position_noise_std = position_noise_std
        self.heading_noise_std = heading_noise_std
        self.sim_dt = sim_dt
        self.update_period = 1.0 / update_rate_hz
        self._elapsed_since_update = self.update_period
        self.latest_estimated_pose: Pose2D | None = None
        self.estimate_updated = False

        self.bus.subscribe("/pose", self._on_true_pose)

    def _on_true_pose(self, true_pose: Pose2D) -> None:
        self._elapsed_since_update += self.sim_dt
        if self._elapsed_since_update < self.update_period:
            self.estimate_updated = False
            return

        estimated = Pose2D(
            x=true_pose.x + gauss(0.0, self.position_noise_std),
            y=true_pose.y + gauss(0.0, self.position_noise_std),
            theta=true_pose.theta + gauss(0.0, self.heading_noise_std),
        )
        self._elapsed_since_update = 0.0
        self.estimate_updated = True
        self.latest_estimated_pose = estimated
        self.bus.publish("/estimated_pose", estimated)
