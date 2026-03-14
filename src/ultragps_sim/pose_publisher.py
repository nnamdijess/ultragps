"""Pose publisher / estimator placeholder."""

from ultragps_sim.messages import Pose2D


class PosePublisher:
    """Currently republishes ground-truth pose as system pose.

    Later this can be replaced by a localization estimator (e.g., UltraGPS fusion).
    """

    def __init__(self, bus) -> None:
        self.bus = bus
        self.latest_pose: Pose2D | None = None
        self.bus.subscribe("/ground_truth_pose", self._on_ground_truth)

    def _on_ground_truth(self, pose: Pose2D) -> None:
        self.latest_pose = pose
        self.bus.publish("/pose", pose)
