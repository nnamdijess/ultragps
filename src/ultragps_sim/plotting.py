"""Trajectory plotting utilities."""

from collections.abc import Sequence
from math import cos, sin

from ultragps_sim.messages import Pose2D, Waypoint


def plot_trajectory(
    poses: Sequence[Pose2D],
    waypoints: Sequence[Waypoint] | None = None,
    title: str = "UltraGPS Simulator Trajectory",
    heading_stride: int = 15,
    show: bool = True,
    save_path: str | None = None,
) -> None:
    """Plot x-y trajectory with start/final points and optional heading arrows."""
    if not poses:
        return

    import matplotlib.pyplot as plt

    x_vals = [pose.x for pose in poses]
    y_vals = [pose.y for pose in poses]

    plt.figure(figsize=(8, 6))
    plt.plot(x_vals, y_vals, "b-", linewidth=2, label="Trajectory")

    start = poses[0]
    final = poses[-1]
    plt.scatter([start.x], [start.y], color="green", marker="o", s=80, label="Start")
    plt.scatter([final.x], [final.y], color="red", marker="x", s=90, label="Final")

    if waypoints:
        wp_x = [wp.x for wp in waypoints]
        wp_y = [wp.y for wp in waypoints]
        plt.scatter(wp_x, wp_y, color="orange", marker="*", s=120, label="Waypoints")

    stride = max(1, heading_stride)
    for index in range(0, len(poses), stride):
        pose = poses[index]
        dx = 0.15 * cos(pose.theta)
        dy = 0.15 * sin(pose.theta)
        plt.arrow(
            pose.x,
            pose.y,
            dx,
            dy,
            head_width=0.05,
            head_length=0.08,
            fc="purple",
            ec="purple",
            alpha=0.6,
            length_includes_head=True,
        )

    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title(title)
    plt.axis("equal")
    plt.grid(True, linestyle="--", alpha=0.4)
    plt.legend()
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150)

    if show:
        plt.show()

    plt.close()
