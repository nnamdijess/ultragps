"""Message-like dataclasses used by the ROS-style simulation."""

from dataclasses import dataclass


@dataclass
class Pose2D:
    x: float
    y: float
    theta: float


@dataclass
class Twist2D:
    v: float
    omega: float


@dataclass
class Waypoint:
    x: float
    y: float
