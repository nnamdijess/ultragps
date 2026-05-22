"""Go-to-goal position controller."""

from math import atan2, pi, sqrt

from ultragps_sim.messages import Pose2D, Twist2D, Waypoint


class GoToGoalController:
    """Computes (v, omega) from pose and waypoint using proportional control."""

    def __init__(
        self,
        bus,
        k_v: float = 1.0,
        k_omega: float = 2.5,
        v_max: float = 1.0,
        omega_max: float = 2.0,
        goal_tolerance: float = 0.1,
    ) -> None:
        self.bus = bus
        self.k_v = k_v
        self.k_omega = k_omega
        self.v_max = v_max
        self.omega_max = omega_max
        self.goal_tolerance = goal_tolerance

        self.current_pose: Pose2D | None = None
        self.current_goal: Waypoint | None = None
        self.goal_reached = False

        self.bus.subscribe("/pose", self._on_pose)
        self.bus.subscribe("/goal_waypoint", self._on_goal)

    def _on_pose(self, pose: Pose2D) -> None:
        self.current_pose = pose

    def _on_goal(self, goal: Waypoint) -> None:
        self.current_goal = goal
        self.goal_reached = False

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(value, max_value))

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def step(self) -> Twist2D:
        if self.current_pose is None or self.current_goal is None:
            cmd = Twist2D(v=0.0, omega=0.0)
            self.bus.publish("/cmd_vel", cmd)
            return cmd

        dx = self.current_goal.x - self.current_pose.x
        dy = self.current_goal.y - self.current_pose.y
        distance = sqrt(dx * dx + dy * dy)

        if distance <= self.goal_tolerance:
            if not self.goal_reached:
                self.goal_reached = True
                self.bus.publish(
                    "/goal_reached",
                    {
                        "goal_x": self.current_goal.x,
                        "goal_y": self.current_goal.y,
                    },
                )
            cmd = Twist2D(v=0.0, omega=0.0)
            self.bus.publish("/cmd_vel", cmd)
            return cmd

        goal_heading = atan2(dy, dx)
        heading_error = self._wrap_to_pi(goal_heading - self.current_pose.theta)

        v = self._clamp(self.k_v * distance, 0.0, self.v_max)
        omega = self._clamp(
            self.k_omega * heading_error,
            -self.omega_max,
            self.omega_max,
        )

        cmd = Twist2D(v=v, omega=omega)
        self.bus.publish("/cmd_vel", cmd)
        return cmd
