"""Waypoint manager component."""

from collections.abc import Iterable

from ultragps_sim.messages import Waypoint


class WaypointManager:
    """Stores waypoint queue and publishes the current active waypoint.

    Topic behavior:
    - Publishes `/goal_waypoint` with the active waypoint.
    - Subscribes to `/goal_reached` and advances to the next waypoint.
    """

    def __init__(self, bus, waypoints: Iterable[Waypoint]) -> None:
        self.bus = bus
        self._waypoints = list(waypoints)
        self._index = 0
        self.completed = False

        self.bus.subscribe("/goal_reached", self._on_goal_reached)

    @property
    def current_goal(self) -> Waypoint | None:
        if self._index < len(self._waypoints):
            return self._waypoints[self._index]
        return None

    def publish_goal(self) -> Waypoint | None:
        goal = self.current_goal
        if goal is None:
            self.completed = True
            return None
        self.bus.publish("/goal_waypoint", goal)
        return goal

    def _on_goal_reached(self, _: dict) -> None:
        self._index += 1
        if self._index >= len(self._waypoints):
            self.completed = True
            return
        self.publish_goal()
