import os
import sys
import unittest
from unittest.mock import patch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIR = os.path.join(ROOT, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from run_sim import parse_args
from ultragps_sim.bus import SimpleBus
from ultragps_sim.controller import GoToGoalController
from ultragps_sim.messages import Pose2D, Twist2D, Waypoint
from ultragps_sim.sim_app import SimulationApp, SimulationConfig
from ultragps_sim.vehicle_simulator import DifferentialDriveSimulator
from ultragps_sim.waypoint_manager import WaypointManager


class TestDifferentialDriveSimulator(unittest.TestCase):
    def test_straight_motion(self):
        bus = SimpleBus()
        sim = DifferentialDriveSimulator(bus, initial_pose=Pose2D(0.0, 0.0, 0.0))
        bus.publish("/cmd_vel", Twist2D(v=1.0, omega=0.0))
        pose = sim.step(1.0)
        self.assertAlmostEqual(pose.x, 1.0, places=5)
        self.assertAlmostEqual(pose.y, 0.0, places=5)
        self.assertAlmostEqual(pose.theta, 0.0, places=5)


class TestCli(unittest.TestCase):
    def test_parse_args_supports_plot_flags(self):
        args = parse_args(
            [
                "--mode",
                "curve",
                "--plot",
                "--plot-headings",
                "--heading-stride",
                "7",
            ]
        )
        self.assertEqual(args.mode, "curve")
        self.assertTrue(args.plot)
        self.assertTrue(args.plot_headings)
        self.assertEqual(args.heading_stride, 7)


class FakeAxis:
    def __init__(self) -> None:
        self.calls: dict[str, list[tuple[tuple, dict]]] = {
            "plot": [],
            "scatter": [],
            "quiver": [],
        }
        self.title = None
        self.xlabel = None
        self.ylabel = None
        self.axis_mode = None
        self.grid_called = False
        self.legend_called = False

    def plot(self, *args, **kwargs):
        self.calls["plot"].append((args, kwargs))

    def scatter(self, *args, **kwargs):
        self.calls["scatter"].append((args, kwargs))

    def quiver(self, *args, **kwargs):
        self.calls["quiver"].append((args, kwargs))

    def set_title(self, value):
        self.title = value

    def set_xlabel(self, value):
        self.xlabel = value

    def set_ylabel(self, value):
        self.ylabel = value

    def axis(self, value):
        self.axis_mode = value

    def grid(self, *args, **kwargs):
        self.grid_called = True

    def legend(self):
        self.legend_called = True


class TestTrajectoryPlotting(unittest.TestCase):
    def test_render_plot_marks_trajectory_start_final_waypoints_and_headings(self):
        app = SimulationApp(
            mode="waypoint",
            config=SimulationConfig(max_steps=5),
            waypoints=[Waypoint(1.0, 0.0), Waypoint(2.0, 1.0)],
        )
        app.trajectory = [
            Pose2D(0.0, 0.0, 0.0),
            Pose2D(1.0, 0.0, 0.0),
            Pose2D(2.0, 1.0, 1.57),
        ]
        ax = FakeAxis()

        app._render_trajectory_plot(ax, show_headings=True, heading_stride=2)

        self.assertEqual(len(ax.calls["plot"]), 1)
        self.assertEqual(len(ax.calls["scatter"]), 3)
        self.assertEqual(len(ax.calls["quiver"]), 1)
        self.assertEqual(ax.title, "UltraGPS Trajectory (waypoint)")
        self.assertEqual(ax.xlabel, "x [m]")
        self.assertEqual(ax.ylabel, "y [m]")
        self.assertEqual(ax.axis_mode, "equal")
        self.assertTrue(ax.grid_called)
        self.assertTrue(ax.legend_called)

    def test_plot_trajectory_requires_matplotlib_when_requested(self):
        app = SimulationApp(mode="straight", config=SimulationConfig(max_steps=1))
        with patch("builtins.__import__", side_effect=ModuleNotFoundError):
            with self.assertRaises(RuntimeError):
                app.plot_trajectory()


class TestControllerAndWaypoints(unittest.TestCase):
    def test_controller_reaches_goal_and_emits_event(self):
        bus = SimpleBus()
        controller = GoToGoalController(bus, goal_tolerance=0.1)

        bus.publish("/goal_waypoint", Waypoint(1.0, 0.0))
        bus.publish("/pose", Pose2D(1.05, 0.0, 0.0))

        cmd = controller.step()
        self.assertEqual(cmd.v, 0.0)
        self.assertEqual(cmd.omega, 0.0)

        reached_event = bus.last_message("/goal_reached")
        self.assertIsNotNone(reached_event)

    def test_waypoint_manager_advances_queue(self):
        bus = SimpleBus()
        manager = WaypointManager(bus, [Waypoint(1.0, 0.0), Waypoint(2.0, 0.0)])

        first = manager.publish_goal()
        self.assertEqual((first.x, first.y), (1.0, 0.0))

        bus.publish("/goal_reached", {"goal_x": 1.0, "goal_y": 0.0})
        second = manager.current_goal
        self.assertIsNotNone(second)
        self.assertEqual((second.x, second.y), (2.0, 0.0))


if __name__ == "__main__":
    unittest.main()
