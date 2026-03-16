import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIR = os.path.join(ROOT, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

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


class TestTrajectoryRecording(unittest.TestCase):
    def test_trajectory_is_recorded(self):
        app = SimulationApp(
            mode="straight",
            config=SimulationConfig(dt=0.1, max_steps=10, log_every_n=1000),
        )

        app.run(plot=False)

        self.assertEqual(len(app.trajectory), 11)


if __name__ == "__main__":
    unittest.main()
