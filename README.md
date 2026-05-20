# UltraGPS Indoor Navigation Simulator (Phase 1: Automatic Setup)

This repository contains a **plain Python simulator** organized in a ROS-style architecture. It is lightweight and runnable without ROS installed, while keeping a clean migration path to ROS 2.

## Design summary

Components are intentionally separated like ROS nodes:

- **Waypoint Manager**: owns a waypoint queue and publishes active goal.
- **Controller**: computes `(v, omega)` from current pose and active goal.
- **Vehicle Simulator**: applies differential-drive kinematics.
- **Pose Publisher**: republishes ground truth as current system pose (placeholder for localization).
- **Message Bus**: tiny in-process pub/sub abstraction that emulates topics.

---

## 1) Folder structures

### A) Automatic setup (current)

```text
ultragps/
├── README.md
├── requirements.txt
├── run_sim.py
├── tests/
│   └── test_sim_core.py
└── src/
    └── ultragps_sim/
        ├── __init__.py
        ├── messages.py
        ├── bus.py
        ├── waypoint_manager.py
        ├── controller.py
        ├── vehicle_simulator.py
        ├── pose_publisher.py
        ├── plotting.py
        └── sim_app.py
```

### B) Manual setup (future ROS 2 mapping)

```text
ros2_ws/
└── src/
    └── ultragps_nav/
        ├── package.xml
        ├── setup.py
        ├── launch/sim.launch.py
        ├── config/controller.yaml
        ├── ultragps_nav/
        │   ├── waypoint_manager_node.py
        │   ├── controller_node.py
        │   ├── vehicle_simulator_node.py
        │   └── pose_publisher_node.py
        └── msg/Waypoint.msg (optional)
```

---

## 2) Logical nodes, topics, and data flow

### Topic responsibilities

- **Waypoint Manager**
  - Publishes: `/goal_waypoint`
  - Subscribes: `/goal_reached` (advance queue)
- **Controller**
  - Subscribes: `/pose`, `/goal_waypoint`
  - Publishes: `/cmd_vel`, `/goal_reached`
- **Vehicle Simulator**
  - Subscribes: `/cmd_vel`
  - Publishes: `/ground_truth_pose`
- **Pose Publisher / Estimator**
  - Subscribes: `/ground_truth_pose`
  - Publishes: `/pose`

### Data flow

`goal_queue -> controller -> cmd_vel -> vehicle_model -> ground_truth_pose -> pose -> controller`

When the goal is reached, controller emits `/goal_reached`, and waypoint manager publishes the next goal.

### Mapping to ROS 2 later

- `SimpleBus.publish(...)` -> ROS 2 publisher
- `SimpleBus.subscribe(...)` -> ROS 2 subscription
- dataclasses -> ROS messages
- loop in `sim_app.py` -> `rclpy` timers/spin

---

## 3) Differential-drive model

Used equations:

- `x_dot = v * cos(theta)`
- `y_dot = v * sin(theta)`
- `theta_dot = omega`

Why use `(v, omega)` first:

- simpler control math
- directly matches ROS `/cmd_vel`
- fewer parameters to tune initially
- easy to convert to/from left/right wheel rates later

---

## 4) Go-to-goal controller (PD)

Given current pose `(x, y, theta)` and goal `(x_g, y_g)`:

1. `dx = x_g - x`, `dy = y_g - y`
2. `distance = sqrt(dx^2 + dy^2)`
3. `goal_heading = atan2(dy, dx)`
4. `heading_error = wrap_to_pi(goal_heading - theta)`
5. `distance_dot = (distance - prev_distance) / dt`
6. `heading_error_dot = (heading_error - prev_heading_error) / dt`
7. `v = clamp(k_v * distance + k_d_v * distance_dot, 0, v_max)`
8. `omega = clamp(k_omega * heading_error + k_d_omega * heading_error_dot, -omega_max, omega_max)`
9. if `distance <= goal_tolerance`: publish stop command and emit `/goal_reached`

---

## 5) Install and run

Install dependencies:

```bash
python -m pip install -r requirements.txt
```

Run scenarios:

```bash
python run_sim.py --mode straight --steps 80
python run_sim.py --mode turn --steps 80
python run_sim.py --mode curve --steps 80
python run_sim.py --mode waypoint --waypoints "1.0,0.0;2.0,2.0" --steps 400
```

Enable trajectory plotting at end of run:

```bash
python run_sim.py --mode waypoint --waypoints "1.0,0.0;2.0,2.0" --steps 400 --plot
```

Save the figure to file (useful in headless environments):

```bash
python run_sim.py --mode waypoint --waypoints "1.0,0.0;2.0,2.0" --steps 400 --plot --plot-output trajectory.png
```

Adjust heading-arrow spacing:

```bash
python run_sim.py --mode waypoint --plot --heading-stride 20
```

Enable CSV performance logging:

```bash
python run_sim.py --mode waypoint --waypoints "1.0,0.0;2.0,2.0" --steps 240 --plot --log-output pd_test.csv --plot-headings --heading-stride 8
```

The CSV includes per-step metrics: time taken, pose (`x`, `y`, `theta`), active goal, distance/heading errors, command (`v`, `omega`), waypoint index, and goal-reached flag.

---

## 6) Tests

```bash
python -m unittest discover -s tests -p "test_*.py"
```

Current tests cover:

- kinematic straight-line update
- goal-reached event generation
- waypoint queue advancement

---

## 7) Interpreting the trajectory plot

- **Blue line**: full x-y robot path over time.
- **Green circle**: start pose position.
- **Red X**: final pose position.
- **Orange stars**: waypoints (when in waypoint mode).
- **Purple arrows**: sampled heading direction (`theta`) along the path.

Use it to quickly verify:

- straight mode => near-straight line
- turn mode => mostly stationary x-y with changing heading
- curve mode => arc-like path
- waypoint mode => path passes through/near waypoint stars and ends at final goal tolerance

---

## 8) What’s next

1. Add timestamped messages and simple logging utility.
2. Add configurable controller gains from YAML/JSON.
3. Add path trace export (CSV) for offline plotting.
4. In ROS 2 migration, replace `SimpleBus` with `rclpy`, add launch files, and publish RViz-friendly markers/TF.
5. Replace `PosePublisher` passthrough with a noisy estimator or UltraGPS-based localization input.
