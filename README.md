# ROS 2 Trajectory Tracking & Path Smoothing for Autonomous Mobile Robots

This repository implements a robust, high-performance trajectory tracking system for a differential drive robot (TurtleBot3). The core of the system uses B-Spline Path Smoothing and a Pure Pursuit Controller, featuring dynamic Local Obstacle Avoidance.

## 1. Setup & Execution

**Environment:** ROS 2 Humble on Ubuntu 22.04.

**Simulator:** Fully compatible with both Gazebo Classic and the new Gazebo (Ignition/GZ Sim).

**Installation:**

```bash
cd ~/trajectory_tracking_ws
colcon build --packages-select trajectory_tracking
source install/setup.bash
```

**Execution:**

```bash
ros2 launch trajectory_tracking bringup.launch.py
```

### Custom Waypoints / Data Input Format

To ensure modularity, waypoints can be added by editing the launch file. They are passed as two synchronized arrays. The system enforces a strict 1:1 mapping where $X[i]$ and $Y[i]$ define the $i^{th}$ control point. It is mandatory for both arrays to have identical lengths to maintain trajectory integrity and prevent runtime exceptions.

**Example 1: Irregular Zigzag / Roller-coaster Path**
```python
{'waypoints_x': [0.0, 2.0, 4.0, 6.0, 8.0, 10.0]},
{'waypoints_y': [0.0, 1.0, -1.0, 0.0, 1.0, -1.0]}
```
*   Point 1: (0.0, 0.0) — Origin
*   Point 2: (2.0, 1.0) — Upward move
*   Point 3: (4.0, -1.0) — Downward dip
*   Point 4: (6.0, 0.0) — Back to centerline
*   Point 5: (8.0, 1.0) — Second peak
*   Point 6: (10.0, -1.0) — Final destination

## 2. Design Choices & Algorithms

### Research & Comparative Analysis

During the development phase, I researched and evaluated multiple smoothing techniques to determine the most suitable approach for autonomous navigation:

*   **Moving Average (MA):** While simple to implement, MA creates significant "corner-cutting" issues and lacks mathematical smoothness at the joints, making it unsuitable for high-speed turns.
*   **Curvature Corrected Moving Average (CCMA):** An improvement over MA, but it still fails to provide the higher-order continuity required for fluid robotic motion.
*   **Savitzky-Golay / Polynomial Interpolation:** These are effective for noise reduction but tend to oscillate significantly when waypoints are sparse.
*   **Bezier Curves:** These utilize global control, meaning a change in one waypoint shifts the entire trajectory. This makes local adjustments for obstacle avoidance computationally inefficient.

### The Selection: B-Spline Path Smoothing

I selected B-Splines for this implementation due to two critical technical advantages:

*   **C2 Continuity:** The generated path is continuous up to the second derivative. This ensures smooth velocity and acceleration profiles, preventing "jerky" movements that can damage real robot motors and drain batteries.
*   **Local Control:** B-Splines allow for local modification. Changing a control point only affects a specific segment of the path, which is the foundation of my Local Control Obstacle Avoidance logic.

## 3. Architecture & Edge Case Handling

### Modular Architecture

The system is designed with a modular approach, separating concerns into distinct logical blocks:

*   **Path Generator:** Handles YAML-based waypoint ingestion and B-Spline interpolation.
*   **Controller:** A Pure Pursuit implementation for precise trajectory following.
*   **Safety Monitor:** A LiDAR-based state machine for dynamic obstacle detection.

### Handled Edge Cases

*   **Automatic Clamping:** The node internally duplicates the start and end waypoints (multiplicity of 4) to ensure the curve is clamped exactly to the user's defined coordinates.
*   **Sharp 90° Turns:** Implemented a refined weighting logic to allow the robot to maintain path adherence even during extreme directional changes.

## 4. Extra Credit: Obstacle Avoidance (Local Control)

Unlike global planners that require heavy re-computation, this system uses a Local State-Managed Avoidance Logic:

*   **Detection:** The LiDAR scan monitors a forward cone for potential collisions.
*   **Local Rerouting:** Upon detection, the node injects a temporary "Dodge Waypoint" transversal to the current trajectory.
*   **Commitment State:** To prevent oscillation ("spider-webbing"), the robot enters a commitment state, following the avoidance path for a calculated clearance distance before returning to the global trajectory.

---

### AI Tools Used

**Antigravity AI (Ralph):** Utilized for architectural optimization of the C++ state machine and refining the B-Spline's local control parameters.
