# AGENTS.md — LQR Nav2 Controller Plugin (ROS2)

## Project Overview

Implement an LQR (Linear Quadratic Regulator) path-tracking controller as a C++ Nav2 Controller Server plugin for ROS2 Humble. The controller linearizes unicycle dynamics around a reference trajectory in body-frame coordinates, solves the Discrete Algebraic Riccati Equation (DARE) for steady-state gains, and produces smooth velocity commands.

- **Robot**: TurtleBot3 Burger (simulated in Gazebo)
- **ROS2 distro**: Humble (ament_cmake build)
- **Package name**: `lqr_controller`
- **Final deadline**: April 17

### Physical Robot Integration (Future)

This project starts in Gazebo simulation but will eventually deploy to a real TurtleBot3 Burger. Design decisions that support this transition:

- **No simulation-specific code in the controller** — the plugin receives poses and returns velocities through the Nav2 interface, which works identically on real hardware.
- **Velocity limits match TurtleBot3 Burger hardware** — `max_linear_speed = 0.22 m/s`, `max_angular_speed = 2.84 rad/s`.
- **The same `nav2_params.yaml`** can be used on the real robot by swapping the simulation launch for `turtlebot3_bringup` and real-robot AMCL/map.
- **Tuning may differ** — real-world friction, wheel slip, and sensor noise will likely require different Q/R weights and lookahead distance vs. simulation.

When transitioning to the physical robot:
1. Replace `nav2_bringup tb3_simulation_launch.py` with the real TurtleBot3 bringup launch
2. Provide a pre-built map (SLAM beforehand) or use SLAM Toolbox live
3. Adjust AMCL parameters for real LiDAR noise characteristics
4. Re-tune LQR Q/R weights for real-world dynamics

## Architecture

```
                         Nav2 Stack
  ┌──────────────────────────────────────────────────┐
  │  BT Navigator                                    │
  │      │                                           │
  │      v                                           │
  │  Planner Server (NavFn)                          │
  │      │  nav_msgs/Path                            │
  │      v                                           │
  │  Controller Server                               │
  │      │  loads plugin: lqr_controller             │
  │      v                                           │
  │  ┌───────────────────────────────────────┐       │
  │  │  LQRController                        │       │
  │  │  (nav2_core::Controller)              │       │
  │  │                                       │       │
  │  │  setPlan(path)                        │       │
  │  │      │                                │       │
  │  │  computeVelocityCommands(pose, vel)   │       │
  │  │      │                                │       │
  │  │      ├── findClosestPoint()           │       │
  │  │      ├── findLookaheadPoint()         │       │
  │  │      ├── lqr_solver::                 │       │
  │  │      │     computeBodyFrameError()    │       │
  │  │      └── u = u_ref - K * error        │       │
  │  │          │                            │       │
  │  │          v                            │       │
  │  │   TwistStamped (v, omega)             │       │
  │  └───────────────────────────────────────┘       │
  │      │                                           │
  │      v                                           │
  │  /cmd_vel --> TurtleBot3 Burger                  │
  └──────────────────────────────────────────────────┘

  Gazebo <--> TurtleBot3 <--> /odom, /tf, /scan
                                    │
                                    v
                               AMCL --> /tf (map -> odom)

  Code Separation:
  ┌─────────────────────┐     ┌──────────────────────────┐
  │  lqr_solver.hpp/cpp │     │  lqr_controller.hpp/cpp  │
  │  (pure math, Eigen) │ <── │  (Nav2 plugin, ROS2)     │
  │                     │     │                          │
  │  buildLinearSystem  │     │  configure / setPlan     │
  │  solveDARE          │     │  computeVelocityCommands │
  │  computeGain        │     │  findClosestPoint        │
  │  computeBodyFrame   │     │  findLookaheadPoint      │
  │  Error              │     │  getYawFromQuaternion    │
  │  normalizeAngle     │     │  setSpeedLimit           │
  └─────────────────────┘     └──────────────────────────┘
```

### Data Flow

1. NavFn planner generates global path (`nav_msgs/Path`)
2. Controller Server calls `setPlan(path)` to store the path
3. At control rate (~20 Hz), Controller Server calls `computeVelocityCommands(pose, velocity)`
4. LQR controller:
   - a. Finds closest point on path to robot
   - b. Advances by lookahead distance to get reference point
   - c. Calls `lqr_solver::computeBodyFrameError()` for error `[e_long, e_lat, e_theta]`
   - d. Applies precomputed LQR gain: `delta_u = -K * error`
   - e. Adds to reference: `v = v_ref + delta_v`, `omega = 0 + delta_omega`
   - f. Clamps to velocity limits
5. Returns `TwistStamped` command

## File Map

```
LQR-Controller/
├── AGENTS.md                                  # This file
├── README.md                                  # Build/run instructions
├── proposal.md                                # Course project proposal
├── .gitignore                                 # Ignores build/, install/, log/
├── ros_entrypoint.sh                          # Docker entrypoint (sources ROS)
├── .devcontainer/
│   ├── devcontainer.json                      # VS Code dev container config
│   └── Dockerfile                             # ROS2 Humble + Nav2 + Gazebo image
├── src/
│   └── lqr_controller/                        # ROS2 C++ package
│       ├── include/lqr_controller/
│       │   ├── lqr_controller.hpp             # Nav2 plugin class header  [IMPLEMENT]
│       │   └── lqr_solver.hpp                 # LQR math (pure Eigen)    [CREATE]
│       ├── src/
│       │   ├── lqr_controller.cpp             # Nav2 plugin (ROS glue)   [IMPLEMENT]
│       │   └── lqr_solver.cpp                 # LQR math implementation  [CREATE]
│       ├── CMakeLists.txt                     # Build config (Eigen3 linked)
│       ├── package.xml                        # ROS2 package manifest
│       ├── lqr_controller_plugin.xml          # Nav2 pluginlib descriptor
│       └── LICENSE
├── config/                                    # [CREATE]
│   ├── nav2_params.yaml                       # Nav2 config with LQR controller
│   └── dwb_params.yaml                        # Nav2 config with DWB (comparison baseline)
├── scripts/                                   # [CREATE]
│   └── benchmark_analysis.py                  # Offline rosbag metrics analysis
├── docs/                                      # Implementation documentation
│   └── YYYY-MM-DD_title.md                    # Timestamped notes (see Documentation section)
└── references/
    └── ros2-model-predictive-controller/      # Reference MPC project (gitignored)
```

Files marked `[IMPLEMENT]` — existing skeleton, fill in the logic.
Files marked `[CREATE]` — do not exist yet, must be created.

## LQR Algorithm

### Dynamics Model — Unicycle (Dubins Car)

Continuous-time:

```
x_dot     = v * cos(theta)
y_dot     = v * sin(theta)
theta_dot = omega
```

State: `x = [px, py, theta]`
Control: `u = [v, omega]`

### Error State — Body Frame

Given robot pose `(x, y, theta)` and reference pose `(x_ref, y_ref, theta_ref)`:

```
dx = x - x_ref
dy = y - y_ref

e_long  =  cos(theta_ref) * dx + sin(theta_ref) * dy     // along-track error
e_lat   = -sin(theta_ref) * dx + cos(theta_ref) * dy     // cross-track (lateral) error
e_theta =  normalize_angle(theta - theta_ref)             // heading error
```

Error vector: `e = [e_long, e_lat, e_theta]^T`   (3x1)

### Linearized Error Dynamics

Around equilibrium `e = 0` with reference controls `v_ref` (constant desired speed) and `omega_ref = 0`:

Continuous A, B matrices:

```
A_c = [ 0,  0,  0     ]         B_c = [ 1,  0 ]
      [ 0,  0,  v_ref ]               [ 0,  0 ]
      [ 0,  0,  0     ]               [ 0,  1 ]
```

Derivation (linearized error dynamics, omega_ref = 0):

```
e_long_dot  = delta_v                        (along-track responds to speed correction)
e_lat_dot   = v_ref * e_theta                (lateral drift from heading error)
e_theta_dot = delta_omega                    (heading responds to turn correction)
```

Discretized via Forward Euler with timestep `dt`:

```
A_d = I + A_c * dt = [ 1,  0,  0          ]       B_d = B_c * dt = [ dt,  0  ]
                     [ 0,  1,  v_ref * dt  ]                       [  0,  0  ]
                     [ 0,  0,  1           ]                       [  0,  dt ]
```

**Key insight**: With constant `v_ref` and `omega_ref = 0`, the `A_d` and `B_d` matrices are constant. DARE only needs to be solved **once** (in `configure()`), not every control cycle.

### DARE Solver — Iterative Method

Solve for steady-state P (3x3 symmetric positive definite):

```
P_{k+1} = Q + A_d^T * P_k * A_d
         - A_d^T * P_k * B_d * (R + B_d^T * P_k * B_d)^{-1} * B_d^T * P_k * A_d
```

Algorithm:

1. Initialize `P_0 = Q`
2. For `k = 0, 1, 2, ...` up to `dare_max_iterations`:
   - Compute `M = R + B_d^T * P_k * B_d`     (2x2 — cheap to invert)
   - Compute `P_{k+1} = Q + A_d^T * P_k * A_d - A_d^T * P_k * B_d * M^{-1} * B_d^T * P_k * A_d`
   - If `max(|P_{k+1} - P_k|) < dare_tolerance` → converged, store P
   - Else continue
3. If not converged after max iterations → log warning, use last P

### LQR Gain Computation

```
K = (R + B_d^T * P * B_d)^{-1} * B_d^T * P * A_d
```

K is 2x3 matrix. Computed once after DARE converges.

### Control Law

```
delta_u = -K * e              // 2x1 = -(2x3) * (3x1)

v     = v_ref + delta_u(0)   // linear velocity
omega = 0.0   + delta_u(1)   // angular velocity (omega_ref = 0)

v     = clamp(v,     0.0,                max_linear_speed )
omega = clamp(omega, -max_angular_speed,  max_angular_speed)
```

### Reference Point Selection

```
  robot
    X .......... lookahead_distance ........... R (reference point)
    :                                           :
    : closest point                             :
    +---*---------------------------------------+------ global path
        ^                                       ^
   closest_idx                              ref_idx
```

1. **Find closest point**: Linear scan through `global_plan_.poses`, find index with minimum Euclidean distance to robot position.
2. **Advance by lookahead**: From `closest_idx`, walk forward along path, accumulating arc-length between consecutive waypoints, until total >= `lookahead_distance`.
3. **Clamp**: If path ends before lookahead distance is reached, use last waypoint.
4. **Reference heading**: Extract yaw from the orientation quaternion of the reference pose. If the path orientation is identity, compute `atan2(dy, dx)` between consecutive waypoints instead.

## Implementation Sections

The code is split into two files for clarity:

| File | Responsibility | ROS dependency? |
|------|---------------|-----------------|
| `lqr_solver.hpp/cpp` | LQR math: linearization, DARE, gain, error | **No** — pure C++ and Eigen |
| `lqr_controller.hpp/cpp` | Nav2 plugin: params, path, ROS interface | **Yes** — ROS2 types, Nav2 |

### File 1: `lqr_solver.hpp` — LQR Math (Pure Eigen, No ROS)

```cpp
#ifndef LQR_CONTROLLER__LQR_SOLVER_HPP_
#define LQR_CONTROLLER__LQR_SOLVER_HPP_

#include <Eigen/Dense>

namespace lqr_solver
{

// Build discrete A_d, B_d matrices from unicycle linearization
// (Forward Euler, constant v_ref, omega_ref = 0)
void buildLinearSystem(
  double v_ref, double dt,
  Eigen::Matrix3d & A_d,
  Eigen::Matrix<double, 3, 2> & B_d);

// Solve DARE iteratively. Stores result in P.
// Returns true if converged within max_iterations.
bool solveDARE(
  const Eigen::Matrix3d & A_d,
  const Eigen::Matrix<double, 3, 2> & B_d,
  const Eigen::Matrix3d & Q,
  const Eigen::Matrix2d & R,
  int max_iterations, double tolerance,
  Eigen::Matrix3d & P);

// Compute LQR gain K from DARE solution P
void computeGain(
  const Eigen::Matrix3d & A_d,
  const Eigen::Matrix<double, 3, 2> & B_d,
  const Eigen::Matrix3d & P,
  const Eigen::Matrix2d & R,
  Eigen::Matrix<double, 2, 3> & K);

// Compute body-frame tracking error between robot and reference pose
// Returns [e_long, e_lat, e_theta]
Eigen::Vector3d computeBodyFrameError(
  double x, double y, double theta,
  double x_ref, double y_ref, double theta_ref);

// Wrap angle to [-pi, pi]
double normalizeAngle(double angle);

}  // namespace lqr_solver

#endif
```

### File 2: `lqr_solver.cpp` — LQR Math Implementation

#### `buildLinearSystem(v_ref, dt, A_d, B_d)`

```cpp
// A_d = I + A_c * dt
A_d = Eigen::Matrix3d::Identity();
A_d(1, 2) = v_ref * dt;         // e_lat row, e_theta column

// B_d = B_c * dt
B_d = Eigen::Matrix<double, 3, 2>::Zero();
B_d(0, 0) = dt;                  // e_long responds to delta_v
B_d(2, 1) = dt;                  // e_theta responds to delta_omega
```

#### `solveDARE(A_d, B_d, Q, R, max_iterations, tolerance, P)` → `bool`

```
P = Q
for k = 0 to max_iterations:
    M     = R + B_d.transpose() * P * B_d            // 2x2
    P_new = Q + A_d.transpose() * P * A_d
            - A_d.transpose() * P * B_d * M.inverse() * B_d.transpose() * P * A_d
    if (P_new - P).cwiseAbs().maxCoeff() < tolerance:
        P = P_new
        return true
    P = P_new
return false   // did not converge
```

#### `computeGain(A_d, B_d, P, R, K)`

```cpp
Eigen::Matrix2d M = R + B_d.transpose() * P * B_d;
K = M.inverse() * B_d.transpose() * P * A_d;
```

#### `computeBodyFrameError(x, y, theta, x_ref, y_ref, theta_ref)` → `Eigen::Vector3d`

```
dx = x - x_ref
dy = y - y_ref
e_long  =  cos(theta_ref) * dx + sin(theta_ref) * dy
e_lat   = -sin(theta_ref) * dx + cos(theta_ref) * dy
e_theta = normalizeAngle(theta - theta_ref)
return Vector3d(e_long, e_lat, e_theta)
```

#### `normalizeAngle(angle)` → `double`

```cpp
#include <angles/angles.h>
return angles::normalize_angle(angle);
```

### File 3: `lqr_controller.hpp` — Nav2 Plugin Header

Add `#include "lqr_controller/lqr_solver.hpp"` and the following private members:

```cpp
private:
  // --- Node / Nav2 handles ---
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("lqr_controller")};
  rclcpp::Clock::SharedPtr clock_;

  // --- Stored path ---
  nav_msgs::msg::Path global_plan_;

  // --- Tunable parameters (declared as ROS2 params) ---
  double desired_speed_;         // v_ref constant (m/s)
  double max_linear_speed_;      // upper clamp for v (m/s)
  double max_angular_speed_;     // upper clamp for |omega| (rad/s)
  double lookahead_distance_;    // meters ahead on path for reference point
  double dt_;                    // discretization timestep (s)
  int    dare_max_iterations_;   // DARE solver iteration cap
  double dare_tolerance_;        // DARE convergence threshold

  // Q diagonal weights (3x3 state cost)
  double q_long_;                // along-track error weight
  double q_lat_;                 // lateral error weight
  double q_head_;                // heading error weight

  // R diagonal weights (2x2 control effort cost)
  double r_v_;                   // linear velocity effort weight
  double r_omega_;               // angular velocity effort weight

  // --- LQR matrices (Eigen3) ---
  Eigen::Matrix3d A_d_;                     // 3x3 discrete state matrix
  Eigen::Matrix<double, 3, 2> B_d_;         // 3x2 discrete input matrix
  Eigen::Matrix3d Q_;                       // 3x3 state cost
  Eigen::Matrix2d R_;                       // 2x2 control cost
  Eigen::Matrix3d P_;                       // 3x3 DARE solution
  Eigen::Matrix<double, 2, 3> K_;           // 2x3 LQR gain

  // --- Helper methods (Nav2-specific, use ROS message types) ---
  size_t findClosestPoint(const geometry_msgs::msg::PoseStamped & pose);
  size_t findLookaheadPoint(size_t closest_idx);
  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q);
```

Note: `buildLinearSystem`, `solveDARE`, `computeGain`, `computeBodyFrameError`, and `normalizeAngle` now live in `lqr_solver::` namespace — not as class methods.

### File 4: `lqr_controller.cpp` — Nav2 Plugin Implementation

#### `configure(parent, name, tf, costmap_ros)`

| Step | What to do |
|------|------------|
| 1 | Store `node_`, `plugin_name_`, `tf_`, `costmap_ros_` from arguments |
| 2 | Get `logger_` and `clock_` from the parent node |
| 3 | Declare + read all ROS parameters (see Parameters table). Use `node->declare_parameter<double>(plugin_name_ + ".desired_speed", 0.2)` pattern |
| 4 | Build diagonal Q_ from `q_long_`, `q_lat_`, `q_head_` |
| 5 | Build diagonal R_ from `r_v_`, `r_omega_` |
| 6 | Call `lqr_solver::buildLinearSystem(desired_speed_, dt_, A_d_, B_d_)` |
| 7 | Call `lqr_solver::solveDARE(A_d_, B_d_, Q_, R_, dare_max_iterations_, dare_tolerance_, P_)` — log warning if returns false |
| 8 | Call `lqr_solver::computeGain(A_d_, B_d_, P_, R_, K_)` |
| 9 | Log the computed K_ matrix at INFO level |

#### `setPlan(path)`

| Step | What to do |
|------|------------|
| 1 | `global_plan_ = path` |
| 2 | (Optional) Log if path is empty |

#### `computeVelocityCommands(pose, velocity, goal_checker)` — Core Loop

| Step | What to do |
|------|------------|
| 1 | If `global_plan_.poses.empty()` → throw `nav2_core::PlannerException("Empty path")` |
| 2 | `closest_idx = findClosestPoint(pose)` |
| 3 | `ref_idx = findLookaheadPoint(closest_idx)` |
| 4 | `ref_pose = global_plan_.poses[ref_idx]` |
| 5 | Extract robot `(x, y, theta)` and ref `(x_ref, y_ref, theta_ref)` using `getYawFromQuaternion()` |
| 6 | `error = lqr_solver::computeBodyFrameError(x, y, theta, x_ref, y_ref, theta_ref)` |
| 7 | `delta_u = -K_ * error` — Eigen matrix-vector multiply, result is `Eigen::Vector2d` |
| 8 | `v = desired_speed_ + delta_u(0)` |
| 9 | `omega = 0.0 + delta_u(1)` |
| 10 | Clamp v to `[0.0, max_linear_speed_]` |
| 11 | Clamp omega to `[-max_angular_speed_, max_angular_speed_]` |
| 12 | Build `TwistStamped`: set `header = pose.header`, `twist.linear.x = v`, `twist.angular.z = omega` |
| 13 | Return the `TwistStamped` |

#### `findClosestPoint(pose)` → `size_t`

1. Extract robot `x`, `y` from `pose.pose.position`
2. Linear scan through all `global_plan_.poses`
3. Compute squared Euclidean distance to each waypoint (skip sqrt for performance)
4. Return index of minimum distance

#### `findLookaheadPoint(closest_idx)` → `size_t`

1. Start at `closest_idx`, set `accumulated_dist = 0.0`
2. For `i = closest_idx` to `global_plan_.poses.size() - 2`:
   - `dx = poses[i+1].x - poses[i].x`
   - `dy = poses[i+1].y - poses[i].y`
   - `accumulated_dist += sqrt(dx*dx + dy*dy)`
   - If `accumulated_dist >= lookahead_distance_` → return `i + 1`
3. Return `global_plan_.poses.size() - 1` (last point)

#### `getYawFromQuaternion(q)` → `double`

```cpp
#include <tf2/utils.h>

tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
double roll, pitch, yaw;
tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
return yaw;
```

#### `setSpeedLimit(speed_limit, percentage)`

```
if percentage:
    max_linear_speed_ = desired_speed_ * speed_limit / 100.0
else:
    max_linear_speed_ = speed_limit
```

#### `activate()` / `deactivate()` / `cleanup()`

Minimal — just log the lifecycle transition. No threads, publishers, or subscribers to manage (Controller Server handles those).

```cpp
void LQRController::activate()   { RCLCPP_INFO(logger_, "%s activated",   plugin_name_.c_str()); }
void LQRController::deactivate() { RCLCPP_INFO(logger_, "%s deactivated", plugin_name_.c_str()); }
void LQRController::cleanup()    { RCLCPP_INFO(logger_, "%s cleaned up",  plugin_name_.c_str()); }
```

### CMakeLists.txt Changes

Add `lqr_solver.cpp` to the shared library sources and link `Eigen3::Eigen`:

```cmake
add_library(${PROJECT_NAME} SHARED
  src/lqr_controller.cpp
  src/lqr_solver.cpp           # <-- ADD THIS
)

# After ament_target_dependencies, add Eigen3 include:
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${EIGEN3_INCLUDE_DIR}        # <-- ADD THIS (if not already picked up)
)
```

Eigen3 is already in `find_package()` — just needs to be linked to the target if not already.

## Parameters

All declared in `configure()`, loadable from YAML under the plugin namespace.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `desired_speed` | double | 0.2 | Reference linear velocity v_ref (m/s) |
| `max_linear_speed` | double | 0.22 | Max linear velocity clamp (m/s). TurtleBot3 Burger hardware limit. |
| `max_angular_speed` | double | 2.84 | Max angular velocity clamp (rad/s). TurtleBot3 Burger hardware limit. |
| `lookahead_distance` | double | 0.5 | How far ahead on path to pick reference point (m) |
| `dt` | double | 0.05 | Discretization timestep for Forward Euler linearization (s) |
| `dare_max_iterations` | int | 1000 | Max iterations for iterative DARE solver |
| `dare_tolerance` | double | 1e-9 | Convergence threshold for DARE (max element-wise change) |
| `q_longitudinal` | double | 1.0 | Q weight for along-track error. Penalizes being ahead/behind on path. |
| `q_lateral` | double | 3.0 | Q weight for cross-track error. Penalizes deviation from path. Higher = tighter tracking. |
| `q_heading` | double | 1.0 | Q weight for heading error. Penalizes pointing away from path. |
| `r_v` | double | 1.0 | R weight for linear velocity effort. Higher = less aggressive speed corrections. |
| `r_omega` | double | 0.5 | R weight for angular velocity effort. Higher = smoother turns. |

### Tuning Tips

- **Tighter path following**: Increase `q_lateral` (e.g., 5.0-10.0)
- **Reduce oscillation**: Increase `q_heading` and `r_omega`
- **Smoother overall**: Increase `r_v` and `r_omega` together
- **More responsive**: Decrease R weights (but may oscillate)
- **Lookahead tradeoff**: Larger = smoother but cuts corners. Smaller = tighter but may oscillate near the path

## Simulation Setup

### Gazebo World Options

Use standard TurtleBot3 worlds from `turtlebot3_gazebo` package. No custom world files needed in this repo.

| World | Description | Best For |
|-------|-------------|----------|
| `turtlebot3_world` | Small room with hexagonal pillar obstacles | Obstacle avoidance benchmarking |
| `turtlebot3_house` | Multi-room indoor house | Long-range navigation tests |
| `empty_world` | Flat plane, no obstacles | Pure path-tracking validation |

Set the robot model environment variable before launching:

```bash
export TURTLEBOT3_MODEL=burger
```

### Running with LQR Controller

```bash
# Terminal 1: Launch Gazebo + Nav2 with LQR controller config
export TURTLEBOT3_MODEL=burger
ros2 launch nav2_bringup tb3_simulation_launch.py \
    params_file:=$(pwd)/config/nav2_params.yaml

# Terminal 2: Send a navigation goal (or use RViz2 "2D Goal Pose" button)
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 2.0, y: 0.5, z: 0.0},
         orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
}"
```

### Running with DWB Controller (Comparison Baseline)

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch nav2_bringup tb3_simulation_launch.py \
    params_file:=$(pwd)/config/dwb_params.yaml
```

### Nav2 Config — Key Section (`config/nav2_params.yaml`)

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "lqr_controller/LQRController"
      desired_speed: 0.2
      max_linear_speed: 0.22
      max_angular_speed: 2.84
      lookahead_distance: 0.5
      dt: 0.05
      dare_max_iterations: 1000
      dare_tolerance: 1.0e-9
      q_longitudinal: 1.0
      q_lateral: 3.0
      q_heading: 1.0
      r_v: 1.0
      r_omega: 0.5
```

The rest of `nav2_params.yaml` should include standard NavFn planner, AMCL, costmap, BT navigator config. Base it on the default `nav2_bringup` params file and only override the `controller_server` section.

## Benchmarking Pipeline

### Metrics

| Metric | Definition | Computation |
|--------|------------|-------------|
| Cross-track error | Distance from robot to closest point on planned path | For each timestep: min distance from robot `(x,y)` to any segment of the global path |
| Mean tracking error | Average cross-track error over entire run | Mean of cross-track error timeseries |
| Max tracking error | Worst-case deviation from path | Max of cross-track error timeseries |
| Smoothness (cmd_vel) | Rate of change of velocity commands | RMS of `dv/dt` and `domega/dt` from `/cmd_vel` |
| Time-to-goal | Seconds from navigation start to goal reached | Elapsed time from first `/cmd_vel` to goal checker satisfied |
| Success rate | Fraction of runs reaching goal within timeout | Count successes over N repeated trials |

### Data Collection — Rosbag

Record relevant topics during each test run:

```bash
# While a navigation goal is active:
ros2 bag record -o lqr_run_1 \
    /tf /tf_static \
    /cmd_vel \
    /plan \
    /odom \
    /amcl_pose
```

Run the same navigation goal multiple times for each controller (LQR and DWB) with the same start/goal poses.

### Offline Analysis (`scripts/benchmark_analysis.py`)

Python script responsibilities:

1. Read rosbag2 SQLite files using `rosbag2_py` (or `mcap` reader)
2. Extract robot trajectory from `/amcl_pose` timestamps + positions
3. Extract planned path from `/plan` topic
4. Extract velocity commands from `/cmd_vel`
5. Compute all metrics from the table above
6. Output:
   - Comparison table (LQR vs DWB, each metric)
   - Time-series plots (cross-track error, cmd_vel over time)
   - Summary statistics (mean, std, max for each metric)

Dependencies for the script: `rosbag2_py`, `numpy`, `matplotlib`

### Test Procedure

1. Launch simulation with LQR config
2. Set a consistent start pose (e.g., default spawn) and goal pose
3. Start rosbag recording
4. Send navigation goal
5. Wait for completion or timeout (e.g., 120s)
6. Stop recording
7. Repeat steps 1-6 with DWB config
8. Run `benchmark_analysis.py` on both bags

## Build & Run

```bash
# Inside the dev container:

# Build the package
colcon build --symlink-install --packages-select lqr_controller
source install/setup.bash

# Verify plugin is discoverable by Nav2
ros2 plugin list | grep lqr
# Expected output: lqr_controller/LQRController
```

## Dependencies

| Dependency | Purpose |
|------------|---------|
| `rclcpp` | ROS2 C++ client library |
| `rclcpp_lifecycle` | Lifecycle node interface (required by nav2_core) |
| `nav2_core` | `Controller` base class interface |
| `nav2_costmap_2d` | Costmap handle (passed to configure, not actively used) |
| `nav_msgs` | `Path` message type |
| `geometry_msgs` | `PoseStamped`, `Twist`, `TwistStamped` |
| `tf2_ros` | TF buffer (passed to configure) |
| `tf2_geometry_msgs` | Quaternion utilities |
| `angles` | `normalize_angle()` |
| `Eigen3` | Matrix math: DARE solver, gain computation |
| `pluginlib` | Nav2 plugin loading mechanism |

**Simulation (not built by this package):**
| Dependency | Purpose |
|------------|---------|
| `nav2_bringup` | Simulation launch files |
| `turtlebot3_gazebo` | Robot model + Gazebo worlds |
| `turtlebot3_description` | URDF |
| `gazebo_ros` | Gazebo-ROS bridge |

## ROS2 Topics

The controller itself does NOT subscribe or publish directly. It is called by the Nav2 Controller Server.

### Controller Interface (called by Controller Server)

| Method | Message Type | Direction | Description |
|--------|-------------|-----------|-------------|
| `computeVelocityCommands()` returns | `TwistStamped` | Output | Velocity command for the robot |
| `setPlan()` receives | `nav_msgs/Path` | Input | Global path from planner |
| `pose` argument | `PoseStamped` | Input | Current robot pose (Controller Server passes this) |
| `velocity` argument | `Twist` | Input | Current robot velocity |

### Topics Managed by Nav2 (for reference)

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `Twist` | Controller Server publishes our TwistStamped.twist here |
| `/plan` | `Path` | Global path from Planner Server |
| `/odom` | `Odometry` | Robot odometry from Gazebo |
| `/amcl_pose` | `PoseWithCovarianceStamped` | Localized robot pose |
| `/tf` | `TFMessage` | Transform tree (map->odom->base_link) |
| `/scan` | `LaserScan` | Lidar (used by AMCL and costmap, not by our controller) |

## Common Pitfalls

1. **Angle wrapping**: `theta - theta_ref` can exceed pi. MUST normalize to `[-pi, pi]`. Use `angles::normalize_angle()` from the `angles` package. Forgetting this causes wild oscillation or spinning.

2. **DARE divergence**: If Q is too large relative to R, or `dt` is too large, DARE may not converge. Always check the return value of `solveDARE()`. Fall back to zero velocity if it fails.

3. **Empty path**: `computeVelocityCommands()` can be called before `setPlan()` during Nav2 startup. Check `global_plan_.poses.empty()` and throw `nav2_core::PlannerException`.

4. **Quaternion to yaw**: Use `tf2::Matrix3x3(q).getRPY(roll, pitch, yaw)`, NOT hand-rolled `atan2`. The tf2 version handles edge cases (gimbal lock, non-normalized quaternions).

5. **Lookahead past path end**: If the robot is near the goal, lookahead may exceed the remaining path length. Always clamp to the last waypoint index.

6. **Eigen include order**: Include `<Eigen/Dense>` AFTER ROS headers to avoid macro conflicts. If you get weird errors about `Success` or `Status`, this is why.

7. **Plugin not found at runtime**: Verify `lqr_controller_plugin.xml` matches `pluginlib_export_plugin_description_file(nav2_core ...)` in CMakeLists.txt. Run `ros2 plugin list` to confirm. Also ensure the library name in the XML `<library path="...">` matches the shared library filename (without `lib` prefix and `.so` suffix).

8. **TwistStamped not Twist**: `computeVelocityCommands()` returns `TwistStamped`, not `Twist`. Set `cmd.header = pose.header` to propagate frame_id and timestamp.

9. **Parameter namespacing**: ROS params must be declared under the plugin name. Use `plugin_name_ + ".param_name"` when calling `declare_parameter()` and `get_parameter()`.

10. **Zero velocity at startup**: Before DARE is solved or path is set, the controller should return zero velocity. The skeleton already does this — don't break it.

11. **Global plan frame**: The path from NavFn is in the `map` frame, same as the robot pose passed to `computeVelocityCommands()`. No frame transformation needed between them.

12. **Inverse of small matrix**: `M.inverse()` on the 2x2 matrix `(R + B^T P B)` is fine for 2x2. For larger matrices, use `.solve()` instead of `.inverse()` — but 2x2 is cheap.

## Coding Style

- Freshman undergrad level — simple, straightforward
- Minimal abstractions — one class, helper methods, no design patterns
- Use Eigen for matrix math but keep it basic: `Matrix3d`, `Vector3d`, `.transpose()`, `.inverse()`
- No template metaprogramming, no auto everywhere, no clever one-liners
- Comment the math — reference the LQR Algorithm section above in code comments
- Explicit types over `auto` for Eigen expressions (avoids lazy evaluation bugs)
- Keep external dependencies minimal — only what is already in `package.xml`
- Prefer `RCLCPP_INFO` / `RCLCPP_WARN` for logging, not `std::cout`

## Documentation Convention

All implementation documentation lives in `docs/` with timestamped filenames.

### File naming

```
docs/YYYY-MM-DD_kebab-case-title.md
```

Examples:
- `docs/2026-03-23_project-setup-and-agents-md.md`
- `docs/2026-03-25_lqr-solver-implementation.md`
- `docs/2026-04-01_gazebo-simulation-setup.md`
- `docs/2026-04-10_benchmarking-lqr-vs-dwb.md`

### Template

Each doc should follow this structure:

```markdown
# Title
**Date**: YYYY-MM-DD
**Author**: [name]

## Summary
Brief description of what was done in this session.

## Changes Made
- List of files modified/created
- What was added or changed

## Decisions
Key technical decisions and their rationale.

## Issues / Blockers
Problems encountered and how they were resolved (or remain open).

## Next Steps
What needs to happen next.
```

### When to create a doc

- After each significant implementation session
- When making non-obvious technical decisions
- When encountering and resolving tricky bugs
- Before and after benchmarking runs
