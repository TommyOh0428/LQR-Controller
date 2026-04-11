# LQR Controller Reference Implementation Analysis

## File Inventory

### Core Algorithm Files
1. `/controller/controller/lqr_algorithm.py` - LQR gain computation and tracking law
2. `/controller/controller/controller_base.py` - Abstract backend interface
3. `/controller/controller/dubins3d_2ctrls.py` - Dubins car dynamics (3D state, 2 controls)
4. `/controller/controller/reference_trajectory.py` - Reference trajectory generation

### ROS2 Integration
5. `/controller/controller/controller_node.py` - ROS2 frontend node
6. `/controller/launch/controller.launch.py` - Launch configuration

### Data Structures & Helpers
7. `/nav_helpers/nav_helpers/trajectory.py` - StateActionTrajectory dataclass & conversions
8. `/nav_helpers_msgs/msg/StateActionPoint.msg` - ROS message for single trajectory point
9. `/nav_helpers_msgs/msg/StateActionTrajectory.msg` - ROS message for full trajectory

### Configuration
10. `/controller/params/controller_params.yaml` - Parameter file with all tuning values
11. `/controller/setup.py` - Package configuration
12. `/nav_helpers/setup.py` - Helper package configuration

### Documentation
13. `/controller/README.md` - Main documentation with implementation tasks
14. `/controller/robot_readme.md` - TurtleBot3 setup and networking guide

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│ ROS2 Network                                                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  /robot_pose (PoseStamped)                                     │
│      ↓                                                          │
│  ┌─────────────────────────────────┐                          │
│  │  ControllerNode (ROS2 Frontend) │                          │
│  │  - Subscribes: /robot_pose      │                          │
│  │  - Subscribes: /traj            │                          │
│  │  - Publishes:  /cmd_vel         │                          │
│  └──────────────┬──────────────────┘                          │
│                 │                                              │
│                 │ Calls backend.get_action()                  │
│                 ↓                                              │
│  ┌──────────────────────────────────────────────────────────┐ │
│  │ ControllerBackend (Abstract Interface)                   │ │
│  │ - get_action(observation, traj) → [v, omega]            │ │
│  ├──────────────────────────────────────────────────────────┤ │
│  │ LQRController (Concrete Implementation)                  │ │
│  │ ┌──────────────────────────────────────────────────────┐ │ │
│  │ │ LQRAlgorithm (Core Algorithm)                        │ │ │
│  │ │ - compute_gains(As, Bs) → Ks, Ps                    │ │ │
│  │ │ - solve(z_0, t_0, z_ref, u_ref) → z_sol, u_sol     │ │ │
│  │ │ - linearize_along_traj(z_traj, u_traj) → As, Bs    │ │ │
│  │ └──────────────────────────────────────────────────────┘ │ │
│  │ Dynamics: DubinsCar3D2Ctrls                              │ │
│  │ - dynamics_np(): x_dot = v*cos(θ), y_dot = v*sin(θ)     │ │
│  │ - linearize(): A, B matrices for discrete-time          │ │
│  └──────────────────────────────────────────────────────────┘ │
│                                                                 │
│  Reference Trajectory Generation                               │
│  - kind: "to_goal" | "straight" | "s_curve" | "path"         │
│  - Generates (n_steps, 3) states and (n_steps, 2) actions     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Core Algorithm Details

### 1. State & Control Definitions

**State** (3D): `z = [x, y, theta]`
- x, y: position in 2D plane
- theta: heading angle (periodic dimension)

**Control** (2D): `u = [v, omega]`
- v: forward speed (linear velocity)
- omega: turn rate (angular velocity)

**Dubins Car Dynamics**:
```
x_dot = v * cos(theta)
y_dot = v * sin(theta)
theta_dot = omega
```

### 2. Finite-Horizon Time-Varying LQR Algorithm

**Problem Setup**:
- Track a nominal trajectory: `z_ref[t]`, `u_ref[t]`
- Linearize around nominal: `δz[t+1] = A_t δz[t] + B_t δu[t]`
- Tracking law: `u_t = u_ref[t] - K_t δz[t]` where `δz = z_t - z_ref[t]`

**Cost Function**:
```
J = sum_{t=0}^{N-1} (δz[t]^T Q δz[t] + δu[t]^T R δu[t]) + δz[N]^T L δz[N]
```

- Q, R: Stage cost matrices (diagonal)
- L: Terminal cost matrix
- Cost coefficients in YAML:
  - State: x_cost=5.0, y_cost=5.0, theta_cost=1.0
  - Control: v_cost=0.1, w_cost=0.1

**Discrete-Time Riccati Recursion** (backward pass):
```
P_N = L  (terminal condition)

For i = N-1, ..., 0:
  K_i = (R + B_i^T P_{i+1} B_i)^(-1) B_i^T P_{i+1} A_i
  P_i = Q + A_i^T P_{i+1} A_i - A_i^T P_{i+1} B_i K_i
```

Returns:
- `Ks`: (N, 2, 3) feedback gain matrices
- `Ps`: (N+1, 3, 3) cost-to-go matrices

### 3. Tracking Control Law

```python
# Compute error (with angle wrapping on theta)
delta_z = z_t - z_ref[t]
delta_z[2] = arctan2(sin(delta_z[2]), cos(delta_z[2]))  # Wrap to [-π, π]

# Apply feedback law with saturation
u_t = u_ref[t] - K_t @ delta_z
u_t = clip(u_t, u_min, u_max)
```

### 4. Linearization (Discrete-Time, Forward Euler)

Continuous-time Jacobians around (z_t, u_t):
```
A_cont = [[0,     0,   -v*sin(θ)],
          [0,     0,    v*cos(θ)],
          [0,     0,         0  ]]

B_cont = [[cos(θ),  0],
          [sin(θ),  0],
          [0,       1]]
```

Discrete-time (forward Euler with dt):
```
A = I + dt * A_cont
B = dt * B_cont
```

---

## LQR Controller Implementation (`LQRController` class)

### Initialization
```python
def __init__(self, config):
    # Parse config dict for:
    # - lqr.horizon: number of steps (default 25)
    # - lqr.{x,y,theta,v,w}_cost: cost coefficients
    # - lqr.{v,w}_{min,max}: control bounds
    
    self._algo = LQRAlgorithm(...)  # Initialize algorithm
    self._step = 0                  # Receding-horizon step counter
    self._z_ref, self._u_ref        # Latest reference trajectory
```

### get_action() - Receding-Horizon Control

```python
def get_action(self, observation: NDArray, traj: StateActionTrajectory) -> NDArray:
    # 1. Update reference trajectory if new one provided
    if traj is not None:
        self._z_ref = traj.states
        self._u_ref = traj.actions
    
    # 2. Generate fallback reference (if none available)
    if self._z_ref is None:
        _, self._z_ref, self._u_ref = generate_reference_trajectory(...)
    
    # 3. Extract fixed-length reference window at current step
    z_w, u_w = self.sample_reference_window(
        self._z_ref, self._u_ref, self._step, self._algo.n
    )
    
    # 4. Solve finite-horizon LQR over this window
    z_sol, u_sol, _ = self._algo.solve(observation, t_0, z_w, u_w)
    
    # 5. Return first control, advance step counter
    action = u_sol[0]
    self._step += 1
    
    return clip(action, u_min, u_max)
```

**Key Design**: `sample_reference_window` pads with the terminal reference sample when the horizon extends beyond the end of the trajectory.

---

## ROS2 Integration (`ControllerNode`)

### Subscriptions
1. `/robot_pose` (PoseStamped) → extracts `[x, y, yaw]` via quaternion conversion
2. `/traj` (StateActionTrajectory msg) → converts to numpy arrays

### Publications
- `/cmd_vel` (Twist message) with `linear.x = v`, `angular.z = omega`

### Timer Callback (`_on_timer`)
- Runs at `control_rate` Hz (default 5 Hz from YAML)
- Calls backend.get_action() and publishes Twist
- Includes error handling (publishes zero command on exception)
- Publishes visualization markers for LQR trajectory

### Message Conversions
- **Pose → State**: `euler_from_quaternion(x, y, z, w) → (roll, pitch, yaw)`
- **State → Pose**: `quaternion_from_euler(roll, pitch, yaw) → (w, x, y, z)`
- **StateActionTrajectory**: Dataclass with N+1 states and N actions, dt, frame_id

---

## Reference Trajectory Generation

### Modes

**"to_goal"**: Simple proportional-heading controller
- Generates trajectory by following go-to-goal law at each step
- Proportional gains: k_rho=0.9 (distance), k_heading=1.8 (angle)
- Stops forward speed within goal_tol=0.1, aligns heading

**"straight"**: Constant speed, zero turn rate
- v = 0.35, omega = 0.0

**"s_curve"** (default): Sinusoidal modulation
- v = 0.35 + 0.05*cos(0.1*t)
- omega = 0.35*sin(0.25*t)

**"path"**: Custom trajectory (from planner)

---

## Cost Function Tuning (from controller_params.yaml)

```yaml
lqr:
  horizon: 20                # Lookahead steps
  x_cost: 5.0               # Penalize x error
  y_cost: 5.0               # Penalize y error
  theta_cost: 1.0           # Penalize heading error (less aggressive)
  v_cost: 0.1               # Penalize speed input (minimal)
  w_cost: 0.1               # Penalize turn rate (minimal)
  v_min: -0.2, v_max: 1.0   # Speed bounds
  w_min: -1.2, w_max: 1.2   # Turn rate bounds
```

---

## Data Flow Example: One Control Tick

```
1. Robot publishes /robot_pose with current position [x, y, yaw]
   
2. Planner publishes /traj (StateActionTrajectory)
   
3. ControllerNode timer fires at 5 Hz
   
4. _on_pose() callback: Store latest state as numpy [x, y, yaw]
   
5. _on_nom_traj() callback: Store latest trajectory
   
6. _on_timer() executes:
   a) Call backend.get_action(observation, traj)
      - LQRController.get_action():
        * Extract z_ref[step:step+horizon], u_ref[step:step+horizon]
        * Call LQRAlgorithm.solve(current_state, ...)
          - linearize_along_traj() → As, Bs matrices
          - compute_gains() → backward Riccati recursion → Ks, Ps
          - Forward rollout with tracking law → z_sol, u_sol
        * Return u_sol[0] (first control in sequence)
        * Increment step counter
   
   b) Create Twist message with [v, omega]
   
   c) Publish to /cmd_vel
   
7. Robot executes control for one time step
```

---

## Key Design Patterns

### 1. Backend-Frontend Separation
- **Backend**: `ControllerBackend` abstract interface with `get_action(obs, traj)`
- **Frontend**: ROS2 node handles all messaging/timing
- **Benefit**: Easy to swap different controllers (LQR, CBF, HJ)

### 2. Receding-Horizon Control
- Solve finite-horizon problem repeatedly
- Use only first control from solution
- Step forward, re-solve with updated reference
- Handles long trajectories with limited lookahead

### 3. Reference Window Padding
- When horizon extends past trajectory end, pad with terminal reference
- Allows smooth transition at trajectory end
- Prevents index-out-of-bounds errors

### 4. Angle Wrapping
- Heading error wrapped to [-π, π] using `arctan2(sin(Δθ), cos(Δθ))`
- Critical for stability near π/-π discontinuity

### 5. Discrete-Time Linearization
- Use forward Euler: `A = I + dt*A_cont`, `B = dt*B_cont`
- Consistent with discrete dynamics rollout
- dt typically 0.1-0.2 seconds

---

## Critical Implementation Notes from README

1. **Terminal Cost**: Don't forget `P_N = L` initialization in Riccati recursion
2. **Matrix Shapes**:
   - K_t: (2, 3) — 2 control dimensions, 3 state dimensions
   - A_t: (3, 3), B_t: (3, 2)
3. **Angle Wrapping**: Use `arctan2(sin(δθ), cos(δθ))` in tracking law
4. **Receding-Horizon**: Return first control, not entire sequence
5. **ROS2 Twist**: Set `linear.x` (linear velocity) and `angular.z` (yaw rate)
6. **Cost Matrix Structure**: Diagonal Q, R, L (only diagonal elements used)

---

## Refactoring Considerations for Your Project

### Current Strengths
1. Clean separation of algorithm and ROS2 messaging
2. Flexible reference trajectory generation
3. Pluggable backend architecture
4. Type hints throughout
5. Configuration via YAML

### Potential Improvements
1. **Error Handling**: Backend currently has bare try-catch; could be more specific
2. **Logging**: Minimal logging; would benefit from debug traces
3. **State Validation**: Could add pre/post-condition checks
4. **Vectorization**: Some loops could be vectorized further
5. **Testing**: No visible unit tests for algorithm
6. **Documentation**: Algorithm steps could have more comments
7. **Magic Numbers**: Some hardcoded values (goal_tol=0.1, gains) could be parameterized
8. **Observability**: No hooks for visualization of gains, trajectories during runtime

### Code Quality Notes
- Uses numpy type hints correctly (NDArray)
- Immutable patterns (creates new arrays, doesn't mutate)
- Good use of dataclasses for StateActionTrajectory
- Proper use of abstract base class pattern

