# Controller Bug Fixes — Goal-Reaching Reliability
**Date**: 2026-03-27
**Author**: Ansh

## Summary

Fixed 6 bugs in the LQR controller that together caused the "not reaching goal" problem observed in the first benchmark run (commit 9e3a5bd). The controller had excellent path-tracking (0.10m mean CTE) but could not reliably converge on the goal due to backward path snapping, overshooting, ignored goal checks, and corrupted heading references.

## Changes Made

### 1. Forward-only closest-point search (Critical)

**Files**: `lqr_controller.hpp`, `lqr_controller.cpp`

**Problem**: `findClosestPoint()` scanned the entire path on every control cycle. When the robot was near the goal on a path that passed close to earlier segments, the closest point could snap backwards, causing the robot to drive away from the goal and re-track old segments.

**Fix**: Added `last_closest_idx_` member variable that tracks progress along the path. The search now starts from `last_closest_idx_ - 2` (small lookback for localization jitter) instead of index 0. Reset to 0 in `setPlan()` when a new path arrives.

### 2. Goal-proximity speed ramp-down (Critical)

**Files**: `lqr_controller.hpp`, `lqr_controller.cpp`, `nav2_params.yaml`

**Problem**: The controller always commanded `desired_speed_ (0.2 m/s)` plus the LQR correction, regardless of distance to goal. At 0.2 m/s, the robot overshoots the 0.25m goal tolerance, turns around, overshoots again — oscillating around the goal indefinitely.

**Fix**: Added `computeRemainingPathLength()` helper and `goal_slowdown_radius` parameter (default 1.0m). When remaining path distance < `goal_slowdown_radius`, the reference speed scales down linearly: `v_ref = desired_speed * (remaining / radius)`, clamped to a 5% minimum to avoid stalling. This lets the robot decelerate smoothly into the goal tolerance zone.

### 3. Goal checker integration (Critical)

**Files**: `lqr_controller.cpp`

**Problem**: The `goal_checker` parameter in `computeVelocityCommands()` was explicitly discarded (`/*goal_checker*/`). The controller never checked if Nav2 considered the goal reached, so it kept commanding velocities even when the robot was within tolerance.

**Fix**: Before computing any control, call `goal_checker->isGoalReached()` with the current pose and the last pose on the path. If the goal is reached, return a zero-velocity `TwistStamped` immediately.

### 4. Identity quaternion fallback for path heading (Moderate)

**Files**: `lqr_controller.hpp`, `lqr_controller.cpp`

**Problem**: NavFn planner often outputs path poses with identity quaternion `(0, 0, 0, 1)`, meaning yaw = 0 for all waypoints. The body-frame error computation depends on `theta_ref` being the actual path direction — with `theta_ref = 0` everywhere, the `e_long`/`e_lat` decomposition was incorrect (the reference frame didn't rotate with the path).

**Fix**: Added `computePathHeading()` helper that detects identity quaternions (`w ≈ 1, x ≈ y ≈ z ≈ 0` within 1e-3) and falls back to `atan2(dy, dx)` between adjacent waypoints. For the last waypoint, it uses the direction from the previous waypoint. This was called out in AGENT.md but not implemented.

### 5. Progress checker timeout relaxed (Moderate)

**File**: `config/nav2_params.yaml`

**Problem**: `required_movement_radius: 0.5` with `movement_time_allowance: 10.0` meant the robot had to move 0.5m every 10 seconds or Nav2 would abort the goal. During fine approach to the goal, the robot is doing small position adjustments within ~0.25m and may not cover 0.5m in 10 seconds, causing a false "stuck" detection.

**Fix**: Changed to `required_movement_radius: 0.25` (matches goal tolerance) and `movement_time_allowance: 20.0` (more time for approach maneuvers).

### 6. Single-pose path guard in findLookaheadPoint (Minor)

**File**: `lqr_controller.cpp`

**Problem**: `global_plan_.poses.size() - 1` on an unsigned `size_t` wraps to a huge number if size is 1. While the empty-path check catches size == 0, a single-pose path would not be caught.

**Fix**: Added early return `if (global_plan_.poses.size() < 2) return closest_idx;` at the top of `findLookaheadPoint()`.

## New Parameter

| Parameter | Default | Description |
|-----------|---------|-------------|
| `goal_slowdown_radius` | 1.0 | Distance from goal (meters along path) at which speed starts ramping down. Set to 0 to disable. |

Added to `nav2_params.yaml` under `FollowPath` section.

## Decisions

- **Forward search lookback window = 2 indices**: Enough to handle minor localization jitter without allowing true backward snaps. Could be increased if AMCL jumps are large.
- **Slowdown minimum speed = 5% of desired_speed**: Prevents the robot from stalling completely before the goal_checker triggers. At 0.2 m/s desired, this is 0.01 m/s — enough to creep into tolerance.
- **Identity quaternion threshold = 1e-3**: Conservative enough to catch NavFn's exact identity quaternions without false-positiving on intentionally-set orientations.

## Issues / Blockers

None. These are code-only changes that need to be rebuilt and tested in simulation to confirm they fix the goal-reaching problem.

## Next Steps

1. Rebuild: `colcon build --symlink-install --packages-select lqr_controller`
2. Run simulation with LQR config and send a navigation goal
3. Confirm robot reaches goal reliably (≥ 4/5 runs)
4. If successful, proceed to Phase 4 tuning (adjusting Q/R for optimal performance)
