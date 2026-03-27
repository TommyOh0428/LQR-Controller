# Evaluation Metrics

This document defines the metrics used to benchmark the LQR controller against the default DWB controller in Nav2.
All metrics are computed offline from ROS2 bag recordings using `scripts/benchmark_analysis.py`.

---

## 1. Cross-Track Error (Path Tracking Error)

**What it measures:** How far the robot strayed from the planned path at any point during navigation.

**Source topics:**
- `/amcl_pose` — robot position over time
- `/plan` — the global path planned by NavFN

**Formula:** For each robot position, compute the minimum perpendicular distance to the nearest segment of the planned path:


CTE(t) = min over all path segments { distance(robot_pos(t), segment_i) }


**Reported values:**
- Mean CTE — average deviation over the full run
- Max CTE — worst-case deviation
- Std CTE — consistency of tracking

**Lower is better.**



## 2. Command Smoothness

**What it measures:** How jerky or smooth the velocity commands are. A smooth controller produces gradual changes; a jerky one causes mechanical wear and unstable motion.

**Source topic:** `/cmd_vel`

**Formula:** RMS (root mean square) of the rate of change of linear and angular velocity:


smoothness_v     = sqrt( mean( (dv/dt)^2 ) )
smoothness_omega = sqrt( mean( (domega/dt)^2 ) )


where `dv/dt` and `domega/dt` are computed between consecutive `/cmd_vel` messages.

**Reported values:**
- `smoothness_v` — RMS linear acceleration (m/s²)
- `smoothness_omega` — RMS angular acceleration (rad/s²)

**Lower is better.**

---

## 3. Time to Goal

**What it measures:** How long the robot took from the start of navigation to reaching the goal.

**Source topic:** `/amcl_pose`

**Formula:**
```
time_to_goal = t_last - t_first
```

where `t_first` and `t_last` are the timestamps of the first and last pose messages in the recording.

**Note:** This metric is only meaningful if the robot successfully reached the goal (see Success Rate below).

**Lower is better.**

---

## 4. Success Rate

**What it measures:** Whether the robot actually reached the goal or got stuck/failed.

**Source:** Final robot position from `/amcl_pose` compared against the goal position.

**Formula:**
```
success = final_robot_position is within threshold distance of goal
success_rate = (successful_runs / total_runs) * 100%
```

A run is considered successful if the robot ends within a threshold (e.g. 0.3m) of the goal position.

**Implemented in:** `scripts/benchmark_analysis.py`. Also reports distance to goal and final position (x, y) where the robot stopped.

**Higher is better.**

---


