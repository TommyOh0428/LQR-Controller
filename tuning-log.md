# Tuning Log — LQR Controller

Append-only. Never overwrite previous entries.

---

## Experiment 0 — 2026-03-23 (baseline)

- **Who**: Ansh
- **Starting gains**: Q = diag(1.0, 3.0, 1.0), R = diag(1.0, 0.5)
- **Changed**: N/A — this is the initial baseline
- **Hypothesis**: Default gains from AGENT.md spec
- **Test conditions**: sim, turtlebot3_world, default spawn → goal (2.0, 0.5)
- **Result**: mean CTE = 0.1015m, max CTE = 0.2554m, time = 30.44s, success = unclear (commit says "not reaching the goal")
- **Conclusion**: Good tracking error but goal-reaching is unreliable. Need to investigate why robot fails to reach goal.
- **Next experiment**: Check if progress_checker is aborting too early, or if controller oscillates near goal
- **Commit**: 9e3a5bd

---

## Experiment 1 — 2026-03-27 (post-bugfix re-baseline)

- **Who**: Ansh
- **Starting gains**: Q = diag(1.0, 3.0, 1.0), R = diag(1.0, 0.5) (unchanged from Exp 0)
- **Changed**: No Q/R changes. 6 controller bugs fixed (see `docs/2026-03-27_controller-bug-fixes.md`):
  1. Forward-only closest-point search (prevents backward snap)
  2. Path-geometry heading fallback (NavFn identity quaternion fix)
  3. Goal-proximity slowdown ramp (new param `goal_slowdown_radius: 1.0`)
  4. `goal_checker` integration (zero velocity at goal)
  5. `progress_checker` relaxed (0.25m / 20s)
  6. Single-pose path guard in `findLookaheadPoint`
- **Hypothesis**: Goal-reaching failure was caused by code bugs, not suboptimal gains
- **Test conditions**: sim, turtlebot3_world, default spawn → goal (2.0, 0.5)
- **Result**: Robot reached goal. `[controller_server]: Reached the goal!` and `[bt_navigator]: Goal succeeded` confirmed in logs. Formal metrics (CTE, time) not yet re-measured with `benchmark_analysis.py`.
- **Conclusion**: Keep. Fixes resolved goal-reaching. Gains may still need tuning, but the robot now reliably completes the navigation task.
- **Next experiment**: Run 5 repeated trials same start/goal to measure success rate and mean CTE. If ≥80% success and CTE <0.15m, Phase 4 exit criteria are met without any Q/R tuning.
- **Commits**: cc55b0a, 1d4a366, f32b14e, fd2ccde

---

## Experiment 2 — 2026-04-03 (5-run validation, Phase 4 exit)

- **Who**: Nick
- **Starting gains**: Q = diag(1.0, 3.0, 1.0), R = diag(1.0, 0.5) (unchanged from Exp 1)
- **Changed**: No Q/R changes. Benchmark script fixed (was reporting ~2.3m CTE due to two bugs):
  1. Plan selection: used last replan fragment instead of longest (full) plan
  2. Trajectory source: used /amcl_pose (~1 Hz, 22 pts) instead of /odom + TF (~20 Hz, 900-5000 pts)
- **Hypothesis**: Current gains already meet Phase 4 exit criteria — bug fixes from Exp 1 were sufficient
- **Test conditions**: sim, turtlebot3_world, default spawn → goal (2.0, 0.5), 5 repeated trials
- **Results**:

| Run | Mean CTE (m) | Max CTE (m) | Time (s) | Reached goal | Dist to goal (m) |
|-----|-------------|-------------|----------|--------------|-------------------|
| 1   | 0.1591      | 0.2595      | 56.17    | Yes          | 0.287             |
| 2   | 0.1090      | 0.2440      | 33.66    | Yes          | 0.184             |
| 3   | 0.1387      | 0.2613      | 203.15*  | Yes          | 0.168             |
| 4   | 0.1262      | 0.2959      | 71.55    | Yes          | 0.174             |
| 5   | 0.1464      | 0.3335      | 36.79    | Yes          | 0.089             |

  *Run 3 time inflated — bag recording stopped late; robot reached goal at similar time as other runs.

  - **Success rate**: 5/5 = 100% (target ≥ 80%)
  - **Mean CTE across runs**: ~0.136m (target < 0.15m)
  - **Smoothness**: consistent dv/dt ~0.15, dw/dt ~1.0 — no oscillation observed
  - **progress_checker**: never aborted

- **Conclusion**: All Phase 4 exit criteria met with original gains. No Q/R tuning was needed — the 6 bug fixes from Exp 1 were the key. Keep current gains.
- **Next**: Move to Phase 5 — multi-run 3×2 benchmarking (LQR/DWB/MPPI × NavFn/Smac)
- **Commit**: TBD
