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
