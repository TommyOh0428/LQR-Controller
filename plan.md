# Project Plan — LQR Nav2 Controller Plugin

**Course**: CMPT 419, SFU
**Deadline**: April 17, 2026
**Team**: Tommy Oh, Ansh Aggarwal, Daniel Senteu, JunHang Wu
**Submission**: Poster session / presentation (no live demo required)

**Current status**: Controller implemented and integrated with Nav2. First benchmark complete (LQR outperforms DWB on CTE and smoothness). Goal-reaching reliability needs tuning. Hardware deployment not yet attempted.

---

## Phase 1 — Foundation & Plugin Skeleton [COMPLETE]

**Objective**: Set up the dev environment, create the ROS2 package, and register a skeleton Nav2 controller plugin.

**Files created/modified**:
- [x] `.devcontainer/Dockerfile`, `devcontainer.json`
- [x] `src/lqr_controller/package.xml`
- [x] `src/lqr_controller/CMakeLists.txt`
- [x] `src/lqr_controller/lqr_controller_plugin.xml`
- [x] `src/lqr_controller/include/lqr_controller/lqr_controller.hpp`
- [x] `src/lqr_controller/src/lqr_controller.cpp` (skeleton)
- [x] `.gitignore`, `ros_entrypoint.sh`

**Exit criteria**:
- [x] Dev container builds successfully
- [x] `colcon build --packages-select lqr_controller` succeeds
- [x] `ros2 plugin list | grep lqr` returns `lqr_controller/LQRController`

**Closing commit**: `Setup controller and lqr backend` (a98a653)
**Owner**: Nick (nick/set-up branch), Tommy

---

## Phase 2 — LQR Algorithm Implementation [COMPLETE]

**Objective**: Implement the full LQR math (DARE solver, linearization, body-frame error) and integrate it into the Nav2 plugin.

**Files created/modified**:
- [x] `src/lqr_controller/include/lqr_controller/lqr_solver.hpp`
- [x] `src/lqr_controller/src/lqr_solver.cpp`
- [x] `src/lqr_controller/src/lqr_controller.cpp` (full implementation)
- [x] `src/lqr_controller/include/lqr_controller/lqr_controller.hpp` (full header)

**Exit criteria**:
- [x] DARE solver converges with default Q/R values (confirmed by log output)
- [x] K matrix logged at INFO level during configure()
- [x] `computeVelocityCommands` produces non-zero `/cmd_vel` when given a path
- [x] Robot moves toward goal in Gazebo simulation (may not reach it reliably)

**Closing commit**: `Setup controller and lqr backend` (a98a653)
**Owner**: Tommy, Ansh (implement-controller branch)

---

## Phase 3 — Nav2 Config & First Benchmark [COMPLETE]

**Objective**: Create full Nav2 param configs for LQR and DWB, run first benchmark comparison.

**Files created/modified**:
- [x] `config/nav2_params.yaml` (LQR controller config)
- [x] `config/dwb_params.yaml` (DWB baseline config)
- [x] `scripts/benchmark_analysis.py`
- [x] `docs/metrics.md`
- [x] `recordings/lqr_run_1/metadata.yaml`
- [x] `recordings/dwb_run_1/metadata.yaml`
- [x] `output/run_1/comparison.txt`

**Exit criteria**:
- [x] Both LQR and DWB configs launch successfully with `tb3_simulation_launch.py`
- [x] `benchmark_analysis.py` produces comparison table and plots
- [x] At least one complete LQR vs DWB benchmark run recorded

**Closing commit**: `add evaluation metrics doc and success rate to benchmark script` (aaf4f86)
**Owner**: Ansh (Nav2-config branch)

---

## Phase 4 — Tuning for Reliable Goal-Reaching [-] IN PROGRESS

**Objective**: Tune LQR Q/R matrices and controller parameters so the robot reliably reaches goals in simulation.

**Files to modify**:
- [ ] `config/nav2_params.yaml` (Q, R, lookahead, desired_speed)
- [ ] `tuning-log.md` (append entries for each experiment)

**Exit criteria**:
- [ ] Robot reaches goal in ≥ 4 out of 5 runs in `turtlebot3_world` (success rate ≥ 80%)
- [ ] Mean cross-track error < 0.15m
- [ ] No oscillation visible — robot tracks path smoothly
- [ ] `progress_checker` does not abort the run (timeout tuned appropriately)
- [ ] Results documented in `tuning-log.md` with commit hashes

**Closing commit**: `tune: finalize sim gains — 80%+ success rate, mean CTE < 0.15m`
**Owner**: All (anyone can run tuning experiments)

---

## Phase 5 — Multi-Run Statistical Benchmarking [ ] NOT STARTED

**Objective**: Run multiple benchmark trials (≥ 3 per controller) for statistical validity. Compare LQR vs DWB with confidence.

**Files to create/modify**:
- [ ] `recordings/lqr_run_2/`, `lqr_run_3/`, etc.
- [ ] `recordings/dwb_run_2/`, `dwb_run_3/`, etc.
- [ ] `output/run_2/`, `run_3/`, etc.
- [ ] Possibly extend `benchmark_analysis.py` for multi-run aggregation

**Exit criteria**:
- [ ] ≥ 3 LQR runs and ≥ 3 DWB runs with same start/goal in same world
- [ ] Mean and std reported for each metric across runs
- [ ] Comparison table with statistical summary saved in `output/`
- [ ] Clear statement: "LQR is better/worse/comparable on [metric] with N runs"

**Closing commit**: `test: complete multi-run LQR vs DWB benchmark — N runs each`
**Owner**: Daniel, JunHang

---

## Phase 6 — Hardware Deployment [ ] NOT STARTED

**Objective**: Deploy the LQR controller on a physical TurtleBot3 Burger and validate real-world performance.

**Files to create/modify**:
- [ ] `config/nav2_params.yaml` (may need hardware-specific tuning)
- [ ] `tuning-log.md` (hardware tuning entries)
- [ ] `recordings/lqr_hw_run_1/`, etc.
- [ ] Possibly a hardware-specific params file if gains differ significantly

**Entry criteria** (must be met before starting Phase 6):
- [ ] Phase 4 exit criteria met (reliable in sim)
- [ ] Pre-built map of the test environment available
- [ ] Hardware pre-deployment checklist completed (see `.cursor/rules/hardware.mdc`)

**Exit criteria**:
- [ ] Robot reaches goal on physical TurtleBot3 at least once
- [ ] Hardware run recorded as rosbag with metadata
- [ ] Any hardware-specific tuning documented in `tuning-log.md`
- [ ] Comparison note: how hardware performance compares to simulation

**Closing commit**: `test: hardware — first successful LQR run on physical TB3`
**Owner**: All (requires physical access to robot)

---

## Phase 7 — Poster, Documentation & Final Polish [ ] NOT STARTED

**Objective**: Prepare poster/presentation materials, finalize README, clean up repo.

**Files to create/modify**:
- [ ] `README.md` (final polish: results summary, clear instructions)
- [ ] Poster content (figures from `output/`, architecture diagram from AGENT.md)
- [ ] `docs/` — any final implementation notes

**Exit criteria**:
- [ ] README has clear build/run/benchmark instructions (already mostly done)
- [ ] README includes a results summary table (LQR vs DWB key metrics)
- [ ] Poster figures generated and saved
- [ ] No uncommitted changes on any branch
- [ ] All branches merged to main via PR

**Closing commit**: `docs: finalize README and poster materials for submission`
**Owner**: All

---

## Known Risks

1. **Goal-reaching reliability**: First benchmark showed LQR has great CTE but the commit history notes "lqr is not reaching the goal." This is the most critical issue to fix in Phase 4.
2. **Hardware access**: Physical TurtleBot3 availability may be limited. Start Phase 6 early if hardware is time-shared.
3. **Uncommitted work on `ansh` branch**: Currently every file in the repo is modified but uncommitted on the `ansh` branch. This must be resolved immediately — either commit or stash.
4. **Single benchmark run**: Only one LQR vs DWB comparison exists. Needs N ≥ 3 for any statistical claim.
5. **Poster format unknown**: Submission format is a poster session but exact requirements are unclear. Plan for: architecture diagram, results table, 2-3 key plots.

## Open Questions

- Exact poster requirements (size, format, what must be included)
- Whether a written report is also required
- Hardware lab schedule / TurtleBot3 availability
- Whether testing on multiple Gazebo worlds is expected

---

## Session Prompts

Copy these word-for-word at the start and end of each session.

### Session Start Prompt (all session types)

```
Read session.md, plan.md, and tuning-log.md. Then:
1. State which phase we are in and what the exit criteria are.
2. Run `git log --oneline -5` and confirm what commits have been made since last session.
3. Run `git status` to check for uncommitted changes.
4. State the single goal for this session.
5. If this is a hardware session, run through the full pre-deployment checklist in .cursor/rules/hardware.mdc before anything else.
```

### Mid-Session Commit Prompt

```
Check: is the work done since the last commit a standalone logical change?
If yes, draft a commit message and commit now before continuing.
If a parameter/gain change happened alongside a code change, make two separate commits.
Show me the commit message for review before committing.
```

### Session End Prompt

```
1. Update session.md with what was done, commits made, and decisions.
2. Update plan.md: check off completed items, update the "Current status" line at the top.
3. If tuning happened, append results to tuning-log.md.
4. Run `git status` — confirm no uncommitted changes remain, or explain why they are staged for next session.
5. State what the next session should start with (one specific task).
```

### Tuning Session Start Prompt

```
Read session.md, plan.md, and tuning-log.md. Then:
1. What are the current Q and R values in config/nav2_params.yaml?
2. What was the last tuning experiment result?
3. What is the hypothesis for this session's tuning change?
4. Confirm we are only changing ONE parameter this session.
```

### Tuning Session End Prompt

```
1. Append the experiment to tuning-log.md with all fields filled.
2. Commit the parameter change with `tune:` prefix, exact before/after values and result.
3. Update session.md and plan.md.
4. State: keep these gains, revert, or what to try next.
```
