#!/usr/bin/env python3
"""Headless Nav2 benchmark runner — no Gazebo / RViz required.

Runs controller × planner combos using a fake robot simulator, records
rosbags for offline analysis with benchmark_analysis.py.

Combos (edit COMBOS list to add more):
  lqr_navfn   — LQR controller  + NavFn planner
  lqr_smac    — LQR controller  + SMAC 2D planner
  dwb_navfn   — DWB controller  + NavFn planner
  dwb_smac    — DWB controller  + SMAC 2D planner

Prerequisites:
    colcon build --symlink-install --packages-select lqr_controller
    source install/setup.bash

Usage:
    python3 scripts/run_benchmarks.py
    python3 scripts/run_benchmarks.py --runs 3 --timeout 120
    python3 scripts/run_benchmarks.py --combos lqr_navfn dwb_navfn
"""

import argparse
import os
import shutil
import signal
import subprocess
import sys
import time

import yaml

# ── paths ────────────────────────────────────────────────────────────
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(BASE_DIR, 'scripts')
CONFIG_DIR = os.path.join(BASE_DIR, 'config')
RECORDINGS_DIR = os.path.join(BASE_DIR, 'recordings')
LOG_DIR = os.path.join(BASE_DIR, 'output', 'benchmark_logs')
TMP_DIR = '/tmp/nav2_benchmark'

MAP_YAML = '/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml'
FAIL_FAST_BT_XML = (
    '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/'
    'navigate_w_replanning_only_if_path_becomes_invalid.xml'
)

# ── navigation goal ─────────────────────────────────────────────────
START = {'x': -2.0, 'y': -0.5, 'yaw': 0.0}
GOAL = {'x': 2.0, 'y': 0.5, 'qz': 0.0, 'qw': 1.0}

# ── combos ───────────────────────────────────────────────────────────
# Each combo has its own dedicated config YAML that already contains the
# correct controller and planner settings — no overrides needed.
# (name, config_yaml_filename)
COMBOS = [
    ('lqr_navfn',  'lqr_navfn_params.yaml'),
    ('lqr_smac',   'lqr_smac_params.yaml'),
    ('dwb_navfn',  'dwb_navfn_params.yaml'),
    ('dwb_smac',   'dwb_smac_params.yaml'),
    ('mppi_navfn', 'mppi_navfn_params.yaml'),
    ('mppi_smac',  'mppi_smac_params.yaml'),
]

# ─────────────────────────────────────────────────────────────────────


def log(msg, level='INFO'):
    ts = time.strftime('%H:%M:%S')
    print(f'[{ts}] [{level}] {msg}', flush=True)


# ── YAML generation ─────────────────────────────────────────────────

def generate_params(config_filename, output_path):
    """Load a combo config YAML, inject runtime overrides, and save to tmp."""
    with open(os.path.join(CONFIG_DIR, config_filename)) as f:
        params = yaml.safe_load(f)

    # Use a fail-fast BT without recovery loops so failed runs return quickly
    # instead of spending the full action timeout in repeated recovery cycles.
    params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = FAIL_FAST_BT_XML

    # Ensure progress checker is permissive enough for controllers that use
    # high angular velocity on initial path entry (e.g. LQR).
    checker = params['controller_server']['ros__parameters'].get('progress_checker', {})
    checker.setdefault('plugin', 'nav2_controller::SimpleProgressChecker')
    if checker.get('required_movement_radius', 1.0) > 0.2:
        checker['required_movement_radius'] = 0.2
    if checker.get('movement_time_allowance', 0) < 30.0:
        checker['movement_time_allowance'] = 30.0
    params['controller_server']['ros__parameters']['progress_checker'] = checker

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w') as f:
        yaml.dump(params, f, default_flow_style=False)
    return output_path


# ── process helpers ──────────────────────────────────────────────────

def kill_pg(proc, sig=signal.SIGTERM):
    """Send *sig* to the entire process group."""
    try:
        os.killpg(os.getpgid(proc.pid), sig)
    except (ProcessLookupError, OSError):
        pass


NAV2_KILL_PATTERNS = [
    'fake_robot_node',
    'controller_server',
    'planner_server',
    'bt_navigator',
    'behavior_server',
    'waypoint_follower',
    'velocity_smoother',
    'smoother_server',
    'lifecycle_manager',
    'map_server',
    'ros2_bag',
    'rosbag2',
]


def purge_nav2_processes(wait_secs=5):
    """Kill any lingering nav2 / fake-robot / rosbag processes by name."""
    for pattern in NAV2_KILL_PATTERNS:
        subprocess.run(
            ['pkill', '-f', pattern],
            capture_output=True,
        )
    # Also reset the ROS 2 daemon so stale node listings are cleared
    try:
        subprocess.run(['ros2', 'daemon', 'stop'], capture_output=True, timeout=10)
    except (subprocess.TimeoutExpired, Exception):
        pass
    time.sleep(wait_secs)


def wait_for_action(action_name, timeout=60):
    """Block until a ROS 2 action server appears."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        r = subprocess.run(
            ['ros2', 'action', 'list'],
            capture_output=True, text=True, timeout=5,
        )
        if action_name in (r.stdout or ''):
            return True
        time.sleep(2)
    return False


def lifecycle_activate(node_name, timeout=30):
    """Drive a lifecycle node through configure → activate."""
    for transition in ('configure', 'activate'):
        deadline = time.time() + timeout
        while time.time() < deadline:
            r = subprocess.run(
                ['ros2', 'lifecycle', 'set', node_name, transition],
                capture_output=True, text=True, timeout=10,
            )
            if r.returncode == 0:
                break
            time.sleep(2)
        else:
            return False
        time.sleep(1)
    return True


# ── single benchmark run ────────────────────────────────────────────

def run_single(combo_name, run_idx, params_file, nav_timeout):
    """Launch nav2 stack + fake robot, record bag, send goal, tear down."""
    bag_name = f'{combo_name}_run_{run_idx}'
    bag_path = os.path.join(RECORDINGS_DIR, bag_name)

    log(f'--- {bag_name} ---')

    if os.path.exists(bag_path):
        shutil.rmtree(bag_path)

    os.makedirs(LOG_DIR, exist_ok=True)
    log_path = os.path.join(LOG_DIR, f'{bag_name}.log')
    log_file = open(log_path, 'w')

    procs = []

    def start(name, cmd, use_log=True):
        kw = dict(start_new_session=True)
        if use_log:
            kw.update(stdout=log_file, stderr=subprocess.STDOUT)
        p = subprocess.Popen(cmd, **kw)
        procs.append((name, p))
        return p

    nav_status = 'UNKNOWN'

    # Ensure no leftover processes from previous run
    log('  Purging any leftover processes …')
    purge_nav2_processes(wait_secs=5)

    try:
        # 1) fake robot  (publishes /clock, /odom, /scan, /amcl_pose, TF)
        log('  Starting fake robot …')
        start('fake_robot', [
            'python3', os.path.join(SCRIPTS_DIR, 'fake_robot_node.py'),
            '--ros-args',
            '-p', f'start_x:={START["x"]}',
            '-p', f'start_y:={START["y"]}',
            '-p', f'start_yaw:={START["yaw"]}',
        ])
        time.sleep(3)

        # 2) map_server
        log('  Starting map server …')
        start('map_server', [
            'ros2', 'run', 'nav2_map_server', 'map_server',
            '--ros-args',
            '-p', f'yaml_filename:={MAP_YAML}',
            '-p', 'use_sim_time:=true',
        ])
        time.sleep(3)

        # 3) activate map_server lifecycle
        log('  Activating map server …')
        if not lifecycle_activate('/map_server'):
            log('  FAILED to activate map server', 'ERROR')
            return False, 'FAILED_STARTUP'
        time.sleep(2)

        # 4) navigation stack  (controller, planner, bt_navigator, …)
        log('  Starting navigation stack …')
        start('navigation', [
            'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
            f'params_file:={params_file}',
            'use_sim_time:=true',
            'autostart:=true',
        ])

        # 5) wait for bt_navigator action server
        log('  Waiting for /navigate_to_pose action …')
        if not wait_for_action('/navigate_to_pose', timeout=90):
            log('  Nav2 action server never appeared', 'ERROR')
            return False, 'FAILED_STARTUP'
        time.sleep(5)          # let costmaps settle

        # 6) start bag recording
        log(f'  Recording → recordings/{bag_name}/')
        bag_proc = subprocess.Popen(
            ['ros2', 'bag', 'record', '-o', bag_path,
             '/tf', '/tf_static', '/cmd_vel', '/plan', '/odom', '/amcl_pose'],
            start_new_session=True,
        )
        procs.append(('bag_record', bag_proc))
        time.sleep(2)

        # 7) send navigation goal
        gx, gy, gqz, gqw = GOAL['x'], GOAL['y'], GOAL['qz'], GOAL['qw']
        goal_yaml = (
            f"{{pose: {{header: {{frame_id: 'map'}}, "
            f"pose: {{position: {{x: {gx}, y: {gy}, z: 0.0}}, "
            f"orientation: {{x: 0.0, y: 0.0, z: {gqz}, w: {gqw}}}}}}}}}"
        )
        log(f'  Goal → ({gx}, {gy})  timeout={nav_timeout}s')

        try:
            result = subprocess.run(
                ['ros2', 'action', 'send_goal',
                 '/navigate_to_pose',
                 'nav2_msgs/action/NavigateToPose',
                 goal_yaml],
                capture_output=True, text=True,
                timeout=nav_timeout,
            )
            stdout = result.stdout or ''
            if 'SUCCEEDED' in stdout:
                log('  Goal SUCCEEDED')
                nav_status = 'SUCCEEDED'
            elif 'ABORTED' in stdout:
                log('  Goal ABORTED')
                nav_status = 'ABORTED'
            else:
                log(f'  Goal finished — {stdout.strip()[-200:]}')
                nav_status = 'UNKNOWN'
        except subprocess.TimeoutExpired:
            log(f'  Goal timed out after {nav_timeout}s')
            nav_status = 'TIMED_OUT'

        time.sleep(2)

    except Exception as e:
        log(f'  Exception: {e}', 'ERROR')

    finally:
        # stop bag recorder first (SIGINT lets it flush)
        for name, p in procs:
            if name == 'bag_record':
                kill_pg(p, signal.SIGINT)
        time.sleep(3)

        # SIGINT everything else
        for name, p in reversed(procs):
            kill_pg(p, signal.SIGINT)
        time.sleep(3)

        # force-kill stragglers
        for name, p in reversed(procs):
            kill_pg(p, signal.SIGKILL)
            try:
                p.wait(timeout=5)
            except Exception:
                pass

        log_file.close()

        # Purge any processes that survived the SIGKILL sweep
        purge_nav2_processes(wait_secs=5)

    # verify bag was written
    if os.path.exists(bag_path):
        db3 = [f for f in os.listdir(bag_path) if f.endswith('.db3')]
        if db3:
            log(f'  Bag saved: recordings/{bag_name}/')
            return True, nav_status

    log('  No bag recorded!', 'WARN')
    return False, nav_status


# ── main ─────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Headless Nav2 benchmark runner (no Gazebo / RViz)')
    parser.add_argument(
        '--runs', type=int, default=3,
        help='Number of runs per combo (default: 3)')
    parser.add_argument(
        '--timeout', type=int, default=120,
        help='Navigation timeout in seconds per run (default: 120)')
    parser.add_argument(
        '--combos', nargs='+', default=None,
        help='Run only these combos (e.g. --combos lqr_navfn dwb_smac)')
    args = parser.parse_args()

    # ── preflight checks ─────────────────────────────────────────────
    try:
        from ament_index_python.packages import get_resources
        plugins = get_resources('nav2_core__pluginlib__plugin')
        if 'lqr_controller' not in plugins:
            log('lqr_controller plugin not registered.  '
                'Did you run:  source install/setup.bash ?', 'ERROR')
            sys.exit(1)
    except Exception as e:
        log(f'Could not verify plugin registration: {e}', 'WARN')

    if not os.path.isfile(MAP_YAML):
        log(f'Map not found: {MAP_YAML}', 'ERROR')
        sys.exit(1)

    # ── select combos ────────────────────────────────────────────────
    combos = COMBOS
    if args.combos:
        combos = [c for c in COMBOS if c[0] in args.combos]
        if not combos:
            log(f'No matching combos for: {args.combos}', 'ERROR')
            sys.exit(1)

    os.makedirs(RECORDINGS_DIR, exist_ok=True)
    total = len(combos) * args.runs
    current = 0
    results = {}

    log(f'Benchmarks: {total} runs  '
        f'({len(combos)} combos × {args.runs} runs)')
    log(f'Start ({START["x"]}, {START["y"]})  →  '
        f'Goal ({GOAL["x"]}, {GOAL["y"]})')
    log(f'Timeout {args.timeout}s per run')
    print()

    # ── run loop ─────────────────────────────────────────────────────
    for combo_name, config_filename in combos:
        params_file = os.path.join(TMP_DIR, f'{combo_name}_params.yaml')
        generate_params(config_filename, params_file)
        log(f'Params: {params_file}')

        for run_idx in range(1, args.runs + 1):
            current += 1
            log(f'\n===  [{current}/{total}]  {combo_name}  run {run_idx}  ===')
            bag_ok, nav_status = run_single(combo_name, run_idx, params_file, args.timeout)
            results[f'{combo_name}_run_{run_idx}'] = {
                'bag_ok': bag_ok,
                'nav_status': nav_status,
            }
            time.sleep(3)

    # ── summary ──────────────────────────────────────────────────────
    print('\n' + '=' * 55)
    print('  BENCHMARK RESULTS')
    print('=' * 55)
    for name, result in results.items():
        bag_status = 'BAG' if result['bag_ok'] else 'NO_BAG'
        print(f"  [{bag_status:6}] [{result['nav_status']:^9}]  {name}")

    ok_bags = [n for n, result in results.items() if result['bag_ok']]
    print(f'\n  Recorded {len(ok_bags)}/{len(results)} bags.')

    if ok_bags:
        bag_args = ' '.join(f'--bag recordings/{n}' for n in ok_bags)
        print('\n  Analyse with:')
        print(f'    python3 scripts/benchmark_analysis.py {bag_args}')
    print()


if __name__ == '__main__':
    main()
