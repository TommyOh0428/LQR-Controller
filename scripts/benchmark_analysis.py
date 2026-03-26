#!/usr/bin/env python3
"""Offline rosbag2 benchmark analysis for LQR vs DWB controllers.

Reads rosbag2 recordings and computes path-tracking metrics:
  - Cross-track error (mean, max, std)
  - Command velocity smoothness (RMS of dv/dt, domega/dt)
  - Time-to-goal
  - Comparison plots

Usage:
  python3 benchmark_analysis.py --bag lqr_run_1 --bag dwb_run_1
"""

import argparse
import sqlite3
import os
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path


def read_messages(bag_path, topic):
    """Read all messages from a rosbag2 SQLite database for a given topic."""
    db_path = os.path.join(bag_path, [f for f in os.listdir(bag_path) if f.endswith('.db3')][0])
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    cursor.execute("SELECT id FROM topics WHERE name = ?", (topic,))
    row = cursor.fetchone()
    if row is None:
        conn.close()
        return []

    topic_id = row[0]
    cursor.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
        (topic_id,)
    )
    messages = cursor.fetchall()
    conn.close()
    return messages


def get_msg_type(topic):
    """Map topic name to message type."""
    topic_types = {
        '/amcl_pose': PoseWithCovarianceStamped,
        '/cmd_vel': Twist,
        '/plan': Path,
    }
    return topic_types.get(topic)


def extract_trajectory(bag_path):
    """Extract robot trajectory from /amcl_pose as list of (t, x, y)."""
    messages = read_messages(bag_path, '/amcl_pose')
    trajectory = []
    for timestamp, data in messages:
        msg = deserialize_message(data, PoseWithCovarianceStamped)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        t = timestamp * 1e-9
        trajectory.append((t, x, y))
    return trajectory


def extract_plan(bag_path):
    """Extract the last planned path from /plan topic."""
    messages = read_messages(bag_path, '/plan')
    if not messages:
        return []

    _, data = messages[-1]
    msg = deserialize_message(data, Path)
    path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
    return path


def extract_cmd_vel(bag_path):
    """Extract velocity commands from /cmd_vel as list of (t, v, omega)."""
    messages = read_messages(bag_path, '/cmd_vel')
    cmds = []
    for timestamp, data in messages:
        msg = deserialize_message(data, Twist)
        t = timestamp * 1e-9
        cmds.append((t, msg.linear.x, msg.angular.z))
    return cmds


def point_to_segment_distance(px, py, ax, ay, bx, by):
    """Compute minimum distance from point (px,py) to line segment (ax,ay)-(bx,by)."""
    dx = bx - ax
    dy = by - ay
    seg_len_sq = dx * dx + dy * dy

    if seg_len_sq < 1e-12:
        return np.sqrt((px - ax) ** 2 + (py - ay) ** 2)

    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
    proj_x = ax + t * dx
    proj_y = ay + t * dy
    return np.sqrt((px - proj_x) ** 2 + (py - proj_y) ** 2)


def compute_cross_track_errors(trajectory, plan):
    """Compute cross-track error for each trajectory point."""
    if len(plan) < 2:
        return []

    errors = []
    for _, rx, ry in trajectory:
        min_dist = float('inf')
        for i in range(len(plan) - 1):
            d = point_to_segment_distance(rx, ry, plan[i][0], plan[i][1],
                                          plan[i + 1][0], plan[i + 1][1])
            if d < min_dist:
                min_dist = d
        errors.append(min_dist)
    return errors


def compute_smoothness(cmd_vel):
    """Compute RMS of dv/dt and domega/dt from cmd_vel timeseries."""
    if len(cmd_vel) < 2:
        return 0.0, 0.0

    dv_list = []
    domega_list = []
    for i in range(1, len(cmd_vel)):
        dt = cmd_vel[i][0] - cmd_vel[i - 1][0]
        if dt < 1e-6:
            continue
        dv_list.append((cmd_vel[i][1] - cmd_vel[i - 1][1]) / dt)
        domega_list.append((cmd_vel[i][2] - cmd_vel[i - 1][2]) / dt)

    rms_dv = np.sqrt(np.mean(np.array(dv_list) ** 2)) if dv_list else 0.0
    rms_domega = np.sqrt(np.mean(np.array(domega_list) ** 2)) if domega_list else 0.0
    return rms_dv, rms_domega


def analyze_bag(bag_path):
    """Run full analysis on a single rosbag."""
    name = os.path.basename(bag_path)
    print(f"\n{'=' * 60}")
    print(f"Analyzing: {name}")
    print(f"{'=' * 60}")

    trajectory = extract_trajectory(bag_path)
    plan = extract_plan(bag_path)
    cmd_vel = extract_cmd_vel(bag_path)

    if not trajectory:
        print("  WARNING: No /amcl_pose messages found")
        return None

    if not plan:
        print("  WARNING: No /plan messages found")

    if not cmd_vel:
        print("  WARNING: No /cmd_vel messages found")

    cte = compute_cross_track_errors(trajectory, plan) if plan else []
    rms_dv, rms_domega = compute_smoothness(cmd_vel)
    time_to_goal = trajectory[-1][0] - trajectory[0][0] if len(trajectory) > 1 else 0.0

    results = {
        'name': name,
        'mean_cte': np.mean(cte) if cte else 0.0,
        'max_cte': np.max(cte) if cte else 0.0,
        'std_cte': np.std(cte) if cte else 0.0,
        'rms_dv': rms_dv,
        'rms_domega': rms_domega,
        'time_to_goal': time_to_goal,
        'trajectory': trajectory,
        'plan': plan,
        'cmd_vel': cmd_vel,
        'cte': cte,
    }

    print(f"  Trajectory points:  {len(trajectory)}")
    print(f"  Plan waypoints:     {len(plan)}")
    print(f"  Cmd_vel messages:   {len(cmd_vel)}")
    print(f"  Mean cross-track:   {results['mean_cte']:.4f} m")
    print(f"  Max cross-track:    {results['max_cte']:.4f} m")
    print(f"  Std cross-track:    {results['std_cte']:.4f} m")
    print(f"  Smoothness (dv/dt): {rms_dv:.4f} m/s^2")
    print(f"  Smoothness (dw/dt): {rms_domega:.4f} rad/s^2")
    print(f"  Time to goal:       {time_to_goal:.2f} s")

    return results


def make_output_dir():
    """Create output/run_<N> directory, incrementing N based on existing runs."""
    base = 'output'
    os.makedirs(base, exist_ok=True)
    existing = [d for d in os.listdir(base) if d.startswith('run_') and os.path.isdir(os.path.join(base, d))]
    counts = []
    for d in existing:
        try:
            counts.append(int(d.split('_')[1]))
        except (IndexError, ValueError):
            pass
    next_count = max(counts) + 1 if counts else 1
    run_dir = os.path.join(base, f'run_{next_count}')
    os.makedirs(run_dir)
    return run_dir


def plot_comparison(all_results, run_dir):
    """Generate comparison plots for all analyzed bags."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    for r in all_results:
        traj = r['trajectory']
        if traj:
            times = [p[0] - traj[0][0] for p in traj]
            xs = [p[1] for p in traj]
            ys = [p[2] for p in traj]
            axes[0, 0].plot(xs, ys, label=r['name'])

        if r['plan']:
            plan_x = [p[0] for p in r['plan']]
            plan_y = [p[1] for p in r['plan']]
            axes[0, 0].plot(plan_x, plan_y, '--', alpha=0.5, label=f"{r['name']} plan")

    axes[0, 0].set_title('Trajectories')
    axes[0, 0].set_xlabel('x (m)')
    axes[0, 0].set_ylabel('y (m)')
    axes[0, 0].legend()
    axes[0, 0].set_aspect('equal')

    for r in all_results:
        if r['cte']:
            traj = r['trajectory']
            times = [p[0] - traj[0][0] for p in traj]
            axes[0, 1].plot(times[:len(r['cte'])], r['cte'], label=r['name'])
    axes[0, 1].set_title('Cross-Track Error')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Error (m)')
    axes[0, 1].legend()

    for r in all_results:
        if r['cmd_vel']:
            cv = r['cmd_vel']
            times = [p[0] - cv[0][0] for p in cv]
            vs = [p[1] for p in cv]
            axes[1, 0].plot(times, vs, label=r['name'])
    axes[1, 0].set_title('Linear Velocity Commands')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('v (m/s)')
    axes[1, 0].legend()

    for r in all_results:
        if r['cmd_vel']:
            cv = r['cmd_vel']
            times = [p[0] - cv[0][0] for p in cv]
            omegas = [p[2] for p in cv]
            axes[1, 1].plot(times, omegas, label=r['name'])
    axes[1, 1].set_title('Angular Velocity Commands')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('omega (rad/s)')
    axes[1, 1].legend()

    plt.tight_layout()
    postfix = '_vs_'.join(r['name'] for r in all_results)
    out_path = os.path.join(run_dir, f'benchmark_comparison_{postfix}.png')
    plt.savefig(out_path, dpi=150)
    print(f"\nPlot saved to {out_path}")
    plt.show()


def print_comparison_table(all_results):
    """Print side-by-side metric comparison."""
    print(f"\n{'=' * 70}")
    print("COMPARISON TABLE")
    print(f"{'=' * 70}")
    header = f"{'Metric':<25}"
    for r in all_results:
        header += f"  {r['name']:<20}"
    print(header)
    print("-" * 70)

    metrics = [
        ('Mean CTE (m)', 'mean_cte', '.4f'),
        ('Max CTE (m)', 'max_cte', '.4f'),
        ('Std CTE (m)', 'std_cte', '.4f'),
        ('Smoothness dv/dt', 'rms_dv', '.4f'),
        ('Smoothness dw/dt', 'rms_domega', '.4f'),
        ('Time to goal (s)', 'time_to_goal', '.2f'),
    ]

    for label, key, fmt in metrics:
        row = f"{label:<25}"
        for r in all_results:
            row += f"  {format(r[key], fmt):<20}"
        print(row)


def main():
    parser = argparse.ArgumentParser(description='Benchmark analysis for Nav2 controllers')
    parser.add_argument('--bag', action='append', required=True,
                        help='Path to rosbag2 directory (can specify multiple)')
    args = parser.parse_args()

    all_results = []
    for bag_path in args.bag:
        result = analyze_bag(bag_path)
        if result:
            all_results.append(result)

    if len(all_results) > 1:
        run_dir = make_output_dir()
        print(f"\nSaving results to {run_dir}/")

        import io, sys
        buf = io.StringIO()
        old_stdout = sys.stdout
        sys.stdout = buf
        print_comparison_table(all_results)
        sys.stdout = old_stdout
        table_text = buf.getvalue()
        print(table_text, end='')

        txt_path = os.path.join(run_dir, 'comparison.txt')
        with open(txt_path, 'w') as f:
            f.write(table_text)
        print(f"Table saved to {txt_path}")

        plot_comparison(all_results, run_dir)
    elif len(all_results) == 1:
        print("\nOnly one bag provided. Add --bag <path> for comparison.")


if __name__ == '__main__':
    main()
