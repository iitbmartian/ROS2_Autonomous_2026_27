"""ros2 run rover_nav2 benchmark: run goal sets against a navigation stack and score them.

    ros2 run rover_nav2 benchmark --worlds marsyard ledge bars --runs 5 --config 2d
    ros2 run rover_nav2 benchmark --worlds marsyard --runs 5 --config 3d --out ~/bench/3d
    ros2 run rover_nav2 benchmark --report-only ~/.ros/rover_benchmark/<dir>

Results go to --out (default ~/.ros/rover_benchmark/<date>_<config>): runs.csv, appended after
every run, summary.md, one launch log, one ground-truth pose CSV and one RTAB-Map database per
run. Rerunning with the same --out skips runs already in runs.csv, so an interrupted benchmark
resumes where it stopped.
"""

import argparse
import csv
import datetime
import math
import os
import sys

import yaml

from rover_nav2.benchmark.report import read_rows, summarise


def default_goals_dir():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('rover_nav2'), 'benchmark', 'goals')


def default_params_file():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('rover_nav2'), 'config',
                        'nav2_params_sim.yaml')


def load_goalset(goals_dir, name):
    with open(os.path.join(goals_dir, f'{name}.yaml')) as f:
        gs = yaml.safe_load(f)
    gs['name'] = name
    return gs


NOTES = [
    'Unsafe entries count ground-truth roll or pitch beyond the tip limits from '
    'rover_physical.yaml (placeholder values). Footprint-over-terrain entries need the '
    'Phase 1 ground-truth heightmaps and are not counted yet.',
    'Planner time is one ComputePathToPose call at the start of each run. Mid-run replans are '
    'not timed.',
    'Success needs Nav2 SUCCEEDED and the ground-truth position within the success radius.',
    'RTF is sim time over wall time during the goal. CPU is the stack process tree, '
    '100 percent = one core.',
]


def write_summary(out_dir, title, extra_notes):
    rows = read_rows(os.path.join(out_dir, 'runs.csv'))
    with open(os.path.join(out_dir, 'summary.md'), 'w') as f:
        f.write(summarise(rows, title, list(extra_notes) + NOTES))
    return os.path.join(out_dir, 'summary.md')


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--worlds', nargs='+', default=['marsyard', 'ledge', 'bars'],
                   help='goal set names in the goals directory')
    p.add_argument('--goals', nargs='*', default=None, help='only these goal ids')
    p.add_argument('--runs', type=int, default=5, help='repetitions per goal')
    p.add_argument('--config', choices=['2d', '3d'], default='2d')
    p.add_argument('--params-file', default=None,
                   help='2d only: Nav2 params file passed to pipeline.launch.py')
    p.add_argument('--out', default=None)
    p.add_argument('--goals-dir', default=None)
    p.add_argument('--domain', type=int, default=42, help='ROS_DOMAIN_ID for the benchmark')
    p.add_argument('--partition', default='rover_benchmark', help='GZ_PARTITION')
    p.add_argument('--success-radius', type=float, default=0.5, help='m, ground truth')
    p.add_argument('--startup-timeout', type=float, default=600.0, help='wall s to Nav2 active')
    p.add_argument('--plan-timeout', type=float, default=300.0,
                   help='wall s for the first plan to succeed')
    p.add_argument('--wall-factor', type=float, default=40.0,
                   help='wall timeout = goal timeout_s times this, if the sim stalls')
    p.add_argument('--repo', default=os.environ.get('ROVER_REPO', '/home/ram/ROS2_Autonomous_2026_27'))
    p.add_argument('--report-only', default=None, metavar='OUT_DIR')
    return p.parse_args(argv)


def main(argv=None):
    a = parse_args(sys.argv[1:] if argv is None else argv)
    if a.report_only:
        print(write_summary(a.report_only, f'Benchmark {os.path.basename(a.report_only)}', []))
        return 0

    # The runner joins the stack's isolated domain, so these must be set before rclpy starts.
    env = dict(os.environ, ROS_DOMAIN_ID=str(a.domain), GZ_PARTITION=a.partition,
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k: env[k] for k in
                       ('ROS_DOMAIN_ID', 'GZ_PARTITION', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})

    import rclpy
    from rover_nav2.benchmark import metrics
    from rover_nav2.benchmark.runner import CSV_FIELDS, Runner
    from rover_traversability import physical_params

    phys, placeholders = physical_params.load()
    warn = physical_params.placeholder_warning(placeholders)
    if warn:
        print(f'WARNING: {warn}', flush=True)
    roll_lim, pitch_lim = metrics.tip_limits(phys)
    print(f'unsafe limits: roll {math.degrees(roll_lim):.2f} deg, '
          f'pitch {math.degrees(pitch_lim):.2f} deg', flush=True)

    goals_dir = a.goals_dir or default_goals_dir()
    params = a.params_file or default_params_file()
    stamp = datetime.datetime.now().strftime('%Y-%m-%d')
    out = os.path.expanduser(a.out or f'~/.ros/rover_benchmark/{stamp}_{a.config}')
    for sub in ('logs', 'poses', 'db'):
        os.makedirs(os.path.join(out, sub), exist_ok=True)
    csv_path = os.path.join(out, 'runs.csv')
    done = {r['run_id'] for r in read_rows(csv_path)} if os.path.exists(csv_path) else set()

    def log(msg):
        line = f'{datetime.datetime.now():%H:%M:%S} {msg}'
        print(line, flush=True)
        with open(os.path.join(out, 'benchmark.log'), 'a') as f:
            f.write(line + '\n')

    rclpy.init()
    runner = Runner(a, phys, roll_lim, pitch_lim, params, log)
    log(f'config {a.config}, worlds {a.worlds}, {a.runs} runs per goal, out {out}, '
        f'commit {runner.commit}, params {params}')
    try:
        for name in a.worlds:
            gs = load_goalset(goals_dir, name)
            for goal in gs['goals']:
                if a.goals and goal['id'] not in a.goals:
                    continue
                for rep in range(1, a.runs + 1):
                    run_id = f"{a.config}_{name}_{goal['id']}_r{rep}"
                    if run_id in done:
                        continue
                    row = runner.run_one(gs, goal, rep, out, env)
                    new = not os.path.exists(csv_path)
                    with open(csv_path, 'a', newline='') as f:
                        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
                        if new:
                            w.writeheader()
                        w.writerow(row)
    except KeyboardInterrupt:
        log('interrupted, runs.csv holds every finished run')
    finally:
        runner.shutdown()
        rclpy.try_shutdown()
    if os.path.exists(csv_path):
        log(f"summary: {write_summary(out, f'Benchmark {a.config} stack', [warn] if warn else [])}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
