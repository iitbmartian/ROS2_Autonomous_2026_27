"""Markdown summary of a benchmark's runs.csv, per world and goal."""

import csv
import math
import statistics

NUMERIC = [
    ('time_to_goal_s', 'time to goal [s]'),
    ('path_length_m', 'path [m]'),
    ('recoveries', 'recoveries'),
    ('unsafe_entries', 'unsafe entries'),
    ('max_roll_deg', 'max roll [deg]'),
    ('max_pitch_deg', 'max pitch [deg]'),
    ('planner_time_s', 'planner [s]'),
    ('cpu_mean_pct', 'CPU mean [%]'),
    ('rtf', 'RTF'),
]


def read_rows(csv_path):
    with open(csv_path, newline='') as f:
        return list(csv.DictReader(f))


def _num(v):
    try:
        x = float(v)
    except (TypeError, ValueError):
        return None
    return None if math.isnan(x) else x


def mean_std(values):
    xs = [x for x in (_num(v) for v in values) if x is not None]
    if not xs:
        return '-'
    if len(xs) == 1:
        return f'{xs[0]:.2f}'
    return f'{statistics.mean(xs):.2f} +- {statistics.stdev(xs):.2f}'


def summarise(rows, title, notes=()):
    lines = [f'# {title}', '']
    for n in notes:
        lines.append(f'- {n}')
    if notes:
        lines.append('')
    groups = {}
    for r in rows:
        groups.setdefault((r['world'], r['goal_id']), []).append(r)
    head = ['world', 'goal', 'runs', 'success'] + [label for _, label in NUMERIC]
    lines.append('| ' + ' | '.join(head) + ' |')
    lines.append('|' + '---|' * len(head))
    for (world, goal), rs in sorted(groups.items()):
        ok = sum(int(r['success'] or 0) for r in rs)
        cells = [world, goal, str(len(rs)), f'{ok}/{len(rs)}']
        cells += [mean_std(r[k] for r in rs) for k, _ in NUMERIC]
        lines.append('| ' + ' | '.join(cells) + ' |')
    lines.append('')
    statuses = {}
    for r in rows:
        statuses[r['status']] = statuses.get(r['status'], 0) + 1
    lines.append('Run outcomes: ' + ', '.join(f'{k} {v}' for k, v in sorted(statuses.items())))
    notes_rows = [r for r in rows if r.get('note')]
    if notes_rows:
        lines += ['', 'Notes:', '']
        lines += [f"- `{r['run_id']}`: {r['note']}" for r in notes_rows]
    commits = sorted({r['git_commit'] for r in rows})
    shas = sorted({r['params_sha256'] for r in rows})
    lines += ['', f"Commits: {', '.join(commits)}. Nav2 params sha256: {', '.join(shas)}.", '']
    return '\n'.join(lines)
