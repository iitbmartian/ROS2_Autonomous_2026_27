#!/usr/bin/env python3
"""Per-goal comparison of the final-suspension stages (2026-11-10_*), markdown to stdout.

    python3 compare_final.py [tag ...]   default: 3d_gt_ack_critic_v2 3d_gt_gate_final 2d_base_final

Cells are successes/runs, reached in brackets (ground truth ends within 0.5 m, heading and
Nav2's verdict ignored, from final_error_m so older runs score too), then the median time of the
successful runs. Totals count successes,
unsafe entries and recoveries. success is the benchmark's column, so refusals scored as expected
(lane35) count as successes.
"""
import csv
import statistics
import sys
from collections import defaultdict
from pathlib import Path

B = Path.home() / '.ros/rover_benchmark'
tags = sys.argv[1:] or ['3d_gt_ack_critic_v2', '3d_gt_gate_final', '2d_base_final']
data, goals = {}, []
for t in tags:
    f = B / f'2026-11-10_{t}' / 'runs.csv'
    d = defaultdict(list)
    if f.exists():
        for r in csv.DictReader(open(f)):
            k = (r['world'].replace('.sdf', ''), r['goal_id'])
            d[k].append(r)
            if k not in goals:
                goals.append(k)
    data[t] = d


def reached(r):
    if r['goal_id'].startswith('lane35'):
        return r['success'] == '1'
    try:
        return float(r['final_error_m']) <= 0.5
    except ValueError:
        return False


def cell(rows):
    if not rows:
        return '-'
    ok = [r for r in rows if r['success'] == '1']
    times = [float(r['time_to_goal_s']) for r in ok if r['status'] == 'succeeded']
    s = f'{len(ok)}/{len(rows)} ({sum(map(reached, rows))})'
    return s + (f', {statistics.median(times):.0f} s' if times else '')


print('| world | goal | ' + ' | '.join(tags) + ' |')
print('|---|---|' + '---|' * len(tags))
for k in goals:
    print(f'| {k[0]} | {k[1]} | ' + ' | '.join(cell(data[t].get(k, [])) for t in tags) + ' |')
row = []
for t in tags:
    rs = [r for v in data[t].values() for r in v]
    ok = sum(r['success'] == '1' for r in rs)
    uns = sum(int(r['unsafe_entries'] or 0) for r in rs)
    rec = sum(int(r['recoveries'] or 0) for r in rs)
    row.append(f'{ok}/{len(rs)} ({sum(map(reached, rs))}), unsafe {uns}, rec {rec}')
print('| total | | ' + ' | '.join(row) + ' |')
