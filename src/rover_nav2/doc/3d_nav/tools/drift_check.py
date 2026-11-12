#!/usr/bin/env python3
"""Per-goal drift check of a stack against its own earlier run: ok count (status succeeded) and
median time of ok runs. Args: pairs old_dir:new_dir under ~/.ros/rover_benchmark. Markdown out."""
import csv, statistics, sys
from pathlib import Path
B = Path.home() / '.ros/rover_benchmark'
def load(d):
    out = {}
    for r in csv.DictReader(open(B / d / 'runs.csv')):
        k = (r['world'].replace('.sdf', ''), r['goal_id'])
        o = out.setdefault(k, [0, 0, []]); o[1] += 1
        if r['status'] == 'succeeded':
            o[0] += 1; o[2].append(float(r['time_to_goal_s']) if r.get('time_to_goal_s') else float('nan'))
    return out
for pair in sys.argv[1:]:
    a, b = pair.split(':'); A, N = load(a), load(b)
    print(f'\n### {a} -> {b}\n\n| goal | old ok | new ok | old med s | new med s | ratio |\n|---|---|---|---|---|---|')
    for k in sorted(set(A) | set(N)):
        oa, nb = A.get(k, [0, 0, []]), N.get(k, [0, 0, []])
        ma = statistics.median(oa[2]) if oa[2] else float('nan')
        mb = statistics.median(nb[2]) if nb[2] else float('nan')
        print(f'| {k[0]}/{k[1]} | {oa[0]}/{oa[1]} | {nb[0]}/{nb[1]} | {ma:.0f} | {mb:.0f} | {mb / ma if ma == ma and mb == mb else float("nan"):.2f} |')
