#!/usr/bin/env python3
"""Merges benchmark out dirs (farm lanes) into one: runs.csv rows, poses/ and logs/ files.
Usage: merge_runs.py OUT LANE_DIR [LANE_DIR ...]. Later rows win on a repeated run_id."""
import csv, os, shutil, sys
out, lanes = sys.argv[1], sys.argv[2:]
os.makedirs(out, exist_ok=True)
rows, fields = {}, None
for d in lanes:
    p = os.path.join(d, 'runs.csv')
    if not os.path.exists(p):
        continue
    r = csv.DictReader(open(p))
    fields = fields or r.fieldnames
    for row in r:
        rows[row['run_id']] = row
    for sub in ('poses', 'logs'):
        src = os.path.join(d, sub)
        if os.path.isdir(src):
            os.makedirs(os.path.join(out, sub), exist_ok=True)
            for f in os.listdir(src):
                shutil.copy2(os.path.join(src, f), os.path.join(out, sub, f))
if fields:
    with open(os.path.join(out, 'runs.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields); w.writeheader(); w.writerows(rows.values())
print(f'merged {len(rows)} rows from {len(lanes)} lanes into {out}')
