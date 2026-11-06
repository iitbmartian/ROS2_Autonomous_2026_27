#!/usr/bin/env python3
"""Phase 3 gate table from two benchmark runs.csv (3D vs baseline): per world unsafe entries,
recoveries, successes, and time to goal on goals the baseline reaches with 0 unsafe entries (mean over
those reps), gate 1.5x (Ram, 2026-11-07, was 1.10).
Usage: phase3_gate.py 3D_RUNS BASE_RUNS [out.csv]"""
import csv, sys, collections
def load(p):
    rows = [r for r in csv.DictReader(open(p)) if r['status'] != 'startup_failed']
    return rows
a, b = load(sys.argv[1]), load(sys.argv[2])
def per_world(rows, safe_only=False):
    W = collections.defaultdict(lambda: dict(n=0, ok=0, unsafe=0, rec=0, ttg=collections.defaultdict(list)))
    for r in rows:
        w = W[r['world'].replace('.sdf', '')]
        w['n'] += 1; w['ok'] += int(r['success'])
        w['unsafe'] += int(r['unsafe_entries'] or 0); w['rec'] += int(r['recoveries'] or 0)
        # Baseline times count only from safe runs, 3D drives around what an unsafe 2D run drives over.
        if r['success'] == '1' and r['status'] == 'succeeded' and not (safe_only and int(r['unsafe_entries'] or 0)):
            w['ttg'][r['goal_id']].append(float(r['time_to_goal_s']))
    return W
A, Bw = per_world(a), per_world(b, safe_only=True)
TTG_MAX = 1.5
out = []
hdr = ['world', '3d_ok', 'base_ok', '3d_unsafe', 'base_unsafe', '3d_recov', 'base_recov',
       'ttg_goals', '3d_ttg', 'base_ttg', 'ttg_ratio', 'unsafe_item', 'recov_item', 'ttg_item']
print(' | '.join(hdr))
for w in sorted(set(A) | set(Bw)):
    x, y = A[w], Bw[w]
    goals = [g for g in y['ttg'] if g in x['ttg']]
    missing = [g for g in y['ttg'] if g not in x['ttg']]
    t3 = sum(sum(x['ttg'][g]) / len(x['ttg'][g]) for g in goals)
    tb = sum(sum(y['ttg'][g]) / len(y['ttg'][g]) for g in goals)
    ratio = t3 / tb if tb else float('nan')
    zero_needed = w in ('slopes', 'crater_field')
    unsafe_ok = (x['unsafe'] == 0) if zero_needed else (x['unsafe'] < y['unsafe'] or x['unsafe'] == 0)
    ttg_ok = (not missing) and goals and ratio <= TTG_MAX
    row = [w, f"{x['ok']}/{x['n']}", f"{y['ok']}/{y['n']}", x['unsafe'], y['unsafe'], x['rec'], y['rec'],
           len(goals), f'{t3:.0f}', f'{tb:.0f}', f'{ratio:.2f}', 'pass' if unsafe_ok else 'FAIL',
           'pass' if x['rec'] <= y['rec'] else 'FAIL',
           'n/a (baseline never succeeds safely)' if not y['ttg'] else
           'pass' if ttg_ok else ('FAIL (3D misses ' + ' '.join(missing) + ')' if missing else 'FAIL')]
    out.append(row); print(' | '.join(map(str, row)))
if len(sys.argv) > 3:
    with open(sys.argv[3], 'w', newline='') as f:
        c = csv.writer(f); c.writerow(hdr); c.writerows(out)
