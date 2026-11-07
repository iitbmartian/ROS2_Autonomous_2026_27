#!/usr/bin/env python3
"""Per phase: max |roll| and |pitch| (deg), z drop (cm) from the settled pose, and the range of
each rocker joint (deg), for each probe csv given."""
import csv, math, sys
from collections import defaultdict
for path in sys.argv[1:]:
    R = list(csv.DictReader(open(path)))
    rock = [k for k in R[0] if k.endswith('S_joint')]
    z0 = [float(r['z']) for r in R if r['phase'] == 'settle'][-1]
    ph = defaultdict(list)
    for r in R:
        ph[r['phase']].append(r)
    print(f'== {path}')
    for name, rs in ph.items():
        roll = max(abs(math.degrees(float(r['roll']))) for r in rs)
        pitch = max(abs(math.degrees(float(r['pitch']))) for r in rs)
        dz = (min(float(r['z']) for r in rs) - z0) * 100
        rk = ' '.join(f"{k[:3]} {math.degrees(max(float(r[k]) for r in rs) - min(float(r[k]) for r in rs)):4.1f}" for k in rock)
        print(f'  {name:10s} roll {roll:5.1f} pitch {pitch:5.1f} dz {dz:+5.1f} cm  rocker range: {rk}')
