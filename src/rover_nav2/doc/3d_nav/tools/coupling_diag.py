#!/usr/bin/env python3
"""Per coupling and stack: time, path, body speed and pitch vibration from the ground-truth poses.

    python3 coupling_diag.py <out dir with runs.csv and poses/>

Pitch jitter is the pitch minus its 0.5 s moving mean, rms. Reverse share is the fraction of time
the body moves backward (20-sample mean below -0.02 m/s).
"""
import csv
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

out = Path(sys.argv[1])
rows = defaultdict(list)
for r in csv.DictReader(open(out / 'runs.csv')):
    rows[r['config']].append(r)
print('| coupling, stack | ok | time s | path m | v p50 m/s | reverse | pitch jitter mdeg | pitch rate p95 deg/s | peak Hz |')
print('|---|---|---|---|---|---|---|---|---|')
for tag, rs in rows.items():
    st = defaultdict(list)
    for r in rs:
        f = out / 'poses' / f"{r['run_id']}.csv"
        if not f.exists():
            continue
        a = np.loadtxt(f, delimiter=',', skiprows=1)
        if len(a) < 200:
            continue
        t = np.arange(a[0, 0], a[-1, 0], 0.01)
        p = np.interp(t, a[:, 0], np.unwrap(a[:, 5]))
        j = p - np.convolve(p, np.ones(50) / 50, 'same')
        x, y = np.interp(t, a[:, 0], a[:, 1]), np.interp(t, a[:, 0], a[:, 2])
        yaw = np.interp(t, a[:, 0], np.unwrap(a[:, 6]))
        vx = np.convolve(np.gradient(x, 0.01) * np.cos(yaw) + np.gradient(y, 0.01) * np.sin(yaw),
                         np.ones(20) / 20, 'same')
        f_ = np.fft.rfftfreq(len(j), 0.01)
        s = np.abs(np.fft.rfft(j * np.hanning(len(j))))
        m = f_ > 0.5
        st['ok'].append(r['status'] == 'succeeded')
        st['t'].append(float(r['time_to_goal_s']))
        st['path'].append(float(r['path_length_m'] or 0))
        st['v'].append(np.median(vx))
        st['rev'].append((vx < -0.02).mean())
        st['jit'].append(np.degrees(j[50:-50].std()) * 1000)
        st['rate'].append(np.percentile(np.abs(np.degrees(np.gradient(p, 0.01))), 95))
        st['hz'].append(f_[m][np.argmax(s[m])])
    if not st:
        continue
    med = {k: np.median(v) for k, v in st.items() if k != 'ok'}
    print(f"| {tag} | {sum(st['ok'])}/{len(st['ok'])} | {med['t']:.0f} | {med['path']:.1f} | "
          f"{med['v']:.2f} | {med['rev']:.2f} | {med['jit']:.0f} | {med['rate']:.1f} | {med['hz']:.1f} |")
