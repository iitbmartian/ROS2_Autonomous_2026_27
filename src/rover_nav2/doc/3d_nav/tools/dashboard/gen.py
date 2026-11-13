#!/usr/bin/env python3
"""Builds the 3D nav status page (www/index.html) from the live chain: processes, runs.csv files,
batch.log, PROGRESS.md and plan.json (queue, next goals, deliverables, kept by hand). Run every
5 min from cron. Read-only, it never touches a run."""
import csv, datetime as dt, glob, html, json, os, re, statistics, subprocess, time
from pathlib import Path

H = Path.home()
E, B = H / '.ros/rover_3d_nav', H / '.ros/rover_benchmark'
D = E / 'dashboard'
LOG = H / 'rover_3d_nav_results/batch.log'
PROG = H / 'ROS2_Autonomous_2026_27/src/rover_nav2/doc/3d_nav/PROGRESS.md'
now = time.time()
esc = html.escape


def sh(cmd):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=20).stdout
    except Exception:
        return ''


def rows(d):
    f = B / d / 'runs.csv'
    try:
        return list(csv.DictReader(open(f)))
    except Exception:
        return []


def sec_per_row(d):
    """Mean wall seconds per run, from the spacing of the per-run logs of a finished run."""
    ts = sorted(os.path.getmtime(p) for p in glob.glob(str(B / d / 'logs/*.log')))
    if len(ts) < 3:
        return 180.0
    gaps = [b - a for a, b in zip(ts, ts[1:]) if b - a < 3 * 3600]
    return statistics.mean(gaps) if gaps else 180.0


def ago(s):
    s = int(s)
    return f'{s} s' if s < 120 else f'{s // 60} min' if s < 7200 else f'{s / 3600:.1f} h'


def clock(t):
    return dt.datetime.fromtimestamp(t).strftime('%a %d %b %H:%M')


plan = json.load(open(D / 'plan.json'))

# --- machine and chain state
try:
    cpu = int(open('/sys/class/hwmon/hwmon5/temp1_input').read()) // 1000
except Exception:
    cpu = -1
disk = sh("df -h ~ | awk 'END{print $5}'").strip()
chains = [l.split(None, 2) for l in sh(
    "ps -eo pid,etimes,args | awk '$4 ~ /exp_[a-z0-9_]*\\.sh$/ {print $1, $2, $4}'").splitlines()]
running_scripts = {c[2].split('/')[-1] for c in chains if len(c) == 3}
runner = sh("pgrep -af 'lib/rover_nav2/benchmark --worlds' | head -1").strip()
outs = sorted(glob.glob(str(E / 'evidence/*.out')), key=os.path.getmtime)
newest_out = outs[-1] if outs else None
out_age = now - os.path.getmtime(newest_out) if newest_out else 1e9
last_lines = [l for l in open(newest_out, errors='replace').read().splitlines()
              if re.match(r'^\d\d:\d\d', l)][-4:] if newest_out else []
logtxt = open(LOG, errors='replace').read().splitlines() if LOG.exists() else []

alerts = []
if not chains:
    alerts.append('No chain script is running: the machine is idle.')
elif out_age > 900:
    alerts.append(f'No runner output for {ago(out_age)}: the current run may be stuck.')
if cpu >= 95:
    alerts.append(f'CPU at {cpu} C.')
if disk and int(disk.rstrip('%') or 0) >= 92:
    alerts.append(f'Disk at {disk}.')

# --- queue with ETA
t_cursor = now
queue_html = []
for q in plan['queue']:
    name = q['script'].replace('.sh', '')
    done_logged = any(f'{name}: done' in l for l in logtxt)
    started = any(f'{name}: start' in l for l in logtxt)
    if q['script'] in running_scripts:
        state = 'running' if started else 'queued'
    else:
        state = 'done' if done_logged else 'not running'
    srows = []
    for s in q['stages']:
        r = rows(s['out'])
        n, planned = len(r), s['rows']
        spr = sec_per_row(s.get('ref', s['out']))
        left = 0 if state == 'done' else max(0, planned - n)
        eta = ''
        if left and state in ('running', 'queued'):
            t_cursor += left * spr
            eta = clock(t_cursor)
        ok = sum(1 for x in r if x.get('success') == '1')
        srows.append(f'<tr><td><code>{esc(s["out"])}</code></td><td>{n}/{planned}</td>'
                     f'<td>{ok}</td><td>{ago(spr)}</td><td>{eta or ("finished" if not left else "")}</td></tr>')
    if not q['stages'] and state in ('running', 'queued') and q.get('minutes'):
        t_cursor += q['minutes'] * 60
    queue_html.append(
        f'<div class="card q {state.replace(" ", "")}"><div class="qh"><span class="pill {state.replace(" ", "")}">{state}</span>'
        f'<b>{esc(name)}</b><span class="muted">deadline {esc(q.get("deadline", ""))}</span></div>'
        f'<p>{esc(q["label"])}</p><table><tr><th>stage</th><th>rows</th><th>ok</th><th>per run</th><th>ETA</th></tr>'
        + ''.join(srows) + '</table></div>')
eta_all = clock(t_cursor) if t_cursor > now else 'nothing queued'

# --- results
def summary(d):
    r = rows(d)
    if not r:
        return None
    w = {}
    for x in r:
        k = x['world'].replace('.sdf', '')
        a = w.setdefault(k, [0, 0, 0, 0, 0])
        a[0] += 1
        a[1] += x.get('success') == '1'
        # A correct refusal (aborted with success 1, lane35) counts as reached.
        a[2] += x.get('success') == '1' or (x.get('reached') == '1' if x.get('reached') else (
            x.get('status') not in ('startup_failed',) and float(x.get('final_error_m') or 99) <= 0.5))
        a[3] += int(x.get('unsafe_entries') or 0)
        a[4] += int(x.get('recoveries') or 0)
    return w


LABEL = {'3d_gt_plain_d250': '3D (default)', '3d_gt_lethalfree_d250': '3D lethal-free check',
         '2d_base_d250': '2D baseline', '3d_gt_g01_d250': '3D, 0.1 m global map', '3d_gt_fs_d250': '3D + fine step (fix)', '3d_gt_fs_rep_d250': '3D + fine step, repeat'}


def nice(d):
    k = d[11:]
    return LABEL.get(k, k.replace('_', ' '))


# One benchmark row per run: goals reached, unsafe entries, recoveries.
sums = {d: summary(d) for d in plan['compare']}
bench = []
for d, s in sums.items():
    if not s:
        continue
    a = [sum(v[i] for v in s.values()) for i in range(5)]
    per = ' '.join(f'{w} {v[1]}/{v[0]}' for w, v in sorted(s.items()))
    bench.append(f'<tr><td><b>{esc(nice(d))}</b><div class="muted">{esc(d[:10])}</div></td>'
                 f'<td class="big">{a[1]}/{a[0]}</td><td>{a[3]}</td><td>{a[4]}</td>'
                 f'<td class="muted small">{esc(per)}</td></tr>')


def gate_line(fn):
    f = E / 'evidence' / fn
    if not f.exists():
        return ''
    lines = [l for l in open(f).read().splitlines() if '|' in l][1:]
    fails = [l.split('|')[0].strip() for l in lines if 'FAIL' in l]
    name = fn.replace('phase3_gate_', '').replace('.txt', '').replace('_', ' ')
    v = (f'<span class="pill fail">{len(fails)} fail</span>' if fails else '<span class="pill done">pass</span>')
    return f'<li>{v}{esc(name)} <span class="muted">{esc(", ".join(fails))}</span></li>'


status = 'Idle' if not chains else 'Stalled?' if out_age > 900 else 'Running'
scls = 'fail' if status != 'Running' else 'running'
now_item = next((q for q in plan['queue'] if q['script'] in running_scripts), None)
now_txt = esc(now_item['label']) if now_item else 'nothing running'
days_left = (dt.datetime.strptime(plan['deadline'], '%Y-%m-%d %H:%M').timestamp() - now) / 86400


def qrow(q):
    name = q['script'].replace('.sh', '')
    run = q['script'] in running_scripts
    started = any(f'{name}: start' in l for l in logtxt)
    done = any(f'{name}: done' in l for l in logtxt)
    st = 'running' if run and (started or not q['stages']) else 'queued' if run else 'done' if done else 'stopped'
    n = sum(len(rows(x['out'])) for x in q['stages'])
    tot = sum(x['rows'] for x in q['stages'])
    prog = f'{n}/{tot} runs' if tot else ''
    return (f'<tr><td><span class="pill {st}">{st}</span></td><td>{esc(q["label"])}</td>'
            f'<td>{prog}</td></tr>')


page = f'''<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><meta http-equiv="refresh" content="60">
<title>Rover 3D Nav Status</title><style>
:root{{--bg:#f6f7f9;--card:#fff;--ink:#1c2430;--mut:#6b7686;--line:#e2e6ec;--ok:#1f7a4d;--okb:#e3f4ea;
--bad:#b3261e;--badb:#fbe7e6;--run:#1d5fbf;--runb:#e4eefc;--wait:#8a6d00;--waitb:#fbf3d6}}
@media (prefers-color-scheme:dark){{:root{{--bg:#11151b;--card:#1a2029;--ink:#e6ebf2;--mut:#93a0b2;--line:#2b3340;
--ok:#6fd39c;--okb:#173326;--bad:#ff8a80;--badb:#3a1a18;--run:#8ab4ff;--runb:#18284a;--wait:#e8c75a;--waitb:#352c10}}}}
*{{box-sizing:border-box}}body{{margin:0;background:var(--bg);color:var(--ink);font:15px/1.5 system-ui,sans-serif}}
main{{max-width:820px;margin:0 auto;padding:16px}}h1{{font-size:21px;margin:4px 0}}h2{{font-size:16px;margin:24px 0 8px}}
.muted{{color:var(--mut);font-size:13px}}.small{{font-size:12px;white-space:normal}}
.card{{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px 14px;overflow-x:auto}}
.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:10px;margin-top:12px}}
.stat{{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:10px 12px}}
.stat b{{display:block;font-size:19px}}.stat span{{color:var(--mut);font-size:12px}}
table{{border-collapse:collapse;width:100%;font-size:14px}}th,td{{text-align:left;padding:7px 8px;border-bottom:1px solid var(--line);vertical-align:top}}
th{{color:var(--mut);font-weight:600;font-size:12px}}td.big{{font-size:18px;font-weight:700}}
.pill{{display:inline-block;border-radius:99px;padding:1px 9px;font-size:12px;font-weight:700;margin-right:8px}}
.pill.running{{background:var(--runb);color:var(--run)}}.pill.done{{background:var(--okb);color:var(--ok)}}
.pill.queued{{background:var(--waitb);color:var(--wait)}}.pill.stopped,.pill.fail{{background:var(--badb);color:var(--bad)}}
.alert{{background:var(--badb);color:var(--bad);border-radius:10px;padding:10px 14px;font-weight:700;margin:10px 0}}
ul{{margin:0;padding-left:20px}}li{{margin:4px 0}}</style></head><body><main>
<h1>Rover 3D Nav</h1><div class="muted">Updated {clock(now)}</div>
{''.join(f'<div class="alert">{esc(a)}</div>' for a in alerts)}
<div class="grid">
<div class="stat"><span>status</span><b><span class="pill {scls}">{status}</span></b></div>
<div class="stat"><span>queue done by</span><b>{esc(eta_all)}</b></div>
<div class="stat"><span>CPU / disk</span><b>{cpu} C / {esc(disk)}</b></div>
<div class="stat"><span>to real-rover tests</span><b>{days_left:.1f} days</b></div></div>
<h2>Now</h2><div class="card"><b>{now_txt}</b><div class="muted" style="margin-top:4px">{esc(plan["headline"])}</div></div>
<h2>Current issues</h2><div class="card"><ul>{''.join(f"<li>{esc(x)}</li>" for x in plan.get("issues", [])) or '<li class="muted">none</li>'}</ul></div>
<h2>Plan</h2><div class="card"><ul>{''.join(f"<li>{esc(x)}</li>" for x in plan.get("plans", []))}</ul></div>
<h2>Queue</h2><div class="card"><table>{''.join(qrow(q) for q in plan["queue"]) or '<tr><td class="muted">empty</td></tr>'}</table></div>
<h2>Milestones</h2><div class="card"><table>{''.join(f"<tr><td>{esc(x['what'])}</td><td class='muted'>{esc(x['when'])}</td></tr>" for x in plan["deliverables"])}</table></div>
<h2>Benchmarks</h2><div class="card"><table><tr><th>stack</th><th>goals reached</th><th>unsafe</th><th>recoveries</th><th>per world</th></tr>{''.join(bench) or '<tr><td class="muted">no runs yet</td></tr>'}</table>
<div class="muted" style="margin-top:6px">Phase 3 gate</div><ul>{''.join(gate_line(g) for g in plan["gates"])}</ul></div>
</main></body></html>'''
tmp = D / 'www/index.html.new'
tmp.write_text(page)
os.replace(tmp, D / 'www/index.html')
