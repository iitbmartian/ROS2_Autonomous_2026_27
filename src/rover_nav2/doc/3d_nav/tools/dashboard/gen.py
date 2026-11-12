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


sums = {d: summary(d) for d in plan['compare']}
worlds = sorted({k for s in sums.values() if s for k in s})
res = ['<table><tr><th>world</th>' + ''.join(
    f'<th>{esc(d[11:])}<br><span class="muted">ok / reached / unsafe / rec</span></th>' for d in sums) + '</tr>']
for wd in worlds + ['total']:
    cells = []
    for d, s in sums.items():
        if not s:
            cells.append('<td class="muted">no rows yet</td>')
            continue
        a = [sum(v[i] for v in s.values()) for i in range(5)] if wd == 'total' else s.get(wd)
        cells.append(f'<td>{a[1]}/{a[0]} / {a[2]} / {a[3]} / {a[4]}</td>' if a else '<td class="muted">-</td>')
    res.append(f'<tr class="{"tot" if wd == "total" else ""}"><td>{wd}</td>{"".join(cells)}</tr>')
res.append('</table>')


def gate_table(fn):
    f = E / 'evidence' / fn
    if not f.exists():
        return f'<p class="muted"><code>{esc(fn)}</code>: not produced yet</p>'
    lines = [l for l in open(f).read().splitlines() if '|' in l]
    if not lines:
        return f'<pre>{esc(open(f).read())}</pre>'
    keep = [0, 1, 2, 5, 6, 10, 11, 12, 13]
    out = []
    for i, l in enumerate(lines):
        c = [x.strip() for x in l.split('|')]
        c = [c[k] for k in keep if k < len(c)]
        tag = 'th' if i == 0 else 'td'
        out.append('<tr>' + ''.join(
            f'<{tag} class="{"fail" if "FAIL" in x else "pass" if x == "pass" else ""}">{esc(x)}</{tag}>' for x in c) + '</tr>')
    nfail = sum(l.count('FAIL') for l in lines[1:])
    verdict = '<span class="pill fail">FAIL ' + str(nfail) + '</span>' if nfail else '<span class="pill done">PASS</span>'
    return (f'<h3>{verdict} <code>{esc(fn)}</code> <span class="muted">{clock(os.path.getmtime(f))}</span></h3>'
            f'<table>{"".join(out)}</table>')


# --- progress log, newest entries
prog = open(PROG, errors='replace').read().splitlines() if PROG.exists() else []
last_h2 = max((i for i, l in enumerate(prog) if l.startswith('## ')), default=0)
prog_tail = '\n'.join(prog[last_h2:])[-9000:]

days_left = (dt.datetime.strptime(plan['deadline'], '%Y-%m-%d %H:%M').timestamp() - now) / 86400
status = 'IDLE' if not chains else 'STALLED?' if out_age > 900 else 'RUNNING'

page = f'''<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><meta http-equiv="refresh" content="60">
<title>Rover 3D Nav Status</title><style>
:root{{--bg:#f6f7f9;--card:#fff;--ink:#1c2430;--mut:#6b7686;--line:#e2e6ec;--ok:#1f7a4d;--okb:#e3f4ea;
--bad:#b3261e;--badb:#fbe7e6;--run:#1d5fbf;--runb:#e4eefc;--wait:#8a6d00;--waitb:#fbf3d6}}
@media (prefers-color-scheme:dark){{:root{{--bg:#11151b;--card:#1a2029;--ink:#e6ebf2;--mut:#93a0b2;--line:#2b3340;
--ok:#6fd39c;--okb:#173326;--bad:#ff8a80;--badb:#3a1a18;--run:#8ab4ff;--runb:#18284a;--wait:#e8c75a;--waitb:#352c10}}}}
*{{box-sizing:border-box}}body{{margin:0;background:var(--bg);color:var(--ink);font:15px/1.45 system-ui,sans-serif}}
main{{max-width:1100px;margin:0 auto;padding:16px}}h1{{font-size:22px;margin:4px 0}}h2{{font-size:17px;margin:22px 0 8px}}
h3{{font-size:15px;margin:12px 0 6px}}.muted{{color:var(--mut);font-weight:normal;font-size:13px}}
.card{{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px 14px;margin:10px 0;overflow-x:auto}}
.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px}}
.stat{{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:10px 12px}}
.stat b{{display:block;font-size:20px}}.stat span{{color:var(--mut);font-size:12px}}
table{{border-collapse:collapse;width:100%;font-size:13px}}th,td{{text-align:left;padding:5px 8px;border-bottom:1px solid var(--line);white-space:nowrap}}
th{{color:var(--mut);font-weight:600}}tr.tot td{{font-weight:700}}td.fail,th.fail{{color:var(--bad);font-weight:700}}td.pass{{color:var(--ok)}}
.pill{{display:inline-block;border-radius:99px;padding:1px 9px;font-size:12px;font-weight:700;margin-right:8px}}
.pill.running{{background:var(--runb);color:var(--run)}}.pill.done{{background:var(--okb);color:var(--ok)}}
.pill.queued{{background:var(--waitb);color:var(--wait)}}.pill.notrunning,.pill.fail{{background:var(--badb);color:var(--bad)}}
.qh{{display:flex;gap:8px;align-items:center;flex-wrap:wrap}}.q p{{margin:6px 0}}
.alert{{background:var(--badb);color:var(--bad);border-radius:10px;padding:10px 14px;font-weight:700;margin:10px 0}}
pre{{white-space:pre-wrap;word-break:break-word;font-size:12.5px;margin:0}}ul{{margin:6px 0;padding-left:20px}}
code{{font-size:12.5px}}</style></head><body><main>
<h1>Rover 3D Nav Status</h1>
<div class="muted">Built {clock(now)} from the live chain, rebuilt every 5 min, page reloads every minute. Plan updated {clock(os.path.getmtime(D / "plan.json"))}.</div>
{''.join(f'<div class="alert">{esc(a)}</div>' for a in alerts)}
<div class="grid" style="margin-top:12px">
<div class="stat"><span>chain</span><b>{status}</b></div>
<div class="stat"><span>all queued work done by</span><b>{esc(eta_all)}</b></div>
<div class="stat"><span>last runner output</span><b>{ago(out_age)} ago</b></div>
<div class="stat"><span>CPU / disk</span><b>{cpu} C / {esc(disk)}</b></div>
<div class="stat"><span>to 22 Nov real-rover tests</span><b>{days_left:.1f} days</b></div></div>
<div class="card"><b>Now:</b> {esc(plan["headline"])}<pre class="muted" style="margin-top:6px">{esc(chr(10).join(last_lines) or "no runner output")}</pre></div>
<h2>Queue and estimated completion</h2>{''.join(queue_html)}
<p class="muted">ETA = rows left x mean time per run of a comparable finished run, stages in order.</p>
<h2>Next goals</h2><div class="card"><ul>{''.join(f"<li>{esc(g)}</li>" for g in plan["next_goals"])}</ul></div>
<h2>Deliverables</h2><div class="card"><table><tr><th>what</th><th>when</th></tr>{''.join(f"<tr><td>{esc(x['what'])}</td><td>{esc(x['when'])}</td></tr>" for x in plan["deliverables"])}</table></div>
<h2>Latest results</h2><div class="card">{''.join(res)}<p class="muted">ok = Nav2 succeeded within 0.5 m, reached = ended within 0.5 m (heading ignored), rec = recoveries.</p></div>
<h2>Phase 3 gate</h2><div class="card">{''.join(gate_table(g) for g in plan["gates"])}</div>
<h2>batch.log</h2><div class="card"><pre>{esc(chr(10).join(l[:300] for l in logtxt[-15:]))}</pre></div>
<h2>PROGRESS.md, newest day</h2><div class="card"><pre>{esc(prog_tail)}</pre></div>
</main></body></html>'''
tmp = D / 'www/index.html.new'
tmp.write_text(page)
os.replace(tmp, D / 'www/index.html')
