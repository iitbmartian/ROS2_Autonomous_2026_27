#!/usr/bin/env python3
"""Builds the 3D nav status page (www/index.html). It answers what Ram asks in chat: where the work
stands, what changed lately, what is running and how it is doing against its reference, what is
queued and when it ends, and what needs Ram. The narrative parts (summary, updates, phases, needs)
are kept in plan.json at each health check, the live parts come from batch.log and runs.csv.
Run every 5 min from cron. Read-only, it never touches a run."""
import csv, datetime as dt, glob, html, json, os, re, subprocess, time
from collections import defaultdict
from pathlib import Path

H = Path.home()
E, B = H / '.ros/rover_3d_nav', H / '.ros/rover_benchmark'
D = E / 'dashboard'
LOG = H / 'rover_3d_nav_results/batch.log'
now = time.time()
esc = html.escape


def sh(cmd):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=20).stdout
    except Exception:
        return ''


def rows(d):
    try:
        return list(csv.DictReader(open(B / d / 'runs.csv')))
    except Exception:
        return []


def newest_dir(tag):
    ds = sorted(glob.glob(str(B / f'*_{tag}')))
    return os.path.basename(ds[-1]) if ds else None


def ago(s):
    s = int(s)
    return f'{s} s' if s < 120 else f'{s // 60} min' if s < 7200 else f'{s / 3600:.1f} h'


def clock(t):
    return dt.datetime.fromtimestamp(t).strftime('%a %d %b %H:%M')


def world(r):
    return r['world'].replace('.sdf', '')


def ok(r):
    # Ground-truth success column, the one every reported number uses.
    return r.get('success') == '1'


plan = json.load(open(D / 'plan.json'))
logtxt = open(LOG, errors='replace').read().splitlines() if LOG.exists() else []


def log_time(line):
    try:
        return dt.datetime.strptime(line[:16], '%Y-%m-%d %H:%M').timestamp()
    except ValueError:
        return None


def state_of(name):
    """running, done or queued, from the chain's own start and done lines in batch.log."""
    st, t0 = 'queued', None
    for l in logtxt:
        if f' {name}: start' in l:
            st, t0 = 'running', log_time(l)
        elif f' {name}: done' in l and st == 'running':
            st = 'done'
    return st, t0


# --- machine and chain state
try:
    cpu = int(open('/sys/class/hwmon/hwmon5/temp1_input').read()) // 1000
except Exception:
    cpu = -1
dfree = sh("df -BG ~ | awk 'END{print $4}'").strip().rstrip('G')
dfree = int(dfree) if dfree.isdigit() else -1
chains = sh("ps -eo args | grep -c '[e]xp_[a-z0-9_]*\\.sh'").strip()
chains = int(chains) if chains.isdigit() else 0
outs = sorted(glob.glob(str(E / 'evidence/*.out')), key=os.path.getmtime)
out_age = now - os.path.getmtime(outs[-1]) if outs else 1e9
last_lines = [l for l in open(outs[-1], errors='replace').read().splitlines()
              if re.match(r'^\d\d:\d\d:\d\d \[', l)][-3:] if outs else []

alerts = []
if not chains:
    alerts.append('No experiment is running: the machine is idle.')
elif out_age > 900:
    alerts.append(f'No runner output for {ago(out_age)}: the current run may be stuck.')
if cpu >= 98:
    alerts.append(f'CPU at {cpu} C.')
if 0 <= dfree < 10:
    alerts.append(f'Disk: only {dfree} GB free.')

# --- queue with ETA
t_cursor, queue_rows, running = now, [], None
for q in plan['queue']:
    st, t0 = state_of(q['log'])
    mins = q.get('minutes', 120)
    if st == 'running':
        end = max(now + 600, (t0 or now) + mins * 60)
        t_cursor = end
        running = running or q
    elif st == 'queued':
        t_cursor += mins * 60
        end = t_cursor
    else:
        end = None
    eta = clock(end) if end else 'finished'
    queue_rows.append(f'<tr><td><span class="pill {st}">{st}</span></td><td><b>{esc(q["log"])}</b>'
                      f'<div class="muted">{esc(q["label"])}</div></td><td class="nowrap">{eta}</td></tr>')
queued_after = sum(1 for q in plan['queue'] if state_of(q['log'])[0] == 'queued')


# --- live table: current run per goal against its reference
def per_goal(dirs):
    g = defaultdict(lambda: [0, 0])
    for d in dirs:
        d = newest_dir(d[4:]) if d.startswith('tag:') else d
        for r in rows(d) if d else []:
            k = (world(r), r['goal_id'])
            g[k][0] += ok(r)
            g[k][1] += 1
    return g


live_html = '<p class="muted">nothing running</p>'
live_head = 'nothing running'
if running:
    d = newest_dir(running['tag'])
    cur = rows(d) if d else []
    refs = running.get('ref', [])
    ref = per_goal(refs)
    g = defaultdict(list)
    for r in cur:
        g[(world(r), r['goal_id'])].append(r)
    trs = []
    for k in sorted(g):
        rs = g[k]
        n_ok = sum(ok(r) for r in rs)
        fails = [f"{r['status']} {float(r['final_error_m'] or 0):.1f} m" for r in rs if not ok(r)]
        rk = ref.get(k)
        rtxt = f'{rk[0]}/{rk[1]}' if rk else ''
        cls = '' if not rk else 'better' if n_ok / len(rs) > rk[0] / rk[1] else 'worse' if n_ok / len(rs) < rk[0] / rk[1] else ''
        trs.append(f'<tr class="{cls}"><td>{esc(k[0])}</td><td>{esc(k[1])}</td><td class="big">{n_ok}/{len(rs)}</td>'
                   f'<td>{rtxt}</td><td class="muted small">{esc(", ".join(fails))}</td></tr>')
    tot = sum(ok(r) for r in cur)
    ref_same = sum(ref[k][0] for k in g if k in ref), sum(ref[k][1] for k in g if k in ref)
    live_head = (f'{esc(running["label"])}<br><b>{tot}/{len(cur)}</b> so far'
                 + (f', reference on the same goals {ref_same[0]}/{ref_same[1]}' if ref_same[1] else ''))
    live_html = (f'<table><tr><th>world</th><th>goal</th><th>now</th><th>ref</th><th>failures</th></tr>{"".join(trs)}</table>'
                 f'<div class="muted" style="margin-top:6px">Reference: {esc(", ".join(refs) or "none")}. '
                 f'Last output {ago(out_age)} ago.</div>'
                 + ''.join(f'<div class="mono">{esc(l[:140])}</div>' for l in last_lines))

# --- headline comparison by world
WORLDS = ['slopes', 'crater_field', 'marsyard', 'rock_field', 'urc_mix', 'bars']
cmp_rows = []
for s in plan.get('stacks', []):
    w = defaultdict(lambda: [0, 0])
    unsafe = 0
    # A dir entry is a result dir, "tag:<tag>" for the newest dir of a tag, or
    # {"dir": ..., "worlds": [...]} to take only some worlds from it.
    for e in s['dirs']:
        e = e if isinstance(e, dict) else {'dir': e}
        d = newest_dir(e['dir'][4:]) if e['dir'].startswith('tag:') else e['dir']
        for r in rows(d) if d else []:
            if 'worlds' in e and world(r) not in e['worlds']:
                continue
            w[world(r)][0] += ok(r)
            w[world(r)][1] += 1
            unsafe += int(r.get('unsafe_entries') or 0)
    tot = [sum(v[0] for v in w.values()), sum(v[1] for v in w.values())]
    cells = ''.join(f'<td>{w[x][0]}/{w[x][1]}</td>' if w[x][1] else '<td class="muted">-</td>' for x in WORLDS)
    pct = f' <span class="muted">{100 * tot[0] / tot[1]:.0f}%</span>' if tot[1] else ''
    cmp_rows.append(f'<tr><td><b>{esc(s["label"])}</b><div class="muted">{esc(s.get("note", ""))}</div></td>'
                    f'<td class="big">{tot[0]}/{tot[1]}{pct}</td>{cells}<td>{unsafe}</td></tr>')

# --- recent evaluations straight from batch.log, as a fallback for the hand-kept updates
evals = [l for l in logtxt if ' evaluated' in l or ' health:' in l][-6:][::-1]

status = 'Idle' if not chains else 'Stalled?' if out_age > 900 else 'Running'
days_left = (dt.datetime.strptime(plan['deadline'], '%Y-%m-%d %H:%M').timestamp() - now) / 86400
li = lambda xs: ''.join(f'<li>{esc(x)}</li>' for x in xs) or '<li class="muted">none</li>'

page = f'''<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><meta http-equiv="refresh" content="120">
<title>Rover 3D Nav</title><style>
:root{{--bg:#f6f7f9;--card:#fff;--ink:#1c2430;--mut:#6b7686;--line:#e2e6ec;--ok:#1f7a4d;--okb:#e3f4ea;
--bad:#b3261e;--badb:#fbe7e6;--run:#1d5fbf;--runb:#e4eefc;--wait:#8a6d00;--waitb:#fbf3d6}}
@media (prefers-color-scheme:dark){{:root{{--bg:#11151b;--card:#1a2029;--ink:#e6ebf2;--mut:#93a0b2;--line:#2b3340;
--ok:#6fd39c;--okb:#173326;--bad:#ff8a80;--badb:#3a1a18;--run:#8ab4ff;--runb:#18284a;--wait:#e8c75a;--waitb:#352c10}}}}
*{{box-sizing:border-box}}body{{margin:0;background:var(--bg);color:var(--ink);font:15px/1.55 system-ui,sans-serif}}
main{{max-width:860px;margin:0 auto;padding:16px}}h1{{font-size:21px;margin:4px 0}}h2{{font-size:16px;margin:26px 0 8px}}
.muted{{color:var(--mut);font-size:13px}}.small{{font-size:12px}}.nowrap{{white-space:nowrap}}
.mono{{font:12px ui-monospace,monospace;color:var(--mut);overflow-wrap:anywhere}}
.card{{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px 14px;overflow-x:auto}}
.lead{{font-size:16px}}.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px;margin-top:12px}}
.stat{{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:10px 12px}}
.stat b{{display:block;font-size:18px}}.stat span{{color:var(--mut);font-size:12px}}
table{{border-collapse:collapse;width:100%;font-size:14px}}th,td{{text-align:left;padding:6px 8px;border-bottom:1px solid var(--line);vertical-align:top}}
th{{color:var(--mut);font-weight:600;font-size:12px}}td.big{{font-weight:700;white-space:nowrap}}
tr.better td.big{{color:var(--ok)}}tr.worse td.big{{color:var(--bad)}}
.pill{{display:inline-block;border-radius:99px;padding:1px 9px;font-size:12px;font-weight:700}}
.pill.running{{background:var(--runb);color:var(--run)}}.pill.done{{background:var(--okb);color:var(--ok)}}
.pill.queued{{background:var(--waitb);color:var(--wait)}}.pill.open{{background:var(--badb);color:var(--bad)}}
.alert{{background:var(--badb);color:var(--bad);border-radius:10px;padding:10px 14px;font-weight:700;margin:10px 0}}
ul{{margin:0;padding-left:20px}}li{{margin:5px 0}}.upd b{{white-space:nowrap}}</style></head><body><main>
<h1>Rover 3D Nav</h1><div class="muted">Updated {clock(now)}, refreshes every 2 min</div>
{''.join(f'<div class="alert">{esc(a)}</div>' for a in alerts)}
<div class="grid">
<div class="stat"><span>chain</span><b>{status}</b></div>
<div class="stat"><span>queued behind current</span><b>{queued_after}</b></div>
<div class="stat"><span>queue ends</span><b>{clock(t_cursor) if t_cursor > now else '-'}</b></div>
<div class="stat"><span>CPU / disk free</span><b>{cpu} C / {dfree} GB</b></div>
<div class="stat"><span>to real-rover tests</span><b>{days_left:.1f} days</b></div></div>

<h2>Where we are</h2><div class="card"><p class="lead" style="margin-top:0">{esc(plan["headline"])}</p><ul>{li(plan.get("summary", []))}</ul></div>

<h2>Latest findings</h2><div class="card"><ul>{''.join(f'<li class="upd"><b>{esc(u["when"])}</b> {esc(u["text"])}</li>' for u in plan.get("updates", [])[:8])}</ul></div>

<h2>Running now</h2><div class="card"><p style="margin-top:0">{live_head}</p>{live_html}</div>

<h2>Needs you</h2><div class="card"><ul>{li(plan.get("needs_you", []))}</ul></div>

<h2>Results by world</h2><div class="card"><table><tr><th>stack</th><th>total</th>{''.join(f'<th>{w}</th>' for w in WORLDS)}<th>unsafe</th></tr>{''.join(cmp_rows)}</table>
<div class="muted" style="margin-top:6px">Ground-truth success, 3 runs per goal. Run-to-run noise is about ±2 successes per 21 to 24 runs.</div></div>

<h2>Queue</h2><div class="card"><table>{''.join(queue_rows) or '<tr><td class="muted">empty</td></tr>'}</table></div>

<h2>Phases</h2><div class="card"><table>{''.join(f'<tr><td><b>{esc(p["phase"])}</b></td><td><span class="pill {esc(p["state"])}">{esc(p["state"])}</span></td><td>{esc(p["note"])}</td></tr>' for p in plan.get("phases", []))}</table></div>

<h2>Plan to 22 Nov</h2><div class="card"><ul>{li(plan.get("plans", []))}</ul></div>

<h2>Chain log</h2><div class="card">{''.join(f'<div class="mono">{esc(l[:220])}</div>' for l in evals)}</div>
</main></body></html>'''
tmp = D / 'www/index.html.new'
tmp.write_text(page)
os.replace(tmp, D / 'www/index.html')
