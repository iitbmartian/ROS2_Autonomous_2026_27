#!/bin/bash
# Stop every process carrying ROVER_SIM_RUN=<name> (or any ROVER_SIM_RUN with no argument).
NAME=$1
python3 - "$NAME" <<'PY'
import sys, psutil
name = sys.argv[1]
procs = []
for p in psutil.process_iter():
    try:
        m = p.environ().get('ROVER_SIM_RUN')
    except Exception:
        continue
    if m and (not name or m == name):
        procs.append(p)
for p in procs:
    try: p.send_signal(2)
    except Exception: pass
gone, alive = psutil.wait_procs(procs, timeout=20)
for p in alive:
    try: p.kill()
    except Exception: pass
print(f'stopped {len(procs)} processes ({len(alive)} needed SIGKILL)')
PY
# Shared memory left by killed nodes (zombie segments only, live ones are untouched).
fastdds shm clean > /dev/null 2>&1 || /opt/ros/jazzy/bin/fastdds shm clean > /dev/null 2>&1
python3 $HOME/.ros/rover_3d_nav/tools/shm_stale_ports.py --rm > /dev/null 2>&1  # port mutexes of killed nodes, which shm clean keeps
