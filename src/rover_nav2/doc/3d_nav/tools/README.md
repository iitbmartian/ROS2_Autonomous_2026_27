# 3D navigation research tools (snapshot)

Copy of `~/.ros/rover_3d_nav/tools/`, the scripts behind every number in `PROGRESS.md` and the
results archive: unattended batch and experiment chains (each with a start deadline and a
`guard.sh` cap), `hang_watch.sh`, `phase2_eval.sh`, `phase3_gate.py`, `goal_audit.py`,
`shm_stale_ports.py`, figure scripts and remote (NUC) helpers. They hard-code paths under
`$HOME/.ros/rover_3d_nav` and are kept here for reproducibility, not as a package. Refreshed with
`rsync -a --exclude __pycache__ --exclude '*.pyc' ~/.ros/rover_3d_nav/tools/ <here>/`.
