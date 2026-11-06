run() {  # config tag out runs world [extra]
  for i in 1 2; do  # a second pass picks up runs lost to startup failures
    python3 - $B/$3/runs.csv <<'PY' 2>/dev/null
import csv, os, sys
p = sys.argv[1]
if os.path.exists(p):
    rows = list(csv.DictReader(open(p))); keep = [r for r in rows if r['status'] != 'startup_failed']
    if len(keep) != len(rows):
        w = csv.DictWriter(open(p, 'w', newline=''), fieldnames=rows[0].keys()); w.writeheader(); w.writerows(keep)
PY
    ros2 run rover_nav2 benchmark --worlds $5 --runs $4 --config $1 --tag $2 \
      --launch-args odom_source:=ground_truth --out $B/$3 "${@:6}" >> $E/evidence/night3.out 2>&1
  done
}
