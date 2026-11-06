#!/bin/bash
# Unattended batch, detached from Claude: survives network drops and session ends. Each step is
# resumable (the benchmark skips runs already in runs.csv), so rerunning this script continues.
# Progress: ~/rover_3d_nav_results/batch.log. Usage: setsid nohup batch_overnight.sh &
E=$HOME/.ros/rover_3d_nav
LOG=$HOME/rover_3d_nav_results/batch.log
source $E/tools/simenv.sh
cd /home/ram/ROS2_Autonomous_2026_27
step() { echo "$(date '+%F %H:%M') $*" >> $LOG; }
space_ok() {
  local free=$(df --output=avail -BG ~ | tail -1 | tr -dc 0-9)
  if [ "$free" -lt 2 ]; then step "STOP: only ${free} GB free on the disk"; exit 1; fi
}
step "batch start, commit $(git rev-parse --short HEAD)"

# 1. Phase 2 gate: map RMSE per world against ground truth, crater check, one snapshot each.
if [ ! -f $E/evidence/phase2/.done_phase2_all ]; then
  space_ok; step "phase 2 evaluation (all worlds)"
  $E/tools/phase2_all.sh > $E/evidence/phase2/all.out 2>&1 && touch $E/evidence/phase2/.done_phase2_all
  step "phase 2 evaluation finished"
fi

# 2. 3D stack, one run per goal on every world: where it fails before the Phase 3 tuning.
space_ok; step "3D check, all worlds, 1 run per goal"
ros2 run rover_nav2 benchmark --worlds slopes crater_field rock_field urc_mix marsyard ledge bars \
  --runs 1 --config 3d --tag 3d --out ~/.ros/rover_benchmark/2026-11-03_3d_check \
  >> $E/evidence/phase2/3d_check.out 2>&1
step "3D check finished"

# 3. Phase 1 gate 3: 2D baseline on all seven worlds, 5 runs per goal (the long one).
space_ok; step "2D baseline, all worlds, 5 runs per goal"
ros2 run rover_nav2 benchmark --worlds marsyard ledge bars slopes crater_field rock_field urc_mix \
  --runs 5 --config 2d --tag 2d --out ~/.ros/rover_benchmark/2026-11-03_baseline_2d_all \
  >> $E/evidence/phase1/baseline_all.out 2>&1
step "2D baseline finished"

# Figures for whatever finished.
python3 $E/tools/fig_benchmark.py check_3d ~/.ros/rover_benchmark/2026-11-03_3d_check 3d=3d >> $LOG 2>&1
python3 $E/tools/fig_benchmark.py baseline_2d_all ~/.ros/rover_benchmark/2026-11-03_baseline_2d_all 2d=2d >> $LOG 2>&1
python3 $E/tools/fig_benchmark.py compare_2d_3d ~/.ros/rover_benchmark/2026-11-03_baseline_2d_all \
  ~/.ros/rover_benchmark/2026-11-03_3d_check 2d=2d 3d=3d >> $LOG 2>&1
step "batch done"
