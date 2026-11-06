# 3D navigation: remaining work by phase

Snapshot of everything still to do, written 2026-11-07 03:30. `PHASES.md` holds the gates,
`PROGRESS.md` the evidence. Dates are estimates for this laptop alone, a cloud benchmark farm
would cut 3 to 4 days. Items marked **(Ram)** need hardware, measured data or a decision from Ram,
**(other owner)** go on a `ram/<package>-<description>` branch.

## Phase 2: terrain mapping (gate open, 2 items)

Done: elevation RMSE within 5 cm on all 7 worlds (crater_field 4.15 cm on ground truth).

1. Crater check. Goals that drive toward crater rims on `crater_field`, or the 3D mapper riding
   along a blind 2D run, so the footprint actually reaches a crater wider than the track. Show it
   is flagged `negative_obstacle` first. The 2026-11-07 run avoided all 24, which tests nothing.
2. **(Ram)** Map update latency and CPU on a NUC-class machine. Lab NUC, or the Intel cloud
   Meteor Lake or Arrow Lake machine.

## Phase 3: traversability with Ackermann (gate run tonight)

Done: pose evaluation, layer v2, 3D overlay. Head-to-head 3D 40 of 63 vs `2d_bounded` 37 of 63,
0 unsafe. Governor v_nominal 0.5 chosen (12 of 14, about half the old time to goal).

1. Confirming run, all worlds, 3 runs per goal (`tools/exp_confirm.sh`, queued), then the gate
   table `analysis/phase3_gate_confirm.csv`.
2. Time to goal within 10 percent of baseline. At 0.5 m/s it is 1.0 to 2.2x. Levers: final
   approach (the near-miss timeouts 0.2 to 0.5 m out), S-curves Smac adds to meet the goal yaw,
   cruise share and acceleration limits. If 10 percent stays out of reach on rough worlds, propose
   a per-world gate change to Ram with the numbers.
3. Recoveries no worse than baseline: marsyard (7 vs 0) and urc_mix (22 vs 0) fail. Find which
   behaviour fires and why.
4. slopes `lane35_top` (aborts 2.9 m out) and `sidehill_uphill` (times out 0.2 m short). Likely
   Ackermann limits, so they are the Phase 4 target, but check the abort cause first.
5. **(Ram)** ledge and bars. Both fail on the placeholder `step_max` 0.10 m, which is correct for
   that value. Re-run once the measured step capability and CG height arrive.
6. Rebuild the 2D stack and rerun its tests before the phase's final commit.

## Phase 4: swerve (first run tonight)

Done: swerve mode, TerrainCritic, swerve overlay. Plugin name fixed 2026-11-07 (`0108531`).

1. Evaluate `exp_swerve` against Ackermann at the same governor on marsyard, slopes, rock_field,
   urc_mix.
2. Tune the Omni MPPI overlay (critic weights, `vy_max`, redirect and uphill weights) to a good
   sim baseline, without deep dives on tuning-only issues.
3. Velocity smoother and collision monitor with `vy`, checked in a run.
4. Gate: time to goal below Phase 3 on slopes, rock_field and urc_mix, unsafe and recoveries no
   worse, a sidehill crossing Phase 3 refuses done facing uphill.
5. **(other owner: `rover_gazebo`)** Hand the swerve kinematics branch to its owner.

## Phase 5: speed and safety

Done: governor (with `u_free`), safety monitor, collision monitor wiring.

1. Full benchmark with zero unsafe entries on every world.
2. Time to goal no worse than Phase 4. Final governor values per drive mode.
3. **(Ram)** Slip layer, only with wheel speed feedback. Ask before starting.

## Between phases: real odometry

Everything is tuned on ground-truth odometry so far.

1. Rerun the chosen config on EKF odometry, measure how much success drops from drift.
2. Report the yaw drift during recovery spins (27 deg on crater_field) to `rover_ekf`'s owner, try
   the RTAB-Map ICP odometry option.

## Phase 6: global terrain memory

1. Global terrain map in `map`, handling RTAB-Map loop closures (re-stitch submaps or coarse
   resolution, propose which with evidence).
2. Long multi-waypoint goal sets, 100 to 500 m, in `urc_mix` and the Gale Crater patch.
3. Gate: runs of at least 100 m with zero unsafe entries, global map alignment error after loop
   closure reported.
4. Memory and CPU of the global map over a 500 m run.

## Phase 7: hardware (Ram)

1. Orin and NUC bring-up, clock sync, depth cloud bandwidth.
2. Real L2 mount calibration.
3. `rover_physical.yaml` filled with measured values: mass, CG height and offsets, ground
   clearance, rocker range, step capability, stability margin, top speed, steering limit and rate.
4. Bags, offline tuning, field slope tests. Gate agreed with Ram when Phase 6 closes.

## Infrastructure

1. **(Ram)** Intel cloud farm: Docker on Ubuntu 26.04, the rover-dev image, `/dev/dri`
   rendering, RTF per lane, parallel lanes with `tools/farm_run.sh`.
2. Build and test inside the `docker/` container before each phase's final commit.
3. Keep `tools/` snapshot, `RESULTS.md`, figures and dated backups current.
4. **(Ram)** Push and PRs, other-owner branches sent to their owners.

## Publication

1. Final benchmark tables and figures per phase, 3D against `2d_bounded`.
2. Recorded sim runs (RViz and Gazebo captures) of the hard cases: rock_field, crater_field,
   sidehill.
3. Write-up: method (2.5D map, utilisation cost, heading-aware layer, TerrainCritic, governor),
   results, honest failures.

## On hold (Ram, 2026-11-06)

- Red no-go zones as a costmap input.
- Marker-based final approach.
- Anything IRC-specific (heading-free goals, competition configs). `--goal-yaw bearing`,
  `nav_extra_overlay` and `exp/nav2_3d_irc.yaml` stay parked.

## Estimated dates

| Milestone | Estimate |
|---|---|
| Presentable short-range 3D demo | Tue 10 Nov |
| Phase 3 gate | 10 to 11 Nov |
| Phase 4 swerve gate | about 13 Nov |
| Phase 5 | about 15 Nov |
| Phase 6 long routes | 21 to 25 Nov |
| Phase 7 | on hardware access |
