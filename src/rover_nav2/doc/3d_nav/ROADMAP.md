# 3D navigation: remaining work by phase

Snapshot of everything still to do, written 2026-11-07 03:30. `PHASES.md` holds the gates,
`PROGRESS.md` the evidence. Dates are estimates for this laptop alone, a cloud benchmark farm
would cut 3 to 4 days. Items marked **(Ram)** need hardware, measured data or a decision from Ram,
**(other owner)** go on a `ram/<package>-<description>` branch.

## Versions (Ram, 2026-11-07)

Ram calls the stack as it stands on 2026-11-07 **v1**. The phases below map onto versions like this.

| Version | Scope | Phases | Target |
|---|---|---|---|
| **v1** | Terrain-aware Ackermann navigation: 2.5D map, utilisation cost, governor, 0 unsafe entries in 63 runs, wins on craters, rocks and slopes | 0 to 3 (gate open) | done 7 Nov |
| **v1.1** | Phase 3 gate closed against the fresh 2D baseline: far_south final-approach fix, governor setting chosen, marsyard and rock_field time | 3 | 8 to 9 Nov |
| **v2** | Fast, agile, heading-aware: swerve with roll and speed fixed, dynamic (turning and braking) tip margin, small-hole map filling, speed near 2D parity on easy ground, heading-aware terrain (Phase 4b: grade along travel direction, body yaw and travel direction chosen separately), backtrack and wiggle recovery, zero unsafe entries in both drive modes, crater check | 2 (crater check), 4, 4b, 5 | 18 to 20 Nov |
| **v3** | Long range, real odometry, hardware: global terrain memory over 100 to 500 m, EKF or LIO odometry with map uncertainty growing with drift, CVaR risk knob, paper baselines and ablations, Orin or NUC bring-up, measured `rover_physical.yaml`, field tests, slip layer if wheel speeds exist | 5.3, 6, 7, real odometry | late Nov onward |

The Intel cloud farm would pull v2 in by 3 to 4 days. Measured step capability, CG height and mass
unblock ledge and bars and make the tip limits real. On hold items stay out unless Ram adds them.

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

## Phase 4b: heading-aware terrain, travel direction and body yaw chosen separately (planned 2026-11-07)

Ram's idea: a car crosses a speed breaker at an angle and climbs a hill diagonally because it is smoother and easier. Survey in `~/rover_3d_nav_results/paper/related_work_survey.md` (local). Summary:
- Direction-dependent (anisotropic) slope cost is established: Rowe and Ross 1990 prove switchbacks are optimal when straight up is impermissible, and CAMIS (Sánchez-Ibáñez 2021) and Inotsume (RA-L 2020) are current rover work. We adopt it, we don't claim it.
- Perseverance's operators drove cross-slope and backwards on a 15 to 20 deg loose slope to cut slip (Maimone et al., AAS 2026), but by hand.
- We found no planner that picks **body yaw** (tip margin) and **travel direction** (grade, traction) separately for a 4-wheel-steered rover. That is the likely novel core, and swerve already allows it.
- Bumps: MPPI with a vertical-acceleration cost exists, but none chooses the crossing angle. A speed-dependent CG vertical-acceleration term covers Ram's speed-breaker case.

Today's gap: the costmap is isotropic (best heading per cell), so Smac and the Ackermann controller cannot prefer a diagonal. Only swerve's TerrainCritic sees the actual heading, and it has no grade or traction term.

Plan (starts once swerve v2 drives without tip aborts):
1. Read CAMIS (paper and `spaceuma/CAMIS_python`), Rowe and Ross, Inotsume RA-L 2020 in full. Settle the grade model before coding.
2. `u_grade` along the travel direction, from the cell's plane gradient: `max(0, grade_up) / tan(climb_slope_max)`, plus a descent term with `descent_slope_max`. These are two new `# PLACEHOLDER` values in `rover_physical.yaml`. gtests with hand-computed cases. A rollout's travel direction comes from its world-frame displacement, body yaw from its pose. Pose utilisation becomes `max(u_static(body yaw), u_grade(travel direction))`.
3. (Dropped 2026-11-07, Ram: no focus on speed breakers.) The bump jolt term and the bumps world are out.
4. TerrainCritic for Ackermann too, as an experiment overlay, since today it is swerve-only.
5. A `long_slope` world (a uniform 20 to 25 deg hillside, about 30 m, steeper than `climb_slope_max` straight up), on a `ram/rover_gazebo-heading-worlds` branch, unless the realistic worlds under discussion already cover it.
6. Experiment: Ackermann vs swerve with heading tied to travel vs swerve with heading free, on long_slope and the slopes sidehill. Metrics: success, time, peak vertical acceleration, peak roll and pitch, drive effort from joint torques, slip (commanded vs ground-truth velocity).
7. Only if switchbacks are needed beyond the 2.5 s MPPI horizon: a heading-aware global planner. Either a Nav2 planner plugin running A* over (x, y, travel direction) on the 0.2 m global grid with the cost computed per direction from stored gradients, or a cost hook in Smac.
8. Caveat for the paper: Gazebo has no deformable soil, so traction claims stay geometric (grade limits), and the slip numbers are only Gazebo friction.

Estimate: steps 1 to 4 about 2 days, 5 and 6 about 2 days, step 7 2 to 3 days if needed. This moves v2 to about 18 to 20 Nov.

## Robustness and advanced features (general survey, 2026-11-07)

From a broad survey of terrain navigation systems (local, `~/rover_3d_nav_results/paper/general_survey.md`). Order and where each fits:

| Item | Source | Fits | Effort |
|---|---|---|---|
| Control-barrier safety filter on MPPI samples (TerrainCritic already costs u > 1 at 1e6, which is near-rejection for static limits) | GS-MPPI (Rabiee and Hoagg), epsilon-constraint MPC | after the dynamic margin | medium |
| Small-hole inpainting of the elevation map (OpenCV first, learned later). Unknown cells slow the governor (u 0.5) and raise cost. | Stölzle et al., RA-L | speed work | small |
| **First:** dynamic rollover margin (zero-moment point with centripetal and braking acceleration) as one more utilisation. The swerve tip aborts happened on flat ground while spinning, which a static margin cannot see. | Daş, Ames, Burdick 2024 | with swerve v2, speed work | medium |
| Backtrack-along-own-path recovery and a stuck "wiggle", in place of the tip abort ending the goal | STEP and CoSTAR at DARPA SubT | before hardware | medium |
| Robot-centric variance growth with odometry drift, LIO pose source | Fankhauser 2018, FAST-LIO2 | with real odometry (deferred) | medium |
| CVaR over utilisation in place of `z + kappa sigma` | STEP, PIETRA | paper | medium |
| Global cost-to-go and coarse, locally consistent global map | Field D* on MER, teach and repeat | Phase 6 | large |
| External benchmark and terramechanics (Verti-Bench on Chrono, SCM or CRM soil) | Xu et al. RSS 2025 | paper, if the ROS bridge is cheap | large |
| **(Ram)** Learned traversability or kinodynamics (Wild Visual Navigation, TAL, WayFAST). Out of SPEC scope today. | | ask after v2 | large |

## Paper (local, `~/rover_3d_nav_results/paper`)

- Established 3D baseline: `elevation_mapping_cupy` (Miki et al. 2022, ROS 2 Jazzy) plus a traversability costmap. PUTN if feasible.
- Ablations: utilisation vs weighted sum, best-heading vs heading-free, kappa on and off, governor on and off.
- More runs per goal (5) for the final tables.

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
