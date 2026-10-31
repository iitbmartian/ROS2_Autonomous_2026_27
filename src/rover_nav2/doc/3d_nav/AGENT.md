# 3D navigation: agent brief

You are building terrain-aware navigation for the IITB Mars Rover Team rover inside this colcon
workspace (ROS 2 Jazzy, Nav2, Gazebo Harmonic). The work happens in phases. Each phase ends on a
**gate**: a checkable criterion you prove with evidence, record, and then move on.

- Design, interfaces and the cost model: `src/rover_nav2/doc/3d_nav/SPEC.md`. Read it before any
  design or code decision.
- Phase tasks and gates: `src/rover_nav2/doc/3d_nav/PHASES.md`. Read the current phase's section
  at the start of every session.
- Progress log: `src/rover_nav2/doc/3d_nav/PROGRESS.md`. You own it. Append one entry per session.

## Session workflow

1. On branch `ram/3d_nav`, read `PROGRESS.md`, then the current phase in `PHASES.md`.
2. Work through all of `PHASES.md` continuously. Phases order the work, they are not stopping
   points. When a gate is met, record the evidence in `PROGRESS.md` (test output, benchmark
   numbers, RViz screenshots or bag names, each with the command that produced it), commit, and
   move straight on to the next phase.
3. Commit in small steps with clear messages on `ram/3d_nav`, without asking first. Ram is the
   only author: no `Co-Authored-By` line or any other Claude attribution in commit messages or
   PR descriptions. Build and test after each meaningful change. Always ask before pushing.
4. Make routine decisions yourself, following `SPEC.md` and this file: implementation details,
   parameter defaults, test design, file layout, fixing failing tests, choosing between
   equivalent libraries or approaches, small spec clarifications. Log each non-obvious decision
   in `PROGRESS.md` as one line, the choice and the reason.
5. When a gate item fails, debug and fix it. Keep trying different approaches until it passes,
   and log what was tried.
6. When `SPEC.md` is wrong in a minor way, fix the code, update `SPEC.md`, and log the change.
7. Keep the existing 2D stack working. Rebuild it and rerun its tests before each phase's final
   commit.
8. Keep `PROGRESS.md` current, so Ram can read it at any time and see what is done, what is in
   progress and why work stopped.

### When to stop and ask

Stop only for a major decision:

1. Abandoning or replacing a core design choice: FastDEM as the mapper, 2.5D mapping, the
   utilisation cost model, MPPI, the swerve mode.
2. Changing shared areas: `src/rover_interfaces/`, `docker/`, root `README.md` or
   `CONTRIBUTING.md`.
3. Pushing, opening a PR, or sending anything to another owner.
4. Anything that would change or break the existing 2D stack (`nav2_params_sim.yaml`,
   `navigation.launch.py`).
5. Work that needs real hardware or measured physical rover data.
6. A gate that still fails after several genuinely different attempts.

When stopping, write the question in `PROGRESS.md` with the recommended option and the
alternatives, then tell Ram.

### Packages owned by others

Changes to other owners' packages (for example `rover_gazebo` for the lidar, worlds and swerve
mode) go on separate local branches named `ram/<package>-<description>`, cut from `main`. Commit
them but do not push. Merge them into `ram/3d_nav` locally so work is never blocked waiting for
an upstream merge. `PROGRESS.md` lists every such branch and what it changes, so Ram can send
them to the owner.

## Ground rules

- **Scope.** Write inside `src/rover_terrain_mapping/`, `src/rover_traversability/`,
  `src/rover_nav2/` and new launch files in `src/rover_bringup/`. Everything else belongs to other
  owners (see the root README table). When a phase needs a change there, make it on a separate
  small branch as described under "Packages owned by others".
- **Shared areas.** `src/rover_interfaces/`, `docker/`, root `README.md` and `CONTRIBUTING.md`
  change only with explicit approval, and the PR description must say so.
- **Existing behaviour stays intact.** The current 2D stack (`nav2_params_sim.yaml`,
  `navigation.launch.py`) keeps working unchanged. The 3D stack ships as new config and launch
  files selected by a launch argument.
- **Physical parameters** live in one file, `src/rover_traversability/config/rover_physical.yaml`.
  Mechanical data arrives later, so placeholder values are marked `# PLACEHOLDER` and every node
  that loads one logs a warning at startup. Code reads physical values only from that file.
- **Tests.** Every geometric function (stability margin, clearance, articulation, utilisation,
  dilation) gets gtest coverage with hand-computed cases. A gate that cites a number cites the
  command that produced it.
- **Style.** Prose in docs and code comments uses commas and periods. Em dashes and semicolons
  stay out of prose (C++ statement semicolons are fine). One-line comments that say why.
- **Honesty.** Report failures and surprises plainly in `PROGRESS.md`, including anything in
  `SPEC.md` that turned out wrong. Propose the spec change instead of silently diverging.

## Repo gotchas

- Robot base frame is `base_footprint`. `base_link` sits about 0.72 m above the ground.
- Local costmap runs in `odom`, global costmap in `map` on RTAB-Map's `/map`.
- Use `/camera/depth_points` (built by `rover_bringup/launch/pipeline.launch.py`). Gazebo's own
  `/camera/points` is mis-framed.
- colcon stops descending once it finds a package, so new packages go directly under `src/`, never
  inside `src/rover_nav2/`.
- On a machine without the ZED SDK, put a local `COLCON_IGNORE` in
  `src/rover_drivers/zed2i_ws/src/zed-ros2-wrapper/` (it is gitignored).
- The intended build environment is the container in `docker/`.
