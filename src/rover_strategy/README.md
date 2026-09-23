# rover_strategy

**Owner(s):** Nishant + Sohan
**Status:** implemented (`ament_python`) — task-allocation mission commander

## Scope

- Exploration
- Task allocation
- Recovery behaviours

## What's here

`mission_commander` (`rover_strategy/mission_commander.py`) subscribes to a manually-published
`std_msgs/Float64MultiArray` on `/mission/input` describing up to 9 candidate task points
(coordinates, task duration, point value) plus a total time budget `T` and distance budget `D`.
It brute-forces every visiting order (assuming a constant 2 m/s traverse speed and an
obstacle-free map), picks the order that collects the most points within budget, then drives the
rover to each selected point via nav2's `NavigateToPose`, pausing for that point's task duration
before moving on. Progress is logged and mirrored on `/mission/status` (`std_msgs/String`).

See the module docstring in `mission_commander.py` for the exact input layout and an example
`ros2 topic pub` command. Launched as part of `rover_gazebosim`'s `pipeline_launch.launch.py` via
`launch/mission.launch.py`.

See [CONTRIBUTING.md](../../CONTRIBUTING.md) for the branch and PR workflow.
