"""Isolate each simulation launch and kill whatever it leaves behind.

Two problems, one fix.

Leaks. ros_gz_sim starts Gazebo through gz-tools' ruby wrapper (`gz sim ...`). On
shutdown launch signals only that wrapper. Headless (-s), the wrapper *is* the
server and it ignores SIGINT and SIGTERM. With the GUI, the wrapper forks a
separate `gz sim server` into a process group of its own, which no process-group
sweep finds. Either way a server survives the launch, keeps the world running, and
keeps its in-process /controller_manager on the ROS graph.

Crosstalk. All Gazebo instances share one gz-transport partition by default, so
the next launch's bridge and controller spawners talk to the leftover server as
well as their own: duplicated /clock, controllers that fail to configure, a rover
that never moves.

isolate_and_reap() returns launch actions that

    - set ROVER_TESTING_RUN=<unique id> for every process the launch starts, so
      they can all be found again by their environment, whatever their group;
    - set GZ_PARTITION to the same id (unless the caller already chose one), so
      this run's gz-transport traffic cannot mix with any other Gazebo;
    - on shutdown, start this module detached. It waits for the launch process to
      exit, then SIGKILLs every process still carrying the marker. Processes from
      any other launch, or a Gazebo you started yourself, never carry it.

A Gazebo GUI attached by hand (`gz sim -g`) must share the partition to see the
world: the id is logged at startup.
"""

import os
import signal
import subprocess
import sys
import time
import uuid

from launch.actions import (LogInfo, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.event_handlers import OnShutdown

MARKER = "ROVER_TESTING_RUN"


def isolate_and_reap():
    run_id = os.environ.get(MARKER) or f"rover_testing_{os.getpid()}_{uuid.uuid4().hex[:8]}"
    partition = os.environ.get("GZ_PARTITION") or run_id

    def start_reaper(_context):
        subprocess.Popen(
            [sys.executable, "-m", "rover_testing.gz_reaper", str(os.getpid()), run_id],
            start_new_session=True, stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            # The reaper itself must not carry the marker it hunts for.
            env={k: v for k, v in os.environ.items() if k != MARKER})
        return []

    return [
        SetEnvironmentVariable(MARKER, run_id),
        SetEnvironmentVariable("GZ_PARTITION", partition),
        LogInfo(msg=f"rover_testing: run {run_id}, GZ_PARTITION={partition}"),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=start_reaper)])),
    ]


def marked_processes(run_id):
    """PIDs of this user's processes whose environment carries MARKER=run_id."""
    needle = f"{MARKER}={run_id}".encode()
    me = os.getpid()
    pids = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit() or int(entry) == me:
            continue
        try:
            with open(f"/proc/{entry}/environ", "rb") as f:
                if needle in f.read().split(b"\0"):
                    pids.append(int(entry))
        except OSError:   # gone, or not ours
            continue
    return pids


def main():
    launch_pid, run_id = int(sys.argv[1]), sys.argv[2]
    # Launch escalates SIGINT -> SIGTERM over ~10 s; give it a minute to finish.
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        try:
            os.kill(launch_pid, 0)
        except ProcessLookupError:
            break
        time.sleep(0.5)
    for pid in marked_processes(run_id):
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


if __name__ == "__main__":
    main()
