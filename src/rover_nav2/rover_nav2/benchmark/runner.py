"""Benchmark runner: drives goal sets through Nav2 and scores each run from ground truth.

Every run starts a fresh simulated stack (Gazebo, EKF, RTAB-Map with an empty database, Nav2,
plus terrain mapping for the 3D stack), so no run inherits a map from another. The runner and the
stack share an isolated ROS domain and Gazebo partition, so a benchmark never talks to anyone
else's session.

Per run, from Gazebo's ground-truth /odom: success, time to goal, recoveries, unsafe entries,
path length, planner time, CPU load, real-time factor. See metrics.py and SPEC.md "Benchmark".
"""

import csv
import gc
import hashlib
import math
import os
import signal
import subprocess
import threading
import time

import psutil
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener

from rover_nav2.benchmark import metrics

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_ABORTED: 'aborted',
    GoalStatus.STATUS_CANCELED: 'canceled',
}

CSV_FIELDS = [
    'run_id', 'config', 'world', 'goal_id', 'rep', 'status', 'success', 'final_error_m',
    'time_to_goal_s', 'recoveries', 'unsafe_entries', 'max_roll_deg', 'max_pitch_deg',
    'path_length_m', 'planner_time_s', 'cpu_mean_pct', 'cpu_peak_pct', 'rtf', 'wall_s',
    'note', 'git_commit', 'params_sha256',
    # Where each layer thinks the rover ended, against ground truth: odom is the EKF, map is
    # RTAB-Map on top of it. Splits localisation error between the two.
    'odom_err_m', 'map_err_m', 'odom_yaw_err_deg', 'map_yaw_err_deg',
    # Velocity chain, moving/total messages per stage while navigating: shows which link
    # dropped the command when the rover never moved.
    'cmd_chain',
]

# Stage name, topic, message type, along the velocity chain from the controller to the wheels.
CMD_CHAIN = [('nav', 'cmd_vel_nav', TwistStamped), ('smooth', 'cmd_vel_smoothed', TwistStamped),
             ('out', 'cmd_vel', TwistStamped),
             ('wheel', 'wheel_controller/commands', Float64MultiArray)]


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Recorder(Node):
    """ROS side of one run: ground-truth poses, the two Nav2 actions, lifecycle state."""

    def __init__(self, context=None):
        # No rosout: the node is recreated under the same name, and nothing reads its log there.
        # Wall clock node: sim time comes from /odom stamps (sim_now), because /clock arrives every
        # 1 ms physics step and cost this Python node about 18 percent of a core.
        super().__init__('nav_benchmark', context=context, enable_rosout=False)
        self._lock = threading.Lock()
        self._poses = []
        self._sim_t = 0.0
        self.recording = False
        self.create_subscription(Odometry, '/odom', self._odom, 50)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.plan = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.bt_state = self.create_client(GetState, '/bt_navigator/get_state')
        self.recoveries = 0
        self.tf = Buffer()
        self._tfl = TransformListener(self.tf, self, spin_thread=False)
        self.chain = {name: [0, 0] for name, _, _ in CMD_CHAIN}
        for name, topic, typ in CMD_CHAIN:
            self.create_subscription(typ, topic, lambda m, n=name: self._cmd(n, m), 10)

    def _cmd(self, name, msg):
        if not self.recording:
            return
        if isinstance(msg, Float64MultiArray):
            moving = any(abs(v) > 0.05 for v in msg.data)
        else:
            t = msg.twist
            moving = abs(t.linear.x) + abs(t.linear.y) + abs(t.angular.z) > 0.02
        with self._lock:
            c = self.chain[name]
            c[0] += int(moving)
            c[1] += 1

    def take_chain(self):
        with self._lock:
            out = ' '.join(f'{n} {m}/{t}' for n, (m, t) in self.chain.items())
            self.chain = {n: [0, 0] for n in self.chain}
        return out

    def pose_in(self, frame):
        """Latest (x, y, yaw) of base_footprint in frame, or None."""
        try:
            t = self.tf.lookup_transform(frame, 'base_footprint', rclpy.time.Time())
        except Exception:  # noqa: BLE001, any TF failure means no estimate
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                metrics.quat_to_rpy(q.x, q.y, q.z, q.w)[2])

    def _odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._sim_t = max(self._sim_t, t)
        if not self.recording:
            return
        p = msg.pose.pose
        r, pi, y = metrics.quat_to_rpy(p.orientation.x, p.orientation.y,
                                       p.orientation.z, p.orientation.w)
        with self._lock:
            self._poses.append((t, p.position.x, p.position.y, p.position.z, r, pi, y))

    def take_poses(self):
        with self._lock:
            poses, self._poses = self._poses, []
        return poses

    def sim_now(self):
        """Sim time of the newest /odom message. It stops if the sim stalls, like /clock would."""
        return self._sim_t

    def goal_pose(self, goal):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x = float(goal['x'])
        ps.pose.position.y = float(goal['y'])
        ps.pose.orientation.z, ps.pose.orientation.w = yaw_to_quat(float(goal.get('yaw', 0.0)))
        return ps


def wait_future(fut, timeout):
    end = time.time() + timeout
    while not fut.done() and time.time() < end:
        time.sleep(0.05)
    return fut.result() if fut.done() else None


def clean_fastdds_shm():
    """Removes Fast DDS shared memory left by killed stacks. Only zombie segments go, live ones
    are untouched. Stale port locks there were stalling Nav2's lifecycle startup."""
    try:
        subprocess.run(['fastdds', 'shm', 'clean'], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, timeout=30, check=False)
    except (OSError, subprocess.TimeoutExpired):
        pass


# Spawned active by rover_sim.launch.py, without them the rover steers but never drives.
DRIVE_CONTROLLERS = ('steer_controller', 'wheel_controller')


def inactive_controllers(path):
    """The drive controllers the stack log has not yet reported active."""
    try:
        with open(path, errors='replace') as f:
            text = f.read()
    except OSError:
        return list(DRIVE_CONTROLLERS)
    return [c for c in DRIVE_CONTROLLERS if f'Configured and activated {c}' not in text]


def nav2_active_in_log(path):
    """True once the navigation lifecycle manager has logged that its nodes are active."""
    try:
        with open(path, errors='replace') as f:
            return 'Managed nodes are active' in f.read()
    except OSError:
        return False


class Stack:
    """The simulated stack as a process group, with a marker in its environment so stragglers
    that escape the group (gz sim has been seen to) can be found and stopped."""

    def __init__(self, cmd, env, log_path, marker):
        self.marker = marker
        self.log_path = log_path
        self.log = open(log_path, 'w')
        clean_fastdds_shm()
        self.proc = subprocess.Popen(cmd, env=env, stdout=self.log, stderr=subprocess.STDOUT,
                                     start_new_session=True)

    def processes(self):
        try:
            root = psutil.Process(self.proc.pid)
            return [root] + root.children(recursive=True)
        except psutil.NoSuchProcess:
            return []

    def stop(self):
        for sig, wait in ((signal.SIGINT, 30), (signal.SIGTERM, 10), (signal.SIGKILL, 5)):
            if self.proc.poll() is not None:
                break
            try:
                os.killpg(self.proc.pid, sig)
            except ProcessLookupError:
                break
            try:
                self.proc.wait(wait)
            except subprocess.TimeoutExpired:
                pass
        left = kill_marked(self.marker)
        self.log.close()
        return left


def marked_processes(marker):
    me = os.getpid()
    out = []
    for p in psutil.process_iter(['pid']):
        if p.pid == me:
            continue
        try:
            run = p.environ().get('ROVER_BENCH_RUN')
            # marker None: any run's process, a leftover of an earlier stack.
            if run is not None and (marker is None or run == marker):
                out.append(p)
        except (psutil.AccessDenied, psutil.NoSuchProcess, psutil.ZombieProcess):
            pass
    return out


def dump_threads(marker, path):
    """Each of the run's processes with its threads and where they wait, for stalled starts."""
    with open(path, 'w') as f:
        for p in marked_processes(marker):
            try:
                f.write(f'{p.pid} {p.name()} {p.status()}\n')
                for t in sorted(os.listdir(f'/proc/{p.pid}/task')):
                    d = f'/proc/{p.pid}/task/{t}'
                    with open(d + '/comm') as c, open(d + '/wchan') as w:
                        f.write(f'    {c.read().strip():16s} {w.read().strip()}\n')
                shm = set()
                with open(f'/proc/{p.pid}/maps') as m:
                    for line in m:
                        if '/dev/shm' in line:
                            shm.add(line.split()[-1])
                f.write(''.join(f'    shm {x}\n' for x in sorted(shm)))
            except (OSError, psutil.Error):
                pass


def kill_marked(marker):
    """Stop every process still carrying this run's marker (None: any run's). Returns their
    names."""
    procs = marked_processes(marker)
    names = []
    for p in procs:
        try:
            names.append(p.name())
        except psutil.NoSuchProcess:
            pass
    for p in procs:
        try:
            p.terminate()
        except psutil.NoSuchProcess:
            pass
    _, alive = psutil.wait_procs(procs, timeout=5)
    for p in alive:
        try:
            p.kill()
        except psutil.NoSuchProcess:
            pass
    return names


class CpuSampler(threading.Thread):
    """Total CPU percent (100 = one core) of the stack's process tree, sampled at 2 Hz."""

    def __init__(self, stack):
        super().__init__(daemon=True)
        self.stack = stack
        self.samples = []
        self._halt = threading.Event()
        self._seen = {}

    def run(self):
        while not self._halt.wait(0.5):
            total = 0.0
            for p in self.stack.processes():
                try:
                    proc = self._seen.setdefault(p.pid, p)
                    total += proc.cpu_percent(None)
                except psutil.NoSuchProcess:
                    pass
            self.samples.append(total)

    def stop(self):
        self._halt.set()
        self.join(2)
        s = self.samples[2:]   # the first samples of each process read 0 by psutil's design
        return (sum(s) / len(s), max(s)) if s else (float('nan'), float('nan'))


def git_commit(repo):
    try:
        c = subprocess.run(['git', '-C', repo, 'rev-parse', '--short', 'HEAD'],
                           capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(['git', '-C', repo, 'status', '--porcelain', '--', 'src'],
                               capture_output=True, text=True).stdout.strip()
        return c + ('+dirty' if dirty else '')
    except (OSError, subprocess.CalledProcessError):
        return 'unknown'


def sha256(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()[:16]


class Runner:

    def __init__(self, args, phys, roll_limit, pitch_limit, params_path, log):
        self.a = args
        self.phys = phys
        self.roll_limit = roll_limit
        self.pitch_limit = pitch_limit
        self.params_sha = sha256(params_path)
        self.commit = git_commit(args.repo)
        self.log = log
        self.node = None
        self.start_node()

    def start_node(self):
        """The runner's ROS node in a context of its own, so it can be replaced by a fresh DDS
        participant. A participant kept across many stack restarts sometimes never discovers
        the new stack's services (bt_navigator/get_state), though Nav2 itself is active."""
        self.ctx = rclpy.Context()
        rclpy.init(context=self.ctx)
        self.node = Recorder(context=self.ctx)
        self.executor = MultiThreadedExecutor(context=self.ctx)
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

    def stop_node(self):
        if self.node is None:
            return
        node, executor, spin, ctx = self.node, self.executor, self.spin_thread, self.ctx

        def teardown():
            executor.shutdown(timeout_sec=5.0)
            spin.join(timeout=5.0)
            node.destroy_node()
            rclpy.try_shutdown(context=ctx)
            # try_shutdown leaves the DDS participant alive, leaked ones crowd the domain and
            # the stack's own nodes then miss each other in discovery.
            ctx.destroy()
        # The teardown once deadlocked for 1.5 h between runs (2026-11-06), so it gets a bound
        # and a stuck one leaks its context instead of stopping the benchmark.
        t = threading.Thread(target=teardown, daemon=True)
        t.start()
        t.join(timeout=30.0)
        if t.is_alive():
            self.log('runner node teardown stuck after 30 s, its context is left behind')
        self.node = self.executor = self.spin_thread = self.ctx = None
        # The node and its callbacks form reference cycles, only the cycle collector frees them.
        gc.collect()

    def launch_cmd(self, goalset, db_path):
        launch = 'pipeline_3d.launch.py' if self.a.config == '3d' else 'pipeline.launch.py'
        sp = goalset['spawn']
        cmd = ['ros2', 'launch', 'rover_bringup', launch,
               f"world:={goalset['world']}", 'gui:=false', 'viz:=false',
               f"x:={sp['x']}", f"y:={sp['y']}", f"z:={sp['z']}", f"yaw:={sp['yaw']}",
               f'database_path:={db_path}', 'delete_db_on_start:=true']
        if self.a.params_file and self.a.config == '2d':
            cmd.append(f'nav_params_file:={self.a.params_file}')
        cmd += list(self.a.launch_args)
        return cmd

    def wait_ready(self, stack):
        """Nav2 active and a plan to the first goal possible. Returns a note on failure."""
        end = time.time() + self.a.startup_timeout
        active_since, renewed = None, 0.0
        while time.time() < end:
            if stack.proc.poll() is not None:
                return 'stack exited during startup'
            if active_since is None and nav2_active_in_log(stack.log_path):
                active_since = time.time()
            # Our participant joins only once Nav2 is up. Joining while the stack's 50 nodes come
            # up stalled most starts on 2026-11-04 (1 of 4), stacks started without it never did.
            if self.node is None:
                if active_since is not None:
                    self.start_node()
                else:
                    time.sleep(2.0)
                    continue
            # Nav2 says it is active but this node cannot see it 20 s later: a discovery miss,
            # so retry with a fresh participant, at most every 30 s.
            now = time.time()
            if (active_since is not None and now - active_since > 20 and now - renewed > 30
                    and not self.node.bt_state.service_is_ready()):
                self.log('Nav2 active in its log but not discovered, renewing the node')
                self.stop_node()
                self.start_node()
                renewed = now
            if self.node.bt_state.wait_for_service(timeout_sec=1.0):
                res = wait_future(self.node.bt_state.call_async(GetState.Request()), 5)
                if res is not None and res.current_state.label == 'active':
                    # A spawner whose load_controller call timed out under load leaves the wheels
                    # unloaded and Nav2 active (2 of 5 starts on 2026-11-06).
                    missing = inactive_controllers(stack.log_path)
                    if not missing:
                        return None
                    if time.time() - active_since > 60:
                        return f"controllers not active: {' '.join(missing)}"
            time.sleep(2.0)
        return 'Nav2 not active before startup_timeout'

    def planner_time(self, goal):
        """Planning time of one ComputePathToPose call, retried while the map and TF settle."""
        end = time.time() + self.a.plan_timeout
        while time.time() < end:
            if not self.node.plan.wait_for_server(timeout_sec=2.0):
                continue
            g = ComputePathToPose.Goal()
            g.goal = self.node.goal_pose(goal)
            g.use_start = False
            gh = wait_future(self.node.plan.send_goal_async(g), 10)
            if gh is not None and gh.accepted:
                res = wait_future(gh.get_result_async(), 30)
                if res is not None and res.status == GoalStatus.STATUS_SUCCEEDED:
                    t = res.result.planning_time
                    return t.sec + t.nanosec * 1e-9
            time.sleep(3.0)
        return float('nan')

    def navigate(self, goal, timeout_s):
        """Send the goal and wait in sim time. Returns (status, time_to_goal, recoveries, rtf)."""
        self.node.recoveries = 0

        def feedback(fb):
            self.node.recoveries = fb.feedback.number_of_recoveries

        if not self.node.nav.wait_for_server(timeout_sec=30.0):
            return 'no_action_server', float('nan'), 0, float('nan')
        g = NavigateToPose.Goal()
        g.pose = self.node.goal_pose(goal)
        gh = wait_future(self.node.nav.send_goal_async(g, feedback_callback=feedback), 30)
        if gh is None or not gh.accepted:
            return 'rejected', float('nan'), 0, float('nan')
        # sim_now comes from /odom, so wait for the first message before starting the clock.
        w_odom = time.time()
        while self.node.sim_now() == 0.0 and time.time() - w_odom < 30.0:
            time.sleep(0.1)
        t0, w0 = self.node.sim_now(), time.time()
        res_f = gh.get_result_async()
        # Wall guard in case the sim stalls entirely: far beyond any real-time factor seen.
        wall_limit = timeout_s * self.a.wall_factor
        while not res_f.done():
            if self.node.sim_now() - t0 > timeout_s or time.time() - w0 > wall_limit:
                wait_future(gh.cancel_goal_async(), 10)
                wait_future(res_f, 20)
                t1, w1 = self.node.sim_now(), time.time()
                return 'timeout', t1 - t0, self.node.recoveries, (t1 - t0) / max(w1 - w0, 1e-6)
            time.sleep(0.1)
        t1, w1 = self.node.sim_now(), time.time()
        status = STATUS_NAMES.get(res_f.result().status, f'status_{res_f.result().status}')
        return status, t1 - t0, self.node.recoveries, (t1 - t0) / max(w1 - w0, 1e-6)

    def run_one(self, goalset, goal, rep, out_dir, env):
        run_id = f"{self.a.tag or self.a.config}_{goalset['name']}_{goal['id']}_r{rep}"
        marker = f'{run_id}_{os.getpid()}_{int(time.time())}'
        run_env = dict(env, ROVER_BENCH_RUN=marker)
        db = os.path.join(out_dir, 'db', f'{run_id}.db')
        log_path = os.path.join(out_dir, 'logs', f'{run_id}.log')
        row = {k: '' for k in CSV_FIELDS}
        row.update(run_id=run_id, config=self.a.tag or self.a.config, world=goalset['world'],
                   goal_id=goal['id'], rep=rep, git_commit=self.commit,
                   params_sha256=self.params_sha)
        self.log(f'[{run_id}] starting stack')
        w_start = time.time()
        # Nav2's lifecycle manager sometimes waits forever for a service that DDS never
        # discovers (seen in sim: bt_navigator/get_state). That is no result about the
        # navigation stack, so a stalled start gets one fresh stack before the run is scored.
        retried = ''
        for attempt in (1, 2):
            # Fresh participant per stack: stop ours before the shared-memory clean, start after.
            self.stop_node()
            # A process left from an earlier stack can hold a Fast DDS shared-memory lock, and
            # every new node then hangs creating its participant (2026-11-03, 19:30 to 22:00).
            old = kill_marked(None)
            if old:
                self.log(f"[{run_id}] stopped leftovers of earlier stacks: {' '.join(old)}")
            stack = Stack(self.launch_cmd(goalset, db), run_env, log_path, marker)
            note = self.wait_ready(stack)
            if note is None or attempt == 2:
                break
            self.log(f'[{run_id}] startup stalled ({note}), restarting the stack once')
            dump_threads(marker, log_path + '.stall')
            stack.stop()
            retried = f'startup retried after: {note}'
        row['note'] = retried
        if self.node is None:
            self.start_node()
        cpu = CpuSampler(stack)
        cpu.start()
        try:
            if note is None:
                pt = self.planner_time(goal)
                row['planner_time_s'] = f'{pt:.4f}'
                if math.isnan(pt):
                    row['note'] = 'planner gave no path within plan_timeout'
                self.node.take_poses()
                self.node.take_chain()
                self.node.recording = True
                status, ttg, rec, rtf = self.navigate(goal, goalset['timeout_s'])
                self.node.recording = False
                poses = self.node.take_poses()
                row['cmd_chain'] = self.node.take_chain()
                self.save_poses(out_dir, run_id, poses)
                if poses:
                    sp = goalset['spawn']
                    for frame in ('odom', 'map'):
                        e = metrics.estimate_error(self.node.pose_in(frame), poses[-1], sp)
                        if e is not None:
                            row[f'{frame}_err_m'] = f'{e[0]:.3f}'
                            row[f'{frame}_yaw_err_deg'] = f'{math.degrees(e[1]):.2f}'
                err = metrics.final_error(poses, (goal['x'], goal['y']))
                mr, mp = metrics.max_abs_attitude(poses)
                row.update(
                    status=status,
                    success=int(metrics.is_refused(status, poses, goal['refuse_z_max'])
                                if goal.get('expect') == 'refuse'
                                else metrics.is_success(status, err, self.a.success_radius)),
                    final_error_m=f'{err:.3f}', time_to_goal_s=f'{ttg:.2f}', recoveries=rec,
                    unsafe_entries=metrics.unsafe_entries(
                        poses, self.roll_limit, self.pitch_limit),
                    max_roll_deg=f'{math.degrees(mr):.2f}',
                    max_pitch_deg=f'{math.degrees(mp):.2f}',
                    path_length_m=f'{metrics.path_length(poses):.3f}', rtf=f'{rtf:.3f}')
            else:
                row.update(status='startup_failed', success=0, note=note)
        finally:
            self.node.recording = False
            cm, cp = cpu.stop()
            row.update(cpu_mean_pct=f'{cm:.1f}', cpu_peak_pct=f'{cp:.1f}')
            left = stack.stop()
            if left:
                row['note'] = (row['note'] + f"; stopped stray {' '.join(left)}").lstrip('; ')
            row['wall_s'] = f'{time.time() - w_start:.1f}'
            # About 17 MB per run, and the metrics need only the poses. Kept with --keep-db.
            if not getattr(self.a, 'keep_db', False):
                for f in (db, db + '-shm', db + '-wal'):
                    if os.path.exists(f):
                        os.remove(f)
        self.log(f"[{run_id}] {row['status']} success={row['success']} "
                 f"t={row['time_to_goal_s']} unsafe={row['unsafe_entries']} "
                 f"rtf={row['rtf']} {row['note']}")
        return row

    @staticmethod
    def save_poses(out_dir, run_id, poses):
        with open(os.path.join(out_dir, 'poses', f'{run_id}.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'])
            w.writerows(poses)

    def shutdown(self):
        self.stop_node()
