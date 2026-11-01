"""Benchmark runner: drives goal sets through Nav2 and scores each run from ground truth.

Every run starts a fresh simulated stack (Gazebo, EKF, RTAB-Map with an empty database, Nav2,
plus terrain mapping for the 3D stack), so no run inherits a map from another. The runner and the
stack share an isolated ROS domain and Gazebo partition, so a benchmark never talks to anyone
else's session.

Per run, from Gazebo's ground-truth /odom: success, time to goal, recoveries, unsafe entries,
path length, planner time, CPU load, real-time factor. See metrics.py and SPEC.md "Benchmark".
"""

import csv
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
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

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
]


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Recorder(Node):
    """ROS side of one run: ground-truth poses, the two Nav2 actions, lifecycle state."""

    def __init__(self):
        super().__init__('nav_benchmark', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self._lock = threading.Lock()
        self._poses = []
        self.recording = False
        self.create_subscription(Odometry, '/odom', self._odom, 50)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.plan = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.bt_state = self.create_client(GetState, '/bt_navigator/get_state')
        self.recoveries = 0

    def _odom(self, msg):
        if not self.recording:
            return
        p = msg.pose.pose
        r, pi, y = metrics.quat_to_rpy(p.orientation.x, p.orientation.y,
                                       p.orientation.z, p.orientation.w)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._poses.append((t, p.position.x, p.position.y, p.position.z, r, pi, y))

    def take_poses(self):
        with self._lock:
            poses, self._poses = self._poses, []
        return poses

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

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


class Stack:
    """The simulated stack as a process group, with a marker in its environment so stragglers
    that escape the group (gz sim has been seen to) can be found and stopped."""

    def __init__(self, cmd, env, log_path, marker):
        self.marker = marker
        self.log = open(log_path, 'w')
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
            if p.environ().get('ROVER_BENCH_RUN') == marker:
                out.append(p)
        except (psutil.AccessDenied, psutil.NoSuchProcess, psutil.ZombieProcess):
            pass
    return out


def kill_marked(marker):
    """Stop every process still carrying this run's marker. Returns how many there were."""
    procs = marked_processes(marker)
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
    return len(procs)


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
        self.node = Recorder()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def launch_cmd(self, goalset, db_path):
        launch = 'pipeline_3d.launch.py' if self.a.config == '3d' else 'pipeline.launch.py'
        sp = goalset['spawn']
        cmd = ['ros2', 'launch', 'rover_bringup', launch,
               f"world:={goalset['world']}", 'gui:=false', 'viz:=false',
               f"x:={sp['x']}", f"y:={sp['y']}", f"z:={sp['z']}", f"yaw:={sp['yaw']}",
               f'database_path:={db_path}', 'delete_db_on_start:=true']
        if self.a.params_file and self.a.config == '2d':
            cmd.append(f'nav_params_file:={self.a.params_file}')
        return cmd

    def wait_ready(self, stack):
        """Nav2 active and a plan to the first goal possible. Returns a note on failure."""
        end = time.time() + self.a.startup_timeout
        while time.time() < end:
            if stack.proc.poll() is not None:
                return 'stack exited during startup'
            if self.node.bt_state.wait_for_service(timeout_sec=1.0):
                res = wait_future(self.node.bt_state.call_async(GetState.Request()), 5)
                if res is not None and res.current_state.label == 'active':
                    return None
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
        run_id = f"{self.a.config}_{goalset['name']}_{goal['id']}_r{rep}"
        marker = f'{run_id}_{os.getpid()}_{int(time.time())}'
        run_env = dict(env, ROVER_BENCH_RUN=marker)
        db = os.path.join(out_dir, 'db', f'{run_id}.db')
        log_path = os.path.join(out_dir, 'logs', f'{run_id}.log')
        row = {k: '' for k in CSV_FIELDS}
        row.update(run_id=run_id, config=self.a.config, world=goalset['world'],
                   goal_id=goal['id'], rep=rep, git_commit=self.commit,
                   params_sha256=self.params_sha)
        self.log(f'[{run_id}] starting stack')
        w_start = time.time()
        stack = Stack(self.launch_cmd(goalset, db), run_env, log_path, marker)
        cpu = CpuSampler(stack)
        cpu.start()
        try:
            note = self.wait_ready(stack)
            if note is None:
                row['planner_time_s'] = f'{self.planner_time(goal):.4f}'
                self.node.take_poses()
                self.node.recording = True
                status, ttg, rec, rtf = self.navigate(goal, goalset['timeout_s'])
                self.node.recording = False
                poses = self.node.take_poses()
                self.save_poses(out_dir, run_id, poses)
                err = metrics.final_error(poses, (goal['x'], goal['y']))
                mr, mp = metrics.max_abs_attitude(poses)
                row.update(
                    status=status,
                    success=int(metrics.is_success(status, err, self.a.success_radius)),
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
                row['note'] = (row['note'] + f'; stopped {left} stray processes').lstrip('; ')
            row['wall_s'] = f'{time.time() - w_start:.1f}'
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
        self.executor.shutdown()
        self.node.destroy_node()
