#!/usr/bin/env python3
"""Brute-force task-point scheduler + nav2 executor.

Listens for a manually-published mission description, brute-forces every
visiting order of the candidate points to find the order that collects the
most points within a time/distance budget, then drives the rover through
nav2 to each selected point in turn, pausing at each one to "do the task".

Input message (std_msgs/Float64MultiArray) layout, all values float64:

    [n,
     x_1, y_1, t_1, p_1,
     ...
     x_n, y_n, t_n, p_n,
     T, D]

    n  - number of candidate points (0 <= n < 10)
    x_i, y_i - coordinates of point i, in the map frame
    t_i - seconds the rover must stay at point i to finish its task
    p_i - points awarded for finishing the task at point i
    T   - total mission time budget, seconds
    D   - total distance budget, metres

Publish it manually, e.g. for two points:

    ros2 topic pub --once /mission/input std_msgs/msg/Float64MultiArray \\
      "{data: [2, 3.0, 2.0, 5.0, 10.0, -4.0, 1.0, 3.0, 6.0, 120.0, 30.0]}"
"""
import math
import threading
from itertools import permutations

import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

MAX_POINTS = 10
DEFAULT_SPEED_MPS = 2.0
ORIGIN = (0.0, 0.0)


def parse_mission(data):
    """Unpack the flat [n, x,y,t,p, ..., T, D] layout described above."""
    data = list(data)
    if len(data) < 1:
        raise ValueError('empty mission message')

    n = int(round(data[0]))
    if not 0 <= n < MAX_POINTS:
        raise ValueError(f'n must satisfy 0 <= n < {MAX_POINTS}, got {n}')

    expected_len = 1 + 4 * n + 2
    if len(data) != expected_len:
        raise ValueError(f'expected {expected_len} values for n={n} points, got {len(data)}')

    points = []
    idx = 1
    for _ in range(n):
        x, y, t, p = data[idx:idx + 4]
        points.append((x, y, t, p))
        idx += 4

    time_budget = data[idx]
    dist_budget = data[idx + 1]
    return points, time_budget, dist_budget


def best_route(points, time_budget, dist_budget, speed=DEFAULT_SPEED_MPS, origin=ORIGIN):
    """Brute-force every ordering of `points` and pick the best feasible plan.

    For each of the n! orderings, walk the points in that order from `origin`,
    skipping a point whenever visiting it would push cumulative distance past
    `dist_budget` or cumulative time (travel at `speed` + task time) past
    `time_budget`. Any optimal subset+order is a prefix of some ordering, so
    this still finds the true optimum, not just an approximation.

    Returns (visited_indices, distance_used, time_used, points_scored, efficiency).
    """
    n = len(points)
    if n == 0:
        return [], 0.0, 0.0, 0.0, 0.0

    best_key = None
    best = ([], 0.0, 0.0, 0.0, 0.0)

    for order in permutations(range(n)):
        visited = []
        cur = origin
        dist_used = 0.0
        time_used = 0.0
        points_scored = 0.0

        for idx in order:
            x, y, t, p = points[idx]
            leg = math.hypot(x - cur[0], y - cur[1])
            travel_time = leg / speed
            cand_dist = dist_used + leg
            cand_time = time_used + travel_time + t
            if cand_dist <= dist_budget and cand_time <= time_budget:
                dist_used = cand_dist
                time_used = cand_time
                points_scored += p
                visited.append(idx)
                cur = (x, y)

        efficiency = (points_scored / time_used) if time_used > 0 else 0.0
        key = (points_scored, efficiency)
        if best_key is None or key > best_key:
            best_key = key
            best = (visited, dist_used, time_used, points_scored, efficiency)

    return best


class MissionCommander(Node):

    def __init__(self):
        super().__init__('mission_commander')
        self.declare_parameter('input_topic', '/mission/input')
        self.declare_parameter('status_topic', '/mission/status')
        self.declare_parameter('speed_mps', DEFAULT_SPEED_MPS)

        input_topic = self.get_parameter('input_topic').value
        status_topic = self.get_parameter('status_topic').value
        self.speed = float(self.get_parameter('speed_mps').value)

        self._busy = threading.Lock()
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(Float64MultiArray, input_topic, self._on_mission, 10)

        self.get_logger().info(f'Mission commander ready, listening on {input_topic}')

    def _status(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def _on_mission(self, msg):
        if not self._busy.acquire(blocking=False):
            self._status('Mission already in progress, ignoring new input.')
            return
        threading.Thread(target=self._run_mission, args=(list(msg.data),), daemon=True).start()

    def _run_mission(self, data):
        try:
            points, time_budget, dist_budget = parse_mission(data)
        except ValueError as exc:
            self._status(f'Rejected mission input: {exc}')
            self._busy.release()
            return

        self._status(
            f'Received {len(points)} candidate point(s), budget T={time_budget:.1f}s '
            f'D={dist_budget:.1f}m. Brute-forcing '
            f'{math.factorial(len(points)) if points else 1} orderings...'
        )

        visited, dist_used, time_used, points_scored, efficiency = best_route(
            points, time_budget, dist_budget, self.speed)

        if not visited:
            self._status('No point is reachable within the given time/distance budget.')
            self._busy.release()
            return

        order_str = ' -> '.join(f'({points[i][0]:.2f},{points[i][1]:.2f})' for i in visited)
        self._status(
            f'Best plan visits {len(visited)}/{len(points)} point(s) for {points_scored:.1f} pts, '
            f'using {dist_used:.2f}/{dist_budget:.2f} m and {time_used:.1f}/{time_budget:.1f} s '
            f'(efficiency {efficiency:.3f} pts/s). Order: {order_str}'
        )

        self._execute_route(points, visited)
        self._busy.release()

    def _execute_route(self, points, visited):
        navigator = BasicNavigator()
        # localizer='robot_localization' skips the default wait for amcl (and its initial pose):
        # the pipeline runs slam_toolbox, which owns map->odom, so amcl never exists.
        navigator.waitUntilNav2Active(localizer='robot_localization')

        for step, idx in enumerate(visited, start=1):
            x, y, t, p = points[idx]
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = navigator.get_clock().now().to_msg()
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.orientation.w = 1.0

            self._status(f'[{step}/{len(visited)}] Navigating to ({x:.2f}, {y:.2f})...')
            navigator.goToPose(goal)

            while not navigator.isTaskComplete():
                pass

            result = navigator.getResult()
            if result != TaskResult.SUCCEEDED:
                self._status(f'Failed to reach ({x:.2f}, {y:.2f}): {result}. Aborting remaining route.')
                break

            self._status(f'Reached ({x:.2f}, {y:.2f}). Working {t:.1f}s to earn {p:.1f} pts...')
            self._sim_sleep(navigator, t)

        navigator.destroy_node()
        self._status('Mission complete.')

    @staticmethod
    def _sim_sleep(navigator, seconds):
        if seconds <= 0.0:
            return
        start = navigator.get_clock().now()
        duration = Duration(seconds=seconds)
        while navigator.get_clock().now() - start < duration:
            rclpy.spin_once(navigator, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = MissionCommander()
    # Spin this node on a private executor. The mission runs in a worker thread and its
    # BasicNavigator spins rclpy's global executor (spin_until_future_complete / spin_once);
    # rclpy.spin(node) here would occupy that same executor and raise
    # "RuntimeError: Executor is already spinning".
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
