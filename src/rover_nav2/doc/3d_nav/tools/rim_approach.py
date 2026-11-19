#!/usr/bin/env python3
"""Drive the rover's footprint onto every crater rim wider than the wheel track, for the Phase 2
crater item (terrain_eval --craters scores flag time against footprint reach time).

    python3 rim_approach.py CRATERS_YAML LOG [--standoff 3.5] [--nav-timeout 300]

Per crater, nearest first: NavigateToPose to a standoff point facing the crater (the 3D stack
plans the way there), then a blind straight drive on /cmd_vel (TwistStamped) at 0.3 m/s, steering at the centre,
until the ground-truth footprint touches any wide rim, then a 1.5 m reverse. The blind leg is
what the 3D stack never does on its own (it drives around craters), so the reach event happens.
Pose is /odom (ground truth with odom_source:=ground_truth). All timing is sim time.
"""
import argparse
import math
import sys
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

from rover_nav2.benchmark.terrain_eval import footprint_reaches
from rover_traversability import physical_params


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def rp_of(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Driver(Node):
    def __init__(self, craters, phys, log):
        super().__init__('rim_approach', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.craters, self.phys, self.logf = craters, phys, log
        self.pose = None
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def log(self, s):
        line = f'{time.strftime("%H:%M:%S")} t={self.now():.1f} {s}'
        print(line, flush=True)
        self.logf.write(line + '\n')
        self.logf.flush()

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        r, pi = rp_of(q)
        self.pose = (p.x, p.y, yaw_of(q), r, pi)

    def spin_for(self, sim_s):
        t0 = self.now()
        while rclpy.ok() and self.now() - t0 < sim_s:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_pose(self):
        while rclpy.ok() and self.pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def touching(self):
        x, y, yaw = self.pose[:3]
        for k, c in enumerate(self.craters):
            if footprint_reaches(x, y, yaw, self.phys['footprint_length'],
                                 self.phys['footprint_width'], c['x'], c['y'], c['diameter'] / 2):
                return k
        return None

    def goto(self, x, y, yaw, timeout):
        if not self.nav.wait_for_server(timeout_sec=60.0):
            return 'no server'
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.pose.position.x, g.pose.pose.position.y = x, y
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        f = self.nav.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=30.0)
        h = f.result()
        if h is None or not h.accepted:
            return 'rejected'
        r = h.get_result_async()
        t0 = self.now()
        while rclpy.ok() and not r.done() and self.now() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not r.done():
            c = h.cancel_goal_async()
            rclpy.spin_until_future_complete(self, c, timeout_sec=10.0)
            return 'timeout'
        st = r.result().status
        return {GoalStatus.STATUS_SUCCEEDED: 'ok', GoalStatus.STATUS_ABORTED: 'aborted',
                GoalStatus.STATUS_CANCELED: 'canceled'}.get(st, str(st))

    def drive(self, v, w):
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'
        t.twist.linear.x, t.twist.angular.z = v, w
        self.cmd.publish(t)

    def stop(self):
        for _ in range(10):
            self.drive(0.0, 0.0)
            self.spin_for(0.1)

    def approach(self, c, max_dist=6.0, max_t=45.0):
        """Blind straight drive at the crater centre until the footprint touches a wide rim."""
        x0, y0 = self.pose[:2]
        t0 = self.now()
        while rclpy.ok():
            k = self.touching()
            if k is not None:
                self.stop()
                return f'touched crater {k}', k
            x, y, yaw, roll, pitch = self.pose
            if math.hypot(x - x0, y - y0) > max_dist:
                self.stop()
                return 'drove max distance', None
            if self.now() - t0 > max_t:
                self.stop()
                return 'time limit', None
            if max(abs(roll), abs(pitch)) > math.radians(20):
                self.stop()
                return f'tilt {math.degrees(roll):.1f}/{math.degrees(pitch):.1f} deg', None
            err = wrap(math.atan2(c['y'] - y, c['x'] - x) - yaw)
            self.drive(0.3 if abs(err) < math.radians(60) else 0.1, max(-0.3, min(0.3, 1.2 * err)))
            self.spin_for(0.1)
        return 'shutdown', None

    def reverse(self, dist=1.5):
        x0, y0 = self.pose[:2]
        t0 = self.now()
        while rclpy.ok() and math.hypot(self.pose[0] - x0, self.pose[1] - y0) < dist \
                and self.now() - t0 < 4.0 + dist / 0.3 * 1.5:
            self.drive(-0.3, 0.0)
            self.spin_for(0.1)
        self.stop()

    def standoffs(self, c, d):
        """Standoff points d from the crater on the rover's side first, then rotated, clear of
        other wide craters by 1 m."""
        x, y = self.pose[:2]
        base = math.atan2(y - c['y'], x - c['x'])
        for off in (0, 30, -30, 60, -60, 90, -90, 135, -135, 180):
            a = base + math.radians(off)
            sx, sy = c['x'] + d * math.cos(a), c['y'] + d * math.sin(a)
            if all(math.hypot(sx - o['x'], sy - o['y']) > o['diameter'] / 2 + 1.0
                   for o in self.craters):
                yield sx, sy, wrap(a + math.pi)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('craters')
    p.add_argument('log')
    p.add_argument('--standoff', type=float, default=3.5)
    p.add_argument('--nav-timeout', type=float, default=300.0)
    p.add_argument('--tries', type=int, default=3, help='standoff points tried per crater')
    # The touch leaves the wheels on a flagged rim. Back out far enough that the footprint and the
    # costmap's lethal buffer around that rim are behind the rover before Nav2 takes over.
    p.add_argument('--reverse', type=float, default=1.5, help='m driven back after a touch')
    a = p.parse_args()
    phys, _ = physical_params.load()
    craters = [c for c in yaml.safe_load(open(a.craters))['craters'] if c['diameter'] > phys['track']]
    rclpy.init()
    n = Driver(craters, phys, open(a.log, 'a'))
    n.wait_pose()
    n.log(f'{len(craters)} craters wider than the track {phys["track"]} m, footprint '
          f'{phys["footprint_length"]} x {phys["footprint_width"]} m')
    reached = set()
    todo = set(range(len(craters)))
    while todo and rclpy.ok():
        x, y = n.pose[:2]
        k = min(todo, key=lambda i: math.hypot(craters[i]['x'] - x, craters[i]['y'] - y))
        todo.discard(k)
        if k in reached:
            continue
        c = craters[k]
        res = 'no standoff'
        for i, (sx, sy, syaw) in enumerate(n.standoffs(c, a.standoff)):
            if i >= a.tries:
                break
            res = n.goto(sx, sy, syaw, a.nav_timeout)
            n.log(f'crater {k} ({c["x"]:.2f}, {c["y"]:.2f}) d {c["diameter"]}: standoff '
                  f'({sx:.2f}, {sy:.2f}) {res}')
            if res == 'ok':
                break
        if res != 'ok':
            n.log(f'crater {k}: skipped, no standoff reached')
            continue
        out, hit = n.approach(c)
        if hit is not None:
            reached.add(hit)
        n.log(f'crater {k}: approach {out}, at ({n.pose[0]:.2f}, {n.pose[1]:.2f})')
        n.reverse(a.reverse)
    n.log(f'done, footprint touched {len(reached)} of {len(craters)}: {sorted(reached)}')
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
