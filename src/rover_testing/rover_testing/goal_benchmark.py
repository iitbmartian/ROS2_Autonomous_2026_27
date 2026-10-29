"""Drive a route of Nav2 goals, time it, check localization, optionally score the map.

Started by a test case launch file (launch/tests/) alongside the pipeline. It:

    1. waits for the simulation clock, ground truth, the map -> base_footprint
       transform and an active bt_navigator;
    2. checks the initial localized pose against ground truth and the spawn point;
    3. sends one NavigateToPose goal per waypoint (map frame), each facing along
       its leg, timing the whole route in sim and wall seconds and accumulating
       the ground-truth path length. An intermediate waypoint counts as passed once
       the localized pose is within pass_radius of it: the goal is cancelled and
       the next one sent, as a mission planner would. Asked to stop exactly on
       every waypoint, the rover (0.8 m minimum turn radius) was measured circling
       a waypoint 0.3 m away for 300 s to satisfy Nav2's 0.15 m / 0.25 rad goal
       checker. The final waypoint needs a real Nav2 SUCCEEDED, unless
       navigation_checks is false. A navigation route stops at the first failed
       leg; a mapping route moves on to the next waypoint;
    4. checks the final localized pose against ground truth and that the rover is
       really at the last waypoint;
    5. with reference_map set, compares RTAB-Map's /map with that ideal map
       (map_compare.py) and checks precision, recall and coverage;
    6. writes a JSON report to result_file and exits 0 on pass, 1 on fail.

Checks (tolerance in m, from the UI):

    initial_localization   |localized - ground truth| <= tolerance, at the start
    initial_at_spawn       |localized - (0, 0)| <= tolerance, at the start
    nav2_succeeded         every leg passed and the final goal returned SUCCEEDED
                           within `timeout` sim seconds
    final_localization     |localized - ground truth| <= tolerance, at the end
    reached_goal           |ground truth - last waypoint| <= tolerance + nav_goal_tolerance
                           (Nav2 stops within nav_goal_tolerance of the goal as the
                           *localized* pose sees it)
    map_precision, map_recall, map_coverage   with reference_map, >= their minimums

Mapping tests (reference_map set) score the map, not the parking: they set
navigation_checks false, so every waypoint, the last too, is passed within
pass_radius, and nav2_succeeded and reached_goal give way to

    route_completed        every waypoint was passed

with Nav2's own result still in the report.
"""

import math
import os
import time

from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from rover_testing import map_compare
from rover_testing.benchmark_common import BenchmarkNode, run_node

STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}
LIFECYCLE_ACTIVE = 3
# /map is latched by RTAB-Map; match it so a late subscriber still gets the last one.
MAP_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class GoalBenchmark(BenchmarkNode):
    kind = "navigation"

    def __init__(self):
        super().__init__("goal_benchmark")
        # Flat [x1, y1, x2, y2, ...] in the map frame; the last one is the goal.
        self.declare_parameter("waypoints", [3.0, 3.0])
        self.declare_parameter("nav_goal_tolerance", 0.15)  # m, Nav2's xy_goal_tolerance
        self.declare_parameter("settle_time", 0.5)          # s after Nav2 is up
        self.declare_parameter("pass_radius", 0.5)          # m, intermediate waypoints
        self.declare_parameter("leg_timeout", 0.0)          # sim s per leg; 0: no limit
        # False for mapping tests: route_completed instead of nav2_succeeded and
        # reached_goal, the final waypoint passed like the others, failed legs skipped.
        self.declare_parameter("navigation_checks", True)
        # Mapping. Empty reference_map: no map comparison.
        self.declare_parameter("reference_map", "")
        self.declare_parameter("map_match_distance", 0.15)  # m
        self.declare_parameter("map_min_precision", 0.85)
        self.declare_parameter("map_min_recall", 0.80)
        self.declare_parameter("map_min_coverage", 0.60)
        # World-frame [x_min, y_min, x_max, y_max] to score; all zero: everything.
        self.declare_parameter("map_region", [0.0, 0.0, 0.0, 0.0])

        flat = list(self.param("waypoints"))
        if len(flat) < 2 or len(flat) % 2:
            raise ValueError("waypoints must be a flat, even-length [x1, y1, x2, y2, ...]")
        self.waypoints = list(zip(flat[0::2], flat[1::2]))
        if self.param("reference_map"):
            self.kind = "mapping"

        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.bt_state = self.create_client(GetState, "bt_navigator/get_state")
        self.feedback = None
        self.latest_map = None
        self.create_subscription(OccupancyGrid, "/map", self.on_map, MAP_QOS)

    def on_map(self, msg):
        self.latest_map = msg

    def on_feedback(self, msg):
        self.feedback = msg.feedback

    def bt_navigator_active(self):
        if not self.bt_state.service_is_ready():
            return False
        future = self.bt_state.call_async(GetState.Request())
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return future.done() and future.result() is not None \
            and future.result().current_state.id == LIFECYCLE_ACTIVE

    # -- the test -------------------------------------------------------------

    def startup(self, deadline):
        super().startup(deadline)
        self.wait_for("navigate_to_pose action server", self.nav.server_is_ready, deadline)
        # bt_navigator is the last node Nav2's lifecycle manager activates, so
        # once it is active the costmaps are too.
        self.wait_for("bt_navigator active", self.bt_navigator_active, deadline)
        time.sleep(self.param("settle_time"))

    def execute(self, report):
        tol = self.param("tolerance")
        checks, metrics = report["checks"], report["metrics"]
        report["waypoints"] = [{"x": x, "y": y} for x, y in self.waypoints]

        report["navigation"] = nav = self.navigate()
        strict = self.param("navigation_checks")
        if strict:
            checks["nav2_succeeded"] = nav["status"] == "SUCCEEDED"
        else:
            checks["route_completed"] = nav["status"] in ("SUCCEEDED", "PASSED")
        metrics["time_to_goal_s"] = nav["time_sim"]
        metrics["path_length_m"] = nav["path_length"]
        metrics["recoveries"] = nav["number_of_recoveries"]

        # Let the filter settle after the stop.
        self.sleep_sim(1.0)
        final = self.final_localization(report)
        gx, gy = self.waypoints[-1]
        gt = final["ground_truth"]
        final["distance_to_goal"] = math.hypot(gt["x"] - gx, gt["y"] - gy)
        final["localized_distance_to_goal"] = math.hypot(
            final["localized"]["x"] - gx, final["localized"]["y"] - gy)
        if strict:
            checks["reached_goal"] = \
                final["distance_to_goal"] <= tol + self.param("nav_goal_tolerance")
        metrics["distance_to_goal_m"] = final["distance_to_goal"]

        if self.param("reference_map"):
            self.evaluate_map(report)

    def navigate(self):
        timeout = self.param("timeout")
        self.start_tracking()
        sim_start, wall_start = self.sim_now(), time.monotonic()
        strict = self.param("navigation_checks")
        legs, status, recoveries = [], None, 0
        prev = self.localized()[:2]
        last = len(self.waypoints) - 1
        for i, (x, y) in enumerate(self.waypoints):
            remaining = timeout - (self.sim_now() - sim_start)
            if remaining <= 0:
                status = status or "TIMEOUT"
                break
            if self.param("leg_timeout") > 0:
                remaining = min(remaining, self.param("leg_timeout"))
            yaw = math.atan2(y - prev[1], x - prev[0])
            pass_radius = 0.0 if (i == last and strict) else self.param("pass_radius")
            leg = self.drive_leg(x, y, yaw, remaining, wall_start, pass_radius)
            leg["waypoint"] = i
            legs.append(leg)
            recoveries += leg["number_of_recoveries"]
            prev = (x, y)
            if leg["status"] not in ("SUCCEEDED", "PASSED"):
                status = status or leg["status"]   # the first failure names the route
                if strict:
                    break
        if status is None:
            status = legs[-1]["status"]
        sim_time, wall_time = self.sim_now() - sim_start, time.monotonic() - wall_start
        path_length = self.stop_tracking()
        straight = sum(math.dist(a, b) for a, b in zip([(0.0, 0.0)] + self.waypoints[:-1],
                                                        self.waypoints))
        return {
            "status": status,
            "legs": legs,
            "time_sim": sim_time,
            "time_wall": wall_time,
            "real_time_factor": sim_time / wall_time if wall_time > 0 else None,
            "path_length": path_length,
            "straight_line_distance": straight,
            "number_of_recoveries": recoveries,
            "timeout": timeout,
        }

    def drive_leg(self, x, y, yaw, budget, route_wall_start, pass_radius=0.0):
        """One NavigateToPose. pass_radius > 0: done ("PASSED") once within it."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.param("map_frame")
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # bt_navigator can report active a little before it accepts goals; retry.
        handle = None
        for _ in range(10):
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            future = self.nav.send_goal_async(goal, feedback_callback=self.on_feedback)
            while not future.done():
                time.sleep(0.02)
            handle = future.result()
            if handle is not None and handle.accepted:
                break
            self.get_logger().warn("goal rejected, retrying")
            time.sleep(1.0)
        if handle is None or not handle.accepted:
            raise RuntimeError("Nav2 rejected the goal")

        self.feedback = None
        start = self.sim_now()
        self.get_logger().info(f"goal ({x:.2f}, {y:.2f}) sent")
        result = handle.get_result_async()
        outcome = None
        while not result.done():
            here = self.localized()
            if pass_radius > 0 and here is not None and \
                    math.hypot(here[0] - x, here[1] - y) <= pass_radius:
                outcome = "PASSED"
            # Limited in sim seconds; the wall guard catches a stalled sim.
            elif self.sim_now() - start > budget or \
                    time.monotonic() - route_wall_start > 3.0 * self.param("timeout") + 30.0:
                outcome = "TIMEOUT"
                self.get_logger().warn("timeout, cancelling goal")
            if outcome:
                cancel = handle.cancel_goal_async()
                end = time.monotonic() + 5.0
                while not (cancel.done() and result.done()) and time.monotonic() < end:
                    time.sleep(0.05)
                break
            time.sleep(0.02)
        code = result.result().status if result.done() and result.result() else \
            GoalStatus.STATUS_UNKNOWN
        # Nav2 may finish the goal itself in the same instant we pass it.
        if outcome == "PASSED" and code == GoalStatus.STATUS_SUCCEEDED:
            outcome = None
        fb = self.feedback
        return {
            "goal": {"x": x, "y": y, "yaw": yaw},
            "status": outcome or STATUS_NAMES.get(code, str(code)),
            "time_sim": self.sim_now() - start,
            "number_of_recoveries": fb.number_of_recoveries if fb else 0,
        }

    def evaluate_map(self, report):
        checks, metrics = report["checks"], report["metrics"]
        # RTAB-Map publishes on its own 1 Hz loop; give the last stretch a chance.
        self.sleep_sim(2.0)
        grid = self.latest_map
        if grid is None:
            raise RuntimeError("no /map received from RTAB-Map")
        reference = map_compare.load_map_yaml(
            os.path.expanduser(self.param("reference_map")))
        actual = map_compare.grid_from_msg(grid)
        spawn = (self.param("spawn_x"), self.param("spawn_y"), self.param("spawn_yaw"))
        region = list(self.param("map_region"))
        result = map_compare.compare(actual, reference, spawn,
                                     self.param("map_match_distance"),
                                     region=region if any(region) else None)

        out_dir = os.path.dirname(self.result_path())
        stem = os.path.splitext(os.path.basename(self.result_path()))[0]
        overlay = os.path.join(out_dir, f"{stem}_map_overlay.png")
        map_compare.save_png(result.pop("overlay"), overlay)
        actual_yaml = map_compare.save_map_yaml(actual, os.path.join(out_dir, f"{stem}_map"))
        report["mapping"] = {**result, "reference_map": self.param("reference_map"),
                             "actual_map": actual_yaml, "overlay_image": overlay}
        report.setdefault("images", []).append(overlay)

        checks["map_precision"] = result["precision"] >= self.param("map_min_precision")
        checks["map_recall"] = result["recall"] >= self.param("map_min_recall")
        checks["map_coverage"] = result["coverage"] >= self.param("map_min_coverage")
        metrics["map_precision"] = result["precision"]
        metrics["map_recall"] = result["recall"]
        metrics["map_coverage"] = result["coverage"]
        metrics["map_obstacle_error_m"] = result["obstacle_error_mean"]


def main():
    run_node(GoalBenchmark)


if __name__ == "__main__":
    main()
