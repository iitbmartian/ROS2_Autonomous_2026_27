"""What every benchmark node shares: startup waits, localized pose vs ground truth, the report.

Localized means TF map -> base_footprint, i.e. RTAB-Map's map -> odom composed with
the EKF's odom -> base_footprint. Ground truth is Gazebo's OdometryPublisher on
/odom, in the world frame. The map frame starts where the rover spawned, so ground
truth is moved into it with the spawn pose (spawn_x, spawn_y, spawn_yaw); heights
are reported relative to the ground-truth height at the start.

A subclass implements execute(report), filling report["checks"] (name -> bool),
report["metrics"] (flat name -> number, shown in the UI) and anything else it
likes. run() wraps it: the common startup, the initial localization checks,
exceptions turned into a failed report, the JSON written to result_file, and the
exit code (0 pass, 1 fail) that the test launch file shuts down on.
"""

import json
import math
import os
import sys
import threading
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

POSE_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")
# What has to be active before the rover moves at all: rover_sim's spawners bring
# these up in sequence after joint_state_broadcaster, and rover_kinematics_node
# subscribes to /cmd_vel well before they are done (measured: 7 s earlier with
# Nav2 left out). A command sent in between is silently dropped.
DRIVE_CONTROLLERS = ("steer_controller", "wheel_controller")


def rpy_of(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class BenchmarkNode(Node):
    kind = "benchmark"

    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter("test_name", name)
        self.declare_parameter("spawn_x", 0.0)            # m, world frame
        self.declare_parameter("spawn_y", 0.0)
        self.declare_parameter("spawn_yaw", 0.0)          # rad
        self.declare_parameter("tolerance", 0.3)          # m, localization error allowed
        self.declare_parameter("timeout", 120.0)          # sim s for the test's own motion
        self.declare_parameter("startup_timeout", 180.0)  # wall s for the stack to come up
        self.declare_parameter("result_file", "~/.ros/rover_testing/last_result.json")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lock = threading.Lock()
        self.gt = None              # latest ground truth, POSE_KEYS, map frame
        self.gt_z0 = None           # ground-truth height at the first message
        self.path_length = 0.0      # ground-truth distance while tracking
        self.tracking = False
        self.last_track = None
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.list_controllers = self.create_client(ListControllers,
                                                   "/controller_manager/list_controllers")

    def param(self, name):
        return self.get_parameter(name).value

    # -- inputs ---------------------------------------------------------------

    def on_odom(self, msg):
        sx, sy, syaw = self.param("spawn_x"), self.param("spawn_y"), self.param("spawn_yaw")
        p = msg.pose.pose.position
        dx, dy = p.x - sx, p.y - sy
        c, s = math.cos(-syaw), math.sin(-syaw)
        roll, pitch, yaw = rpy_of(msg.pose.pose.orientation)
        with self.lock:
            if self.gt_z0 is None:
                self.gt_z0 = p.z
            pose = (c * dx - s * dy, s * dx + c * dy, p.z - self.gt_z0,
                    roll, pitch, wrap(yaw - syaw))
            self.gt = pose
            if self.tracking:
                if self.last_track is not None:
                    self.path_length += math.dist(pose[:3], self.last_track[:3])
                self.last_track = pose

    def localized(self):
        """POSE_KEYS of base_frame in map_frame, or None if TF has no answer yet."""
        try:
            t = self.tf_buffer.lookup_transform(self.param("map_frame"),
                                                self.param("base_frame"), Time()).transform
        except Exception:  # noqa: BLE001 -- any TF failure just means "not yet"
            return None
        return (t.translation.x, t.translation.y, t.translation.z, *rpy_of(t.rotation))

    def ground_truth(self):
        with self.lock:
            return self.gt

    def start_tracking(self):
        with self.lock:
            self.tracking, self.path_length, self.last_track = True, 0.0, None

    def stop_tracking(self):
        with self.lock:
            self.tracking = False
            return self.path_length

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def sleep_sim(self, seconds):
        end = self.sim_now() + seconds
        wall_end = time.monotonic() + 3.0 * seconds + 5.0   # a stalled sim cannot hang us
        while self.sim_now() < end and time.monotonic() < wall_end:
            time.sleep(0.02)

    # -- helpers --------------------------------------------------------------

    def wait_for(self, what, predicate, deadline):
        self.get_logger().info(f"waiting for {what}")
        while time.monotonic() < deadline:
            if predicate():
                self.get_logger().info(f"{what}: ok")
                return
            time.sleep(0.1)
        raise TimeoutError(f"timed out waiting for {what}")

    def snapshot(self):
        """Localized pose against ground truth, right now."""
        localized, gt = self.localized(), self.ground_truth()
        if localized is None or gt is None:
            raise RuntimeError("lost the localized pose or ground truth")
        return {
            "localized": dict(zip(POSE_KEYS, localized)),
            "ground_truth": dict(zip(POSE_KEYS, gt)),
            "position_error": math.hypot(localized[0] - gt[0], localized[1] - gt[1]),
            "yaw_error": abs(wrap(localized[5] - gt[5])),
        }

    def controllers_active(self):
        if not self.list_controllers.service_is_ready():
            return False
        future = self.list_controllers.call_async(ListControllers.Request())
        end = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < end:
            time.sleep(0.05)
        if not future.done() or future.result() is None:
            return False
        states = {c.name: c.state for c in future.result().controller}
        return all(states.get(name) == "active" for name in DRIVE_CONTROLLERS)

    def wait_for_localization(self, deadline):
        self.wait_for("simulation clock", lambda: self.sim_now() > 0.0, deadline)
        self.wait_for("ground truth on /odom", lambda: self.ground_truth() is not None,
                      deadline)
        self.wait_for(f"TF {self.param('map_frame')} -> {self.param('base_frame')}",
                      lambda: self.localized() is not None, deadline)

    # -- the test -------------------------------------------------------------

    def startup(self, deadline):
        """Wait for whatever execute() needs. Subclasses extend."""
        self.wait_for_localization(deadline)
        self.wait_for("drive controllers active", self.controllers_active, deadline)

    def execute(self, report):
        raise NotImplementedError

    def run(self):
        tol = self.param("tolerance")
        report = {
            "test_name": self.param("test_name"),
            "kind": self.kind,
            "spawn": {"x": self.param("spawn_x"), "y": self.param("spawn_y"),
                      "yaw": self.param("spawn_yaw")},
            "tolerance": tol,
            "checks": {},
            "metrics": {},
            "passed": False,
            "failure_reason": "",
        }
        checks = report["checks"]
        wall_start = time.monotonic()
        try:
            self.startup(wall_start + self.param("startup_timeout"))
            report["metrics"]["startup_wall_s"] = time.monotonic() - wall_start

            initial = self.snapshot()
            initial["distance_from_spawn"] = math.hypot(initial["localized"]["x"],
                                                        initial["localized"]["y"])
            report["initial"] = initial
            checks["initial_localization"] = initial["position_error"] <= tol
            checks["initial_at_spawn"] = initial["distance_from_spawn"] <= tol
            report["metrics"]["initial_loc_error_m"] = initial["position_error"]

            self.execute(report)

            failed = [name for name, ok in checks.items() if not ok]
            report["passed"] = not failed
            if failed:
                report["failure_reason"] = "failed: " + ", ".join(failed)
        except Exception as e:  # noqa: BLE001 -- anything here is a failed test, reported
            report["passed"] = False
            report["failure_reason"] = f"{type(e).__name__}: {e}"
            self.get_logger().error(report["failure_reason"])

        self.write(report)
        verdict = "PASS" if report["passed"] else "FAIL"
        self.get_logger().info(f"{report['test_name']}: {verdict} {report['failure_reason']}")
        return report["passed"]

    def final_localization(self, report):
        """Final localized pose vs ground truth, into report["final"] and its check."""
        final = self.snapshot()
        report["final"] = final
        report["checks"]["final_localization"] = final["position_error"] <= self.param("tolerance")
        report["metrics"]["final_loc_error_m"] = final["position_error"]
        return final

    def result_path(self):
        return os.path.expanduser(self.param("result_file"))

    def write(self, report):
        path = self.result_path()
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w") as f:
            json.dump(report, f, indent=2)
        self.get_logger().info(f"result written to {path}")


def run_node(cls):
    rclpy.init()
    node = cls()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        passed = node.run()
    except KeyboardInterrupt:
        passed = False
    executor.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if passed else 1)
