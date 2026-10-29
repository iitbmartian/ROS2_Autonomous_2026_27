"""Give the rover a fixed drive command and check what the body and the localization did.

No planner: the test publishes a constant forward TwistStamped on /cmd_vel (what
rover_kinematics_node listens to, in its default ackermann mode) for
distance / speed sim seconds, then stops. It is the open-loop "fixed input,
expected output" benchmark, and the way to measure the vehicle and the estimator
on terrain the planner may refuse, slopes in particular (Nav2 and RTAB-Map
classify by height, so a ramp past 0.15 m reads as a wall to them).

Run it with the pipeline's nav2:=false, or Nav2's collision monitor would also be
writing /cmd_vel.

Sampled at 10 Hz while driving: the localization error against ground truth, and
the chassis roll and pitch from ground truth.

Checks (tolerance in m, from the UI):

    initial_localization, initial_at_spawn   as in every benchmark
    progress           ground-truth distance covered >= min_progress * distance.
                       Short of it: wheel slip, or stalled on the slope.
    climbed            ground-truth height gained >= expected_climb - climb_margin
                       (only when expected_climb > 0)
    stable             max |roll| and |pitch| stayed under max_tilt_deg
    localization_during_drive   max localization error while driving <= tolerance
    final_localization |localized - ground truth| <= tolerance, at the end
"""

import math
import time

from geometry_msgs.msg import TwistStamped

from rover_testing.benchmark_common import BenchmarkNode, run_node

RATE = 20.0          # Hz, command rate; rover_kinematics times commands out at 0.5 s
SAMPLE_EVERY = 2     # command ticks per localization sample, i.e. 10 Hz


class DriveBenchmark(BenchmarkNode):
    kind = "drive"

    def __init__(self):
        super().__init__("drive_benchmark")
        self.declare_parameter("speed", 0.5)            # m/s, forward
        self.declare_parameter("distance", 5.0)         # m, commanded
        self.declare_parameter("min_progress", 0.85)    # share of distance actually covered
        self.declare_parameter("expected_climb", 0.0)   # m of height gained
        self.declare_parameter("climb_margin", 0.1)     # m
        self.declare_parameter("max_tilt_deg", 35.0)
        self.cmd = self.create_publisher(TwistStamped, "/cmd_vel", 10)

    def startup(self, deadline):
        super().startup(deadline)
        self.wait_for("rover_kinematics on /cmd_vel",
                      lambda: self.cmd.get_subscription_count() > 0, deadline)
        # Let the suspension settle on the ground before measuring anything.
        self.sleep_sim(2.0)

    def publish(self, speed):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.twist.linear.x = speed
        self.cmd.publish(msg)

    def execute(self, report):
        tol = self.param("tolerance")
        speed, distance = self.param("speed"), self.param("distance")
        checks, metrics = report["checks"], report["metrics"]
        start_gt = self.ground_truth()

        # rover_kinematics ramps speed up and down symmetrically, so the distance
        # lost accelerating is made up coasting to a stop: on flat ground the
        # rover covers about the commanded distance (measured: 100-105%).
        duration = distance / speed
        if duration > self.param("timeout"):
            raise ValueError(f"distance / speed is {duration:.0f} s, over the "
                             f"{self.param('timeout'):.0f} s timeout")
        errors, max_roll, max_pitch, samples = [], 0.0, 0.0, []
        self.start_tracking()
        sim_start, wall_start = self.sim_now(), time.monotonic()
        tick = 0
        while self.sim_now() - sim_start < duration:
            if time.monotonic() - wall_start > 3.0 * duration + 30.0:
                raise TimeoutError("simulation stalled while driving")
            self.publish(speed)
            if tick % SAMPLE_EVERY == 0:
                snap = self.snapshot()
                gt = snap["ground_truth"]
                errors.append(snap["position_error"])
                max_roll = max(max_roll, abs(gt["roll"]))
                max_pitch = max(max_pitch, abs(gt["pitch"]))
                samples.append({"t": self.sim_now() - sim_start, "x": gt["x"], "y": gt["y"],
                                "z": gt["z"], "pitch_deg": math.degrees(gt["pitch"]),
                                "loc_error": snap["position_error"]})
            tick += 1
            time.sleep(1.0 / RATE)
        drive_time = self.sim_now() - sim_start
        for _ in range(int(RATE)):     # one second of explicit stop
            self.publish(0.0)
            time.sleep(1.0 / RATE)
        self.sleep_sim(1.0)
        path_length = self.stop_tracking()

        final = self.final_localization(report)
        end_gt = self.ground_truth()
        covered = math.hypot(end_gt[0] - start_gt[0], end_gt[1] - start_gt[1])
        climb = end_gt[2] - start_gt[2]
        loc_climb = final["localized"]["z"] - report["initial"]["localized"]["z"]

        report["drive"] = {
            "speed": speed,
            "commanded_distance": distance,
            "drive_time_sim": drive_time,
            "distance_covered": covered,
            "path_length": path_length,
            "progress_ratio": covered / distance,
            "average_speed": covered / drive_time if drive_time > 0 else None,
            "climb": climb,
            "localized_climb": loc_climb,
            "max_roll_deg": math.degrees(max_roll),
            "max_pitch_deg": math.degrees(max_pitch),
            "max_loc_error": max(errors) if errors else None,
            "mean_loc_error": sum(errors) / len(errors) if errors else None,
            "samples": samples,
        }
        d = report["drive"]

        checks["progress"] = d["progress_ratio"] >= self.param("min_progress")
        if self.param("expected_climb") > 0.0:
            checks["climbed"] = climb >= self.param("expected_climb") - self.param("climb_margin")
        checks["stable"] = max(d["max_roll_deg"], d["max_pitch_deg"]) < self.param("max_tilt_deg")
        checks["localization_during_drive"] = d["max_loc_error"] is not None \
            and d["max_loc_error"] <= tol

        metrics["drive_time_s"] = drive_time
        metrics["progress_ratio"] = d["progress_ratio"]
        metrics["climb_m"] = climb
        metrics["max_pitch_deg"] = d["max_pitch_deg"]
        metrics["max_loc_error_m"] = d["max_loc_error"]


def main():
    run_node(DriveBenchmark)


if __name__ == "__main__":
    main()
