import math
import os

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node

# ==== WHEEL -> STATION MAPPING (rover_gazebo four-wheel-steer rover) ====
# Same mapping as steer_wheel_splitter. Station positions (x forward, y left,
# from the rover centre) come from rover_gazebo's config/rover_kinematics.yaml.
WHEEL_STATION_MAP = {
    "wheel_fl": "FL",
    "wheel_fr": "FR",
    "wheel_rl": "BL",
    "wheel_rr": "BR",
}

# A wheel sample older than this (sim seconds) relative to the newest one is
# treated as missing, and no body velocity is published until it is fresh again.
MAX_SAMPLE_AGE = 0.1


class WheelBodyVelocity(Node):
    """Fit one rigid-body velocity (vx, vy, yaw rate) to the four wheel velocities.

    A single wheel cannot tell a turn from a translation, so on its own the EKF
    never sees the rover yaw. Four wheels at known stations can: every contact
    point of a rigid body moves at

        vx_i = vx - w * y_i
        vy_i = vy + w * x_i

    Stacking the four wheels gives eight equations in three unknowns, solved here
    by least squares. Input is the per-wheel /wheel_xx/odom_relayed stream, so
    whatever noise those relays inject carries through to the yaw rate. Output
    goes on /wheel_odom/body; ekf_rover_gazebo.yaml fuses only its yaw rate,
    since vx and vy already come in per wheel.

    Blind to lateral wheel slip: a wheel dragged sideways reports only its
    rolling speed, so yaw the wheels cause by scrubbing (explicit mode with a
    large aim, for instance) is under-reported.
    """

    def __init__(self):
        super().__init__('wheel_body_velocity')

        default_cfg = os.path.join(
            get_package_share_directory('rover_gazebo'), 'config', 'rover_kinematics.yaml')
        self.declare_parameter('kinematics_file', default_cfg)
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('child_frame_id', 'base_footprint')

        cfg_path = self.get_parameter('kinematics_file').value
        with open(cfg_path) as fh:
            wheels_cfg = yaml.safe_load(fh)['rover']['wheels']

        self.wheel_names = list(WHEEL_STATION_MAP)
        stations = [(wheels_cfg[s]['x'], wheels_cfg[s]['y'])
                    for s in WHEEL_STATION_MAP.values()]

        # Design matrix A of the 8 x 3 system A @ [vx, vy, w] = measured wheel velocities.
        rows = []
        for x, y in stations:
            rows.append([1.0, 0.0, -y])
            rows.append([0.0, 1.0, x])
        a = np.array(rows)
        self.pinv = np.linalg.pinv(a)
        # Parameter covariance per unit wheel-velocity variance: (A^T A)^-1.
        self.unit_cov = np.linalg.inv(a.T @ a)

        self.latest = {}  # wheel -> Odometry
        for wheel in self.wheel_names:
            self.create_subscription(
                Odometry, f"/{wheel}/odom_relayed",
                lambda msg, w=wheel: self.latest.__setitem__(w, msg), 10)

        self.pub = self.create_publisher(Odometry, '/wheel_odom/body', 10)
        self.create_timer(1.0 / self.get_parameter('publish_rate').value, self.step)

        self.get_logger().info(
            f"wheel_body_velocity started ({os.path.basename(cfg_path)}): "
            + ", ".join(f"/{w}/odom_relayed" for w in self.wheel_names)
            + " -> /wheel_odom/body")

    def step(self):
        if len(self.latest) < len(self.wheel_names):
            return
        msgs = [self.latest[w] for w in self.wheel_names]
        stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in msgs]
        newest = max(stamps)
        if newest - min(stamps) > MAX_SAMPLE_AGE:
            return

        meas = []
        variances = []
        for m in msgs:
            meas.append(m.twist.twist.linear.x)
            meas.append(m.twist.twist.linear.y)
            variances.append(m.twist.covariance[0])
            variances.append(m.twist.covariance[7])
        vx, vy, w = self.pinv @ np.array(meas)
        if not all(math.isfinite(v) for v in (vx, vy, w)):
            return

        # Wheels report the same variance, so the fit covariance is that times (A^T A)^-1.
        cov = self.unit_cov * max(variances)

        out = Odometry()
        out.header.stamp = msgs[stamps.index(newest)].header.stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = self.get_parameter('child_frame_id').value
        out.twist.twist.linear.x = float(vx)
        out.twist.twist.linear.y = float(vy)
        out.twist.twist.angular.z = float(w)
        tc = [0.0] * 36
        tc[0], tc[7], tc[35] = float(cov[0, 0]), float(cov[1, 1]), float(cov[2, 2])
        tc[14] = tc[21] = tc[28] = 1e3  # vz, roll rate, pitch rate: not observed
        out.twist.covariance = tc
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = WheelBodyVelocity()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
