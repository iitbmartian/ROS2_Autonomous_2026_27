import os

import rclpy
from rclpy.executors import ExternalShutdownException
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ==== WHEEL -> STATION MAPPING (rover_gazebo four-wheel-steer rover) ====
# Each wheel has a drive joint (spin) and a steer joint (knuckle angle). Joint
# names, senses and the wheel radius are read from rover_gazebo's
# config/rover_kinematics.yaml, the same file rover_kinematics_node drives from,
# so this node and the controller can never disagree about a sign.
WHEEL_STATION_MAP = {
    "wheel_fl": "FL",
    "wheel_fr": "FR",
    "wheel_rl": "BL",
    "wheel_rr": "BR",
}

JOINT_STATES_TOPIC = "/joint_states"

# The joint_state_broadcaster runs at the controller manager rate (1 kHz).
# Cap the per-wheel output rate: 100 Hz is plenty for the EKF (it filters at
# 50 Hz) and keeps the 4 wheel streams from flooding DDS/CPU.
MIN_DT = 0.01  # seconds - min sample gap between outputs


def stamp_to_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class SteerWheelSplitter(Node):
    """Split /joint_states into one steer-aware encoder stream per wheel.

    Output on /<wheel>/steer_encoder is a JointState with two entries, already
    converted to the body convention the rover_kinematics node commands in:

        name[0] = "drive"  velocity = wheel spin rate, rad/s, + rolls the rover forward
                           position = accumulated spin angle, rad, same sense
        name[1] = "steer"  position = wheel heading, rad, + points the wheel left
                           velocity = steering rate, rad/s, same sense

    The relay projects the drive speed along the steer angle, which is what
    lets explicit, crab and spot steering show up as lateral wheel velocity.
    """

    def __init__(self):
        super().__init__('steer_wheel_splitter')

        default_cfg = os.path.join(
            get_package_share_directory('rover_gazebo'), 'config', 'rover_kinematics.yaml')
        self.declare_parameter('kinematics_file', default_cfg)

        cfg_path = self.get_parameter('kinematics_file').value
        with open(cfg_path) as fh:
            wheels_cfg = yaml.safe_load(fh)['rover']['wheels']

        self.wheels = {}
        for wheel, station in WHEEL_STATION_MAP.items():
            st = wheels_cfg[station]
            self.wheels[wheel] = {
                'drive_joint': st['drive_joint'],
                'steer_joint': st['steer_joint'],
                'drive_sign': float(st['drive_sign']),
                'steer_sign': float(st['steer_sign']),
            }

        self.publishers_by_wheel = {
            wheel: self.create_publisher(JointState, f"/{wheel}/steer_encoder", 10)
            for wheel in self.wheels
        }

        self.sub = self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self.joint_state_callback, 10
        )

        self._warned_missing = set()
        # Per-joint last known (position, stamp_seconds), for the finite-difference
        # fallback used only when joint_states carries no velocity.
        self._last_position = {}
        self._last_stamp = {}
        self._last_publish = {}

        self.get_logger().info(
            f"steer_wheel_splitter started ({os.path.basename(cfg_path)}): "
            f"{JOINT_STATES_TOPIC} -> "
            + ", ".join(f"/{w}/steer_encoder" for w in self.wheels)
        )

    def _joint(self, msg, name_to_index, joint_name, wheel, now):
        """Return (position, velocity) for one joint, or None if unavailable."""
        idx = name_to_index.get(joint_name)
        if idx is None:
            if joint_name not in self._warned_missing:
                self.get_logger().warn(
                    f"Joint '{joint_name}' not found in {JOINT_STATES_TOPIC} "
                    f"(wheel '{wheel}')."
                )
                self._warned_missing.add(joint_name)
            return None
        if idx >= len(msg.position):
            return None

        position = msg.position[idx]

        if idx < len(msg.velocity):
            velocity = msg.velocity[idx]
        else:
            # Finite difference against the previous sample.
            last_position = self._last_position.get(joint_name)
            last_stamp = self._last_stamp.get(joint_name)
            dt = now - last_stamp if last_stamp is not None else 0.0
            velocity = (position - last_position) / dt if dt > 0.0 else 0.0

        self._last_position[joint_name] = position
        self._last_stamp[joint_name] = now
        return position, velocity

    def joint_state_callback(self, msg: JointState):
        name_to_index = {name: i for i, name in enumerate(msg.name)}
        now = stamp_to_seconds(msg.header.stamp)

        for wheel, w in self.wheels.items():
            last = self._last_publish.get(wheel)
            if last is not None:
                dt = now - last
                # Out-of-order stamps, clock resets, or throttling.
                if dt <= 0 or dt < MIN_DT:
                    continue

            drive = self._joint(msg, name_to_index, w['drive_joint'], wheel, now)
            steer = self._joint(msg, name_to_index, w['steer_joint'], wheel, now)
            if drive is None or steer is None:
                continue

            out = JointState()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = "base_footprint"
            out.name = ["drive", "steer"]
            out.position = [w['drive_sign'] * drive[0], w['steer_sign'] * steer[0]]
            out.velocity = [w['drive_sign'] * drive[1], w['steer_sign'] * steer[1]]
            self.publishers_by_wheel[wheel].publish(out)

            self._last_publish[wheel] = now


def main(args=None):
    rclpy.init(args=args)
    node = SteerWheelSplitter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
