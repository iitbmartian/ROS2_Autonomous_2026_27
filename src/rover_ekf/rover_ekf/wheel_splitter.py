import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# ==== WHEEL -> JOINT NAME MAPPING (Husarion Panther) ====
WHEEL_JOINT_MAP = {
    "wheel_fl": "fl_wheel_joint",
    "wheel_fr": "fr_wheel_joint",
    "wheel_rl": "rl_wheel_joint",
    "wheel_rr": "rr_wheel_joint",
}

JOINT_STATES_TOPIC = "/joint_states"

# joint_state_broadcaster on this Panther config only exports the position
# state interface (velocity[] and effort[] are always empty). Velocity is
# derived here via finite difference of position over time instead. These
# are continuous joints (unbounded revolute), so position accumulates
# without wrapping - no angle-unwrap logic needed.
#
# The gz JointStatePublisher runs at the physics step rate (1 kHz) unless
# throttled, so also cap the derived-velocity output rate here. 100 Hz is
# plenty for the EKF (it filters at 50 Hz) and keeps the 4 wheel Odometry
# streams from flooding DDS/CPU.
MIN_DT = 0.01  # seconds - min sample gap between derived-velocity outputs


def stamp_to_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class WheelJointSplitter(Node):

    def __init__(self):
        super().__init__('wheel_joint_splitter')

        self.publishers_by_wheel = {
            wheel: self.create_publisher(Float64, f"/{wheel}/encoder", 10)
            for wheel in WHEEL_JOINT_MAP
        }

        self.sub = self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self.joint_state_callback, 10
        )

        self._warned_missing = set()
        # Per-joint last known (position, stamp_seconds) for finite-differencing.
        self._last_position = {}
        self._last_stamp = {}

        self.get_logger().info(
            f"wheel_joint_splitter (position-diff mode) started: "
            f"{JOINT_STATES_TOPIC} -> "
            + ", ".join(f"/{w}/encoder" for w in WHEEL_JOINT_MAP)
        )

    def joint_state_callback(self, msg: JointState):
            name_to_index = {name: i for i, name in enumerate(msg.name)}
            now = stamp_to_seconds(msg.header.stamp)

            for wheel, joint_name in WHEEL_JOINT_MAP.items():
                idx = name_to_index.get(joint_name)

                if idx is None:
                    if joint_name not in self._warned_missing:
                        self.get_logger().warn(
                            f"Joint '{joint_name}' not found in {JOINT_STATES_TOPIC} "
                            f"(wheel '{wheel}')."
                        )
                        self._warned_missing.add(joint_name)
                    continue

                if idx >= len(msg.position):
                    continue

                position = msg.position[idx]

                last_position = self._last_position.get(joint_name)
                last_stamp = self._last_stamp.get(joint_name)

                # First sample initialization
                if last_position is None or last_stamp is None:
                    self._last_position[joint_name] = position
                    self._last_stamp[joint_name] = now
                    continue

                dt = now - last_stamp
                
                # Handle out-of-order timestamps or clock resets
                if dt <= 0:
                    continue

                # Throttle output rate: skip until enough time has accumulated
                if dt < MIN_DT:
                    continue

                # Calculate finite difference
                velocity = (position - last_position) / dt

                # Publish
                out = Float64()
                out.data = velocity
                self.publishers_by_wheel[wheel].publish(out)

                # ONLY update reference state when a window has successfully completed
                self._last_position[joint_name] = position
                self._last_stamp[joint_name] = now


def main(args=None):
    rclpy.init(args=args)
    node = WheelJointSplitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()