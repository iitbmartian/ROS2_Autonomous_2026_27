import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from rover_ekf_msgs.msg import WheelNoiseMode
import random

# ==== MANUAL NOISE TUNING ====
# Edit these constants directly for quick testing, or override at runtime:
#   ros2 run odomshi wheel_rr_relay --ros-args -p noise_stddev:=0.05
NOISE_MEAN = 0.0
WHEEL_RADIUS = 0.1825   # meters - update to match rover.urdf
NOISE_STDDEV = 0.5/WHEEL_RADIUS  # rad/s of injected Gaussian noise on the raw encoder reading
WHEEL_NAME = "wheel_rr"
RAW_TOPIC = "/wheel_rr/encoder"
RELAYED_TOPIC = "/wheel_rr/odom_relayed"
NOISE_MODE_TOPIC = "/wheel_rr/noise_mode"

# Output clamp applied to the published linear velocity while in pure-noise mode.
NOISY_MODE_MIN_VEL = -2.0
NOISY_MODE_MAX_VEL = 2.0


class WheelRRRelay(Node):

    def __init__(self):
        super().__init__('wheel_rr_relay')

        self.declare_parameter('noise_mean', NOISE_MEAN)
        self.declare_parameter('noise_stddev', NOISE_STDDEV)
        self.declare_parameter('wheel_radius', WHEEL_RADIUS)

        # Pure-noise mode state, set via NOISE_MODE_TOPIC. When active, the
        # real encoder reading is ignored and the relay instead publishes
        # Gaussian noise sampled from noisy_mean/noisy_stddev.
        self.noisy_mode = False
        self.noisy_mean = 0.0
        self.noisy_stddev = 0.0

        self.sub = self.create_subscription(
            Float64, RAW_TOPIC, self.encoder_callback, 10
        )
        self.noise_mode_sub = self.create_subscription(
            WheelNoiseMode, NOISE_MODE_TOPIC, self.noise_mode_callback, 10
        )
        self.pub = self.create_publisher(Odometry, RELAYED_TOPIC, 10)

        # Twist covariance injected into the outgoing Odometry message.
        # [vx, vy, vz, vroll, vpitch, vyaw] x 6, row-major
        self.twist_cov = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.05, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.05, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.05]

        self.get_logger().info(
            f"{WHEEL_NAME} relay started: {RAW_TOPIC} -> {RELAYED_TOPIC} "
            f"(noise stddev={NOISE_STDDEV}); noise mode control on {NOISE_MODE_TOPIC}"
        )

    def noise_mode_callback(self, msg: WheelNoiseMode):
        self.noisy_mode = msg.noisy_mode
        self.noisy_mean = msg.mean
        self.noisy_stddev = msg.stddev
        self.get_logger().info(
            f"{WHEEL_NAME}: noisy_mode={'ON' if self.noisy_mode else 'OFF'} "
            f"(mean={self.noisy_mean}, stddev={self.noisy_stddev})"
        )

    def encoder_callback(self, msg: Float64):
        if self.noisy_mode:
            linear_vel = random.gauss(self.noisy_mean, self.noisy_stddev)
            linear_vel = max(NOISY_MODE_MIN_VEL, min(NOISY_MODE_MAX_VEL, linear_vel))
        else:
            mean = self.get_parameter('noise_mean').value
            stddev = self.get_parameter('noise_stddev').value
            radius = self.get_parameter('wheel_radius').value

            # Raw encoder value is assumed to be angular velocity (rad/s).
            noisy_angular_vel = msg.data + random.gauss(mean, stddev)
            linear_vel = noisy_angular_vel * radius

        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "wheel_6"
        out.child_frame_id = "base_link"  # must match ekf base_link_frame - twist needs a valid TF and base_link is always identity

        # Only the forward wheel-surface speed is populated; EKF config
        # only fuses vx for this source.
        out.twist.twist.linear.x = linear_vel
        out.twist.covariance = self.twist_cov

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = WheelRRRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
