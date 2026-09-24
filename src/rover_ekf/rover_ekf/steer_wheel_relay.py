import math
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rover_interfaces.msg import WheelNoiseMode
from sensor_msgs.msg import JointState

# ==== MANUAL NOISE TUNING ====
# Steer-aware counterpart of wheel_fl.py .. wheel_rr.py, for the rover_gazebo
# four-wheel-steer rover. One script serves all four wheels; pick the wheel and
# tune its noise per instance, e.g. in config/steer_wheels.yaml or:
#   ros2 run rover_ekf steer_wheel_relay --ros-args -p wheel:=wheel_fl -p noise_stddev:=0.05
NOISE_MEAN = 0.0
WHEEL_RADIUS = 0.14985   # meters - rover_gazebo config/rover_kinematics.yaml
NOISE_STDDEV = 0.0 / WHEEL_RADIUS  # rad/s of injected Gaussian noise on the raw encoder reading
STEER_NOISE_STDDEV = 0.0  # rad of injected Gaussian noise on the steer angle

# Output clamp applied to the published linear velocity while in pure-noise mode.
NOISY_MODE_MIN_VEL = -2.0
NOISY_MODE_MAX_VEL = 2.0


class SteerWheelRelay(Node):

    def __init__(self):
        super().__init__('steer_wheel_relay')

        self.declare_parameter('wheel', 'wheel_fl')
        self.declare_parameter('noise_mean', NOISE_MEAN)
        self.declare_parameter('noise_stddev', NOISE_STDDEV)
        self.declare_parameter('steer_noise_stddev', STEER_NOISE_STDDEV)
        self.declare_parameter('wheel_radius', WHEEL_RADIUS)
        # Must match the ekf base_link_frame. On this rover base_footprint is the
        # TF root robot_state_publisher publishes, so the EKF owns odom->base_footprint.
        self.declare_parameter('child_frame_id', 'base_footprint')

        self.wheel_name = self.get_parameter('wheel').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        raw_topic = f"/{self.wheel_name}/steer_encoder"
        relayed_topic = f"/{self.wheel_name}/odom_relayed"
        noise_mode_topic = f"/{self.wheel_name}/noise_mode"

        # Pure-noise mode state, set via noise_mode_topic. When active, the
        # real drive reading is ignored and the relay instead publishes
        # Gaussian noise sampled from noisy_mean/noisy_stddev as the wheel
        # speed, still projected along the measured steer angle.
        self.noisy_mode = False
        self.noisy_mean = 0.0
        self.noisy_stddev = 0.0

        self.sub = self.create_subscription(
            JointState, raw_topic, self.encoder_callback, 10
        )
        self.noise_mode_sub = self.create_subscription(
            WheelNoiseMode, noise_mode_topic, self.noise_mode_callback, 10
        )
        self.pub = self.create_publisher(Odometry, relayed_topic, 10)

        # Twist covariance injected into the outgoing Odometry message.
        # [vx, vy, vz, vroll, vpitch, vyaw] x 6, row-major
        self.twist_cov = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.05, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.05, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.05]

        self.get_logger().info(
            f"{self.wheel_name} steer relay started: {raw_topic} -> {relayed_topic} "
            f"(noise stddev={self.get_parameter('noise_stddev').value}); "
            f"noise mode control on {noise_mode_topic}"
        )

    def noise_mode_callback(self, msg: WheelNoiseMode):
        self.noisy_mode = msg.noisy_mode
        self.noisy_mean = msg.mean
        self.noisy_stddev = msg.stddev
        self.get_logger().info(
            f"{self.wheel_name}: noisy_mode={'ON' if self.noisy_mode else 'OFF'} "
            f"(mean={self.noisy_mean}, stddev={self.noisy_stddev})"
        )

    def encoder_callback(self, msg: JointState):
        # steer_wheel_splitter publishes name = ["drive", "steer"], already in body
        # convention: drive + rolls forward, steer + points the wheel left.
        if len(msg.velocity) < 1 or len(msg.position) < 2:
            return
        drive_rate = msg.velocity[0]
        steer_angle = msg.position[1]

        steer_angle += random.gauss(0.0, self.get_parameter('steer_noise_stddev').value)

        if self.noisy_mode:
            wheel_speed = random.gauss(self.noisy_mean, self.noisy_stddev)
            wheel_speed = max(NOISY_MODE_MIN_VEL, min(NOISY_MODE_MAX_VEL, wheel_speed))
        else:
            mean = self.get_parameter('noise_mean').value
            stddev = self.get_parameter('noise_stddev').value
            radius = self.get_parameter('wheel_radius').value

            # Drive encoder value is angular velocity (rad/s).
            noisy_angular_vel = drive_rate + random.gauss(mean, stddev)
            wheel_speed = noisy_angular_vel * radius

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "odom"
        out.child_frame_id = self.child_frame_id

        # The wheel's contact-patch velocity in the body frame: its rolling speed
        # along the direction the steering points it. With the wheels steered,
        # part of the motion is sideways, so vy is populated as well as vx.
        out.twist.twist.linear.x = wheel_speed * math.cos(steer_angle)
        out.twist.twist.linear.y = wheel_speed * math.sin(steer_angle)
        out.twist.covariance = self.twist_cov

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SteerWheelRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
