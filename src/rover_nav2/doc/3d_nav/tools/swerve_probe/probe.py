#!/usr/bin/env python3
"""Open-loop swerve probe on flat ground: puts the kinematics node in swerve mode, plays fixed
twist phases on /cmd_vel and logs ground-truth attitude plus every joint to argv[1] (csv).
Finds whether the body roll seen in swerve runs comes from the suspension model, not navigation."""
import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String

PHASES = [('settle', 6, 0, 0, 0), ('spin_slow', 25, 0, 0, 0.35), ('rest1', 3, 0, 0, 0),
          ('diag_turn', 25, -0.10, 0.15, -0.35), ('rest2', 3, 0, 0, 0), ('crab', 20, 0, 0.25, 0),
          ('rest3', 3, 0, 0, 0), ('spin_fast', 15, 0, 0, 0.8), ('rest4', 3, 0, 0, 0),
          ('forward', 15, 0.4, 0, 0), ('end', 2, 0, 0, 0)]


class Probe(Node):
    def __init__(self, out):
        super().__init__('swerve_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        q = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE)
        self.mode = self.create_publisher(String, '/rover/steer_mode', q)
        self.cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.js = {}
        self.create_subscription(JointState, '/joint_states', self.on_js, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.f = open(out, 'w'); self.header = None
        self.t0 = None; self.k = 0; self.phase_start = None
        self.create_timer(0.05, self.tick)

    def on_js(self, m):
        for n, p in zip(m.name, m.position):
            self.js[n] = p

    def on_odom(self, m):
        if self.t0 is None or not self.js:
            return
        q = m.pose.pose.orientation; p = m.pose.pose.position
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        names = sorted(self.js)
        if self.header is None:
            self.header = names
            self.f.write('wall,t,phase,x,y,z,roll,pitch,yaw,' + ','.join(names) + '\n')
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        self.f.write(f'{time.time():.3f},{t:.3f},{PHASES[min(self.k, len(PHASES) - 1)][0]},{p.x:.3f},{p.y:.3f},{p.z:.4f},'
                     f'{roll:.4f},{pitch:.4f},{yaw:.4f},' + ','.join(f'{self.js.get(n, float("nan")):.4f}' for n in self.header) + '\n')

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0:
            return
        self.mode.publish(String(data='swerve'))
        if self.t0 is None:
            self.t0 = self.phase_start = now
        name, dur, vx, vy, wz = PHASES[self.k]
        if now - self.phase_start >= dur:
            self.k += 1; self.phase_start = now
            if self.k >= len(PHASES):
                self.f.close(); raise SystemExit
            name, dur, vx, vy, wz = PHASES[self.k]
        m = TwistStamped(); m.header.stamp = self.get_clock().now().to_msg(); m.header.frame_id = 'base_footprint'
        m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = float(vx), float(vy), float(wz)
        self.cmd.publish(m)


rclpy.init()
n = Probe(sys.argv[1])
try:
    rclpy.spin(n)
except SystemExit:
    pass
