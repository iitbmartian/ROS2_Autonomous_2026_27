"""Shared cmd_vel-driving logic for the rover's teleop frontends.

Both teleop_rover.py (raw terminal) and teleop_rover_gui.py (Tkinter window) subclass
TeleopCore from here. Everything that is *not* specific to how a frontend reads the
keyboard lives in this one place -- the mode/aim/recentre state, the publishers and the
rover/steer_aim subscription, and drive(), which is the exact mode-branching logic that
used to live directly in teleop_rover.py's step(). A frontend's only job is to turn
whatever it reads (a raw terminal byte stream, or real Tk key press/release events) into
a (fwd, turn, boost) snapshot once per tick and call drive() with it, so the driving math
itself can never drift between the two.

fwd and turn are each in [-1, 1] (forward-minus-back, left-minus-right, as the W/A/S/D
keys give it); boost is whether the frontend considers Shift down for this tick.
"""

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float64, String

# Key character -> mode name, shared so both frontends map 1/2/3/4/5 identically.
MODE_KEYS = {"1": "ackermann", "2": "crab", "3": "spot", "4": "explicit",
             "5": "explicit_pid"}

# rover/steer_mode is a "what's the current state" topic, published only on change,
# not continuously -- so it needs TRANSIENT_LOCAL durability (latching) on both this
# publisher and every subscriber (rover_kinematics_node.py, control_node.py), or a
# subscriber that finishes DDS discovery after the last mode change simply never
# sees it and silently stays on whatever mode it started in. Plain default (VOLATILE)
# QoS only delivers messages published *after* a subscriber has connected -- with a
# topic like this one that idles for long stretches between publishes, a subscriber
# that comes up mid-idle (control_node.py in particular, which loads a YAML file at
# startup and so connects slightly later than the rest) can easily lose the race and
# never receive the very first mode it should have started in. Measured live: this
# is exactly what made explicit_pid unresponsive on a fresh launch until switching
# away and back. A publisher and a subscriber's QoS need to be compatible for the
# match to happen at all -- every publisher to this topic (measure_rover.py's own
# mode publisher too) must use this same QoS, not just the subscribers.
MODE_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class TeleopCore(Node):

    def __init__(self, node_name="teleop_rover"):
        super().__init__(node_name)
        self.declare_parameter("max_speed", 1.0)
        self.declare_parameter("max_yaw_rate", 1.0)       # rad/s, Ackermann
        self.declare_parameter("max_spin_rate", 0.7854)   # rad/s, spot
        self.declare_parameter("max_steer_rate", 0.2618)  # rad/s, ~15 deg/s, explicit A/D
        self.declare_parameter("boost", 2.5)
        self.declare_parameter("hold_time", 0.4)          # s, terminal frontend only
        self.declare_parameter("update_rate", 50.0)

        self.cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.mode_pub = self.create_publisher(String, "rover/steer_mode", MODE_QOS)
        self.create_subscription(Float64, "rover/steer_aim", self._on_aim, 10)

        self.mode = "ackermann"
        self.aim = 0.0           # rad, echoed back by the kinematics node
        self.recentre = False    # sweeping the aim to zero (explicit/explicit_pid)
        self.dt = 1.0 / self.get_parameter("update_rate").value

    def _on_aim(self, msg: Float64):
        self.aim = msg.data

    # -- actions, called by a frontend in response to its own key handling ----------
    def set_mode(self, name):
        """Switch driving mode and announce it. `name` must already be a valid mode
        (e.g. via MODE_KEYS) -- this mirrors rover_kinematics_node's own state, it does
        not validate against it."""
        self.mode = name
        self.recentre = False
        self.mode_pub.publish(String(data=self.mode))

    def stop(self):
        """Space: stop driving and cancel any recentre sweep in progress. A frontend is
        responsible for also clearing its own held-key state."""
        self.recentre = False

    def recentre_on(self):
        """C, explicit/explicit_pid modes only: start sweeping the aim back to zero."""
        self.recentre = True

    def drive(self, fwd, turn, boost_active):
        """Build and publish one cmd_vel tick from a (fwd, turn, boost) snapshot.

        This is the entire mode branch that used to live in teleop_rover.py's step() --
        moved here unchanged so both frontends drive identically. explicit mode's aim
        readout is a rendering concern, not a driving one, so it stays with each
        frontend: call report/update your own display right after this returns.
        """
        p = self.get_parameter
        boost = p("boost").value if boost_active else 1.0
        speed = p("max_speed").value * boost

        t = Twist()
        if self.mode == "crab":
            t.linear.x = fwd * speed
            t.linear.y = turn * speed
        elif self.mode == "spot":
            t.angular.z = turn * p("max_spin_rate").value * boost
        elif self.mode in ("explicit", "explicit_pid"):
            # Same movement either way -- explicit_pid only changes what happens
            # downstream, in rover_kinematics_node/control_node.py, not what a key
            # does here. Shift boosts W/S only, never the aiming rate, which stays
            # fixed so you can stop where you meant to.
            rate = p("max_steer_rate").value
            t.linear.x = fwd * p("max_speed").value * boost
            if self.recentre:
                # Saturates at the sweep rate while the aim is large and lands exactly
                # on zero as it closes, so it converges without hunting around straight.
                want = -self.aim / self.dt
                t.angular.z = max(-rate, min(rate, want))
                if abs(self.aim) < 0.005:
                    t.angular.z = 0.0
                    self.recentre = False
            else:
                t.angular.z = turn * rate
        else:
            t.linear.x = fwd * speed
            t.angular.z = turn * p("max_yaw_rate").value
        self.cmd.publish(t)
