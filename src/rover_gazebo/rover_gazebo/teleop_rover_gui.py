#!/usr/bin/python3
"""Keyboard driving for the rover, windowed version.

Same keys and modes as teleop_rover.py -- see its docstring for the full mode
explanation. W/S forward-reverse, A/D left-right (meaning depends on mode), C sweep
to straight (explicit/explicit_pid only), Space stop, Shift boost, 1/2/3/4/5 pick the
mode, Q or Escape quit.

The only real difference from teleop_rover.py is how keys are read: this window gets
real, independent press and release events per key from the OS (X11/Wayland), rather
than raw terminal bytes. A raw terminal only reliably auto-repeats one held key at a
time and never sends a release event at all, so holding two keys together (e.g. W+A
to drive an arc) can silently drop one of them. This window doesn't have that
problem -- W and A each stay genuinely held for exactly as long as each is
physically down.

Click into this window once to give it keyboard focus (it tries to take focus on
its own when it opens, but some window managers require the click anyway), then
drive. Publishes geometry_msgs/Twist on cmd_vel and std_msgs/String on
rover/steer_mode, and subscribes to std_msgs/Float64 on rover/steer_aim -- the same
topics teleop_rover.py uses, and the exact same driving logic
(teleop_common.TeleopCore), so the two are interchangeable from anywhere else's
point of view; run either, or both.
"""

import math
import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from teleop_common import MODE_KEYS, TeleopCore

DRIVE_KEYS = ("w", "a", "s", "d")

# Fallback if key_release_debounce_ms isn't reachable yet (see __init__). Raised again
# from an earlier 150 ms: still not generous enough on a loaded machine, where Tk's
# after() timers -- and the real KeyPress/KeyRelease events they're racing against --
# can both be delayed by the same scheduling pressure, not just the raw OS repeat rate
# this was originally sized against. 250 ms is a deliberately large margin; see
# key_release_debounce_ms in config/rover_control.yaml to tune it down once it's
# confirmed reliable on a given machine.
DEFAULT_RELEASE_DEBOUNCE_MS = 250

MODE_LABELS = {"ackermann": "Ackermann", "crab": "Crab", "spot": "Spot",
               "explicit": "Explicit", "explicit_pid": "Explicit (PID)"}

LEGEND = (
    "W/S forward-reverse    A/D left-right (mode-dependent)\n"
    "C sweep to straight (explicit/PID)    Space stop    Shift boost\n"
    "1 Ackermann  2 Crab  3 Spot  4 Explicit  5 Explicit-PID   Q / Esc quit"
)


class TeleopGUI(TeleopCore):

    def __init__(self):
        super().__init__("teleop_rover_gui")
        self.declare_parameter("key_release_debounce_ms", DEFAULT_RELEASE_DEBOUNCE_MS)
        self.keys = {k: False for k in DRIVE_KEYS}
        self.keys["shift"] = False
        self._release_timers = {}   # key -> root.after() id, a pending "maybe released"
        self._suppressed = set()    # keys force-stopped (Space/mode switch), ignored
                                     # until a real release is actually observed
        self._quitting = False
        self._tick_id = None

        self.root = tk.Tk()
        self.root.title("Rover teleop")
        self.root.geometry("420x280")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self._quit)

        self._build_ui()

        self.root.bind("<KeyPress>", self._on_press)
        self.root.bind("<KeyRelease>", self._on_release)
        self.root.bind("<FocusOut>", self._on_focus_out)
        self.root.bind("<FocusIn>", self._on_focus_in)
        self.has_focus = False
        self.root.focus_force()

        self._tick_ms = max(1, int(self.dt * 1000))
        self._tick_id = self.root.after(self._tick_ms, self._tick)

    # -- UI -------------------------------------------------------------------
    def _build_ui(self):
        pad = {"padx": 12, "pady": 4}

        self.mode_var = tk.StringVar(value=MODE_LABELS.get(self.mode, self.mode))
        tk.Label(self.root, textvariable=self.mode_var,
                 font=("TkDefaultFont", 16, "bold")).pack(anchor="w", **pad)

        keys_frame = tk.Frame(self.root)
        keys_frame.pack(anchor="w", padx=12, pady=6)
        self.key_labels = {}
        for k in ("w", "a", "s", "d", "shift"):
            text = "SHIFT" if k == "shift" else k.upper()
            lbl = tk.Label(keys_frame, text=text, font=("TkFixedFont", 12, "bold"),
                            width=6 if k != "shift" else 7, relief="groove",
                            fg="#aaaaaa")
            lbl.pack(side="left", padx=3)
            self.key_labels[k] = lbl

        self.aim_var = tk.StringVar(value="")
        tk.Label(self.root, textvariable=self.aim_var,
                 font=("TkFixedFont", 12)).pack(anchor="w", **pad)

        tk.Label(self.root, text=LEGEND, justify="left",
                 font=("TkDefaultFont", 9), fg="#555555").pack(anchor="w", **pad)

        self.focus_var = tk.StringVar(value="click here, then drive with the keyboard")
        self.focus_label = tk.Label(
            self.root, textvariable=self.focus_var,
            font=("TkDefaultFont", 9, "italic"), fg="#888888")
        self.focus_label.pack(anchor="w", side="bottom", padx=12, pady=8)

    def _refresh_ui(self):
        self.mode_var.set(MODE_LABELS.get(self.mode, self.mode))
        for k, lbl in self.key_labels.items():
            lbl.configure(fg="#000000" if self.keys[k] else "#aaaaaa")
        if self.mode in ("explicit", "explicit_pid"):
            self.aim_var.set(f"aim: {math.degrees(self.aim):+4.0f} deg")
        else:
            self.aim_var.set("")

    # -- key state, with autorepeat debouncing and a real e-stop --------------
    def _mark(self, key, down):
        timer = self._release_timers.pop(key, None)
        if timer is not None:
            self.root.after_cancel(timer)
        if down and key in self._suppressed:
            return   # force-stopped (Space/mode switch); ignore autorepeat presses
                      # until a real release is actually observed
        self.keys[key] = down

    def _schedule_release(self, key):
        debounce_ms = self.get_parameter("key_release_debounce_ms").value
        self._release_timers[key] = self.root.after(
            debounce_ms, self._confirm_release, key)

    def _confirm_release(self, key):
        self._release_timers.pop(key, None)
        self.keys[key] = False
        self._suppressed.discard(key)

    def _clear_drive_keys(self):
        """Space and mode switches: stop driving now, and don't let a key that's
        still physically down (and so still autorepeating) silently resume it --
        only a genuine release-then-press does."""
        for k in DRIVE_KEYS:
            self.keys[k] = False
            self._suppressed.add(k)
            timer = self._release_timers.pop(k, None)
            if timer is not None:
                self.root.after_cancel(timer)

    # -- Tk event handlers ------------------------------------------------------
    def _on_press(self, event):
        sym = event.keysym
        low = sym.lower()
        if low in ("shift_l", "shift_r"):
            self._mark("shift", True)
            return
        if low in DRIVE_KEYS:
            if low in ("a", "d"):
                self.recentre = False   # aiming by hand overrides the sweep home
            self._mark(low, True)
            return
        if sym in MODE_KEYS:
            self._clear_drive_keys()
            self.set_mode(MODE_KEYS[sym])
            return
        if low == "c":
            self.recentre_on()
            return
        if low == "space":
            self._clear_drive_keys()
            self.stop()
            return
        if low in ("q", "escape"):
            self._quit()

    def _on_release(self, event):
        sym = event.keysym
        low = sym.lower()
        if low in ("shift_l", "shift_r"):
            self._schedule_release("shift")
        elif low in DRIVE_KEYS:
            self._schedule_release(low)

    def _on_focus_out(self, event):
        # Only the window itself losing focus (not a click moving between two of our
        # own child widgets, which Tk also reports via FocusOut/FocusIn on each one --
        # both events carry the widget, so compare against the toplevel).
        if event.widget is not self.root:
            return
        self.has_focus = False
        # Once focus is gone we can no longer trust a release event to ever arrive for
        # whatever's currently held -- a key let go while looking at, say, the Gazebo
        # window would otherwise leave this window's state stuck on. Treat losing focus
        # as an immediate, full release of everything, the same as Space, rather than
        # let held state silently go stale.
        for timer in self._release_timers.values():
            self.root.after_cancel(timer)
        self._release_timers.clear()
        self._suppressed.clear()
        for k in self.keys:
            self.keys[k] = False
        self.focus_var.set("NOT FOCUSED -- click here to drive")
        self.focus_label.configure(fg="#cc4444")

    def _on_focus_in(self, event):
        if event.widget is not self.root:
            return
        self.has_focus = True
        self.focus_var.set("click here, then drive with the keyboard")
        self.focus_label.configure(fg="#888888")

    # -- main loop --------------------------------------------------------------
    def _tick(self):
        # One non-blocking spin is enough to drain the rover/steer_aim subscription in
        # the common case; this deliberately does not loop to drain a larger backlog,
        # because every extra spin_once() call is time the single-threaded Tk event
        # loop cannot spend processing a real, already-arrived KeyPress/KeyRelease --
        # under load that delay is exactly what let a genuine key event sit unprocessed
        # long enough to look like a drop.
        rclpy.spin_once(self, timeout_sec=0)

        fwd = (1.0 if self.keys["w"] else 0.0) - (1.0 if self.keys["s"] else 0.0)
        turn = (1.0 if self.keys["a"] else 0.0) - (1.0 if self.keys["d"] else 0.0)
        self.drive(fwd, turn, self.keys["shift"])
        self._refresh_ui()

        self._tick_id = self.root.after(self._tick_ms, self._tick)

    def _quit(self):
        if self._quitting:
            return
        self._quitting = True
        if self._tick_id is not None:
            self.root.after_cancel(self._tick_id)
            self._tick_id = None
        self.cmd.publish(Twist())
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            self.root.destroy()
        except tk.TclError:
            pass


def main():
    rclpy.init()
    node = TeleopGUI()
    node.mode_pub.publish(String(data="ackermann"))
    try:
        node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node._quit()


if __name__ == "__main__":
    main()
