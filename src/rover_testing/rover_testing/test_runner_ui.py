"""Tk window that picks rover_testing test cases, runs them one by one, and shows the results.

    ros2 launch rover_testing run_tests.launch.py

The test cases come from config/test_cases.yaml, grouped by category (navigation,
drive, mapping). Run launches each selected one

    ros2 launch <launch/tests/file> tolerance:=... timeout:=... gui:=... result_file:=...

in its own process group, waits for it to finish, reads the JSON report its
benchmark node wrote, and adds a row to the results table. Tests run strictly one
after another: two Gazebo instances on one ROS graph would clash.

Between tests nothing may survive. Each test runs with its own ROVER_TESTING_RUN
marker (gz_reaper.py, which also gives it its own GZ_PARTITION); after it ends the
runner signals its process group, then SIGKILLs anything still carrying the marker,
which catches the Gazebo server the GUI mode forks into a group of its own. Before
a run starts, Gazebo servers left over from elsewhere are listed and you are
offered to kill them: their in-process /controller_manager would collide with the
test's on the ROS graph.

Each session's reports, launch logs, map overlays and a summary.json are kept under
~/.ros/rover_testing/<timestamp>/. Double-click a result row for the full report.
"""

import json
import os
import queue
import signal
import subprocess
import threading
import time
import tkinter as tk
import uuid
from datetime import datetime
from tkinter import messagebox, ttk

import yaml
from ament_index_python.packages import get_package_share_directory

from rover_testing.gz_reaper import MARKER, marked_processes

PKG = "rover_testing"
RESULTS_ROOT = os.path.expanduser("~/.ros/rover_testing")
# Wall seconds on top of the test's own timeout before it is killed outright: the
# stack's startup (the benchmarks' startup_timeout is 180) plus shutdown.
WALL_SLACK = 300.0
LOG_LINES = 4000
CATEGORIES = ("navigation", "drive", "mapping")

COLUMNS = (
    ("test", "Test", 180, "w"),
    ("kind", "Kind", 85, "center"),
    ("result", "Result", 65, "center"),
    ("duration", "Duration (sim s)", 115, "center"),
    ("startup", "Startup (s)", 85, "center"),
    ("init_err", "Initial loc. err (m)", 130, "center"),
    ("final_err", "Final loc. err (m)", 120, "center"),
    ("metrics", "Key metrics", 340, "w"),
    ("note", "Notes", 220, "w"),
)


def fmt(value, digits=3):
    return "-" if value is None else f"{value:.{digits}f}"


def key_metrics(report):
    """One line of the numbers that matter for this kind of test."""
    m = report.get("metrics", {})
    kind = report.get("kind")
    parts = []
    if kind == "drive":
        if "progress_ratio" in m:
            parts.append(f"progress {m['progress_ratio'] * 100:.0f}%")
        if "climb_m" in m:
            parts.append(f"climb {m['climb_m']:.2f} m")
        if "max_pitch_deg" in m:
            parts.append(f"max pitch {m['max_pitch_deg']:.1f}°")
        if m.get("max_loc_error_m") is not None:
            parts.append(f"max loc err {m['max_loc_error_m']:.3f} m")
    else:
        if kind == "mapping" and "map_precision" in m:
            parts.append(f"precision {m['map_precision']:.2f}")
            parts.append(f"recall {m['map_recall']:.2f}")
            parts.append(f"coverage {m['map_coverage'] * 100:.0f}%")
            if m.get("map_obstacle_error_m") is not None:
                parts.append(f"obst. err {m['map_obstacle_error_m']:.3f} m")
        if "path_length_m" in m:
            parts.append(f"path {m['path_length_m']:.2f} m")
        if "distance_to_goal_m" in m:
            parts.append(f"{m['distance_to_goal_m']:.2f} m from goal")
        if m.get("recoveries"):
            parts.append(f"{m['recoveries']} recoveries")
    return ", ".join(parts) or "-"


def gz_servers():
    """(pid, command) of every running `gz sim` process of this user."""
    found = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            if os.stat(f"/proc/{entry}").st_uid != os.getuid():
                continue
            with open(f"/proc/{entry}/cmdline", "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
        except OSError:
            continue
        if cmd.startswith("gz sim") or "/gz sim " in cmd:
            found.append((int(entry), cmd))
    return found


class Runner(threading.Thread):
    """Runs the selected tests in sequence, posting events to the UI's queue."""

    def __init__(self, tests, settings, events):
        super().__init__(daemon=True)
        self.tests, self.settings, self.events = tests, settings, events
        self.stop_requested = threading.Event()
        self.proc = None
        self.run_id = None
        self.session_dir = os.path.join(RESULTS_ROOT, datetime.now().strftime("%Y%m%d_%H%M%S"))

    def stop(self):
        self.stop_requested.set()
        self.teardown()

    def run(self):
        os.makedirs(self.session_dir, exist_ok=True)
        summary = {"settings": self.settings, "results": []}
        for i, test in enumerate(self.tests):
            if self.stop_requested.is_set():
                break
            self.events.put(("status", f"Running {test['name']} ({i + 1}/{len(self.tests)})"))
            report = self.run_one(test)
            summary["results"].append(report)
            self.events.put(("result", report))
        with open(os.path.join(self.session_dir, "summary.json"), "w") as f:
            json.dump(summary, f, indent=2)
        self.events.put(("done", self.session_dir))

    def run_one(self, test):
        name = test["name"]
        timeout = self.settings["timeout"] or float(test.get("timeout", 120.0))
        result_file = os.path.join(self.session_dir, f"{name}.json")
        log_file = os.path.join(self.session_dir, f"{name}.log")
        launch_file = os.path.join(get_package_share_directory(PKG), "launch", "tests",
                                   test["launch_file"])
        cmd = ["ros2", "launch", launch_file,
               f"tolerance:={self.settings['tolerance']}",
               f"timeout:={timeout}",
               f"gui:={'true' if self.settings['gui'] else 'false'}",
               f"result_file:={result_file}"]
        self.run_id = f"rover_testing_{name}_{uuid.uuid4().hex[:8]}"
        env = {**os.environ, MARKER: self.run_id, "GZ_PARTITION": self.run_id}
        self.events.put(("log", f"\n===== {name} =====\n$ {' '.join(cmd)}\n"))

        start = time.monotonic()
        killed = False
        with open(log_file, "w") as log:
            self.proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
                start_new_session=True, env=env)
            reader = threading.Thread(target=self.pump, args=(self.proc.stdout, log),
                                      daemon=True)
            reader.start()
            while self.proc.poll() is None:
                if self.stop_requested.is_set():
                    break
                if time.monotonic() - start > timeout * 3.0 + WALL_SLACK:
                    killed = True
                    self.events.put(("log", f"\n[runner] {name} exceeded its wall-clock "
                                            "limit, killing it\n"))
                    break
                time.sleep(0.2)
            exit_code = self.proc.poll()
            self.teardown()
            reader.join(timeout=5.0)
        self.proc = None

        try:
            with open(result_file) as f:
                report = json.load(f)
        except (OSError, json.JSONDecodeError):
            if self.stop_requested.is_set():
                reason = "stopped by user"
            elif killed:
                reason = "killed: exceeded wall-clock limit"
            else:
                reason = f"no report written (launch exit code {exit_code})"
            report = {"test_name": name, "kind": test.get("category", "?"), "passed": False,
                      "failure_reason": reason, "checks": {}, "metrics": {}}
        report["log_file"] = log_file
        report["result_file"] = result_file
        report["wall_time_s"] = time.monotonic() - start
        return report

    def pump(self, stream, log):
        for line in stream:
            log.write(line)
            self.events.put(("log", line))

    def teardown(self):
        """Stop the current test's launch and everything it started."""
        proc, run_id = self.proc, self.run_id
        if proc is not None:
            try:
                pgid = os.getpgid(proc.pid)
            except ProcessLookupError:
                pgid = proc.pid   # leader gone; its group may not be
            for sig, wait in ((signal.SIGINT, 15.0), (signal.SIGTERM, 5.0),
                              (signal.SIGKILL, 2.0)):
                try:
                    os.killpg(pgid, sig)
                except ProcessLookupError:
                    break
                end = time.monotonic() + wait
                while time.monotonic() < end:
                    proc.poll()   # reap the leader so it does not hold the group open
                    try:
                        os.killpg(pgid, 0)
                    except ProcessLookupError:
                        break
                    time.sleep(0.2)
                else:
                    continue
                break
        # Whatever escaped the group (the GUI-mode gz server does), by marker.
        if run_id:
            for _ in range(10):
                pids = marked_processes(run_id)
                if not pids:
                    break
                for pid in pids:
                    try:
                        os.kill(pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                time.sleep(0.3)


class App:
    def __init__(self, root):
        self.root = root
        self.events = queue.Queue()
        self.runner = None
        self.reports = {}
        self.passed = self.total = 0
        self.session_dir = None
        self.images = []   # keep PhotoImages alive

        root.title("Rover Test Runner")
        root.geometry("1400x860")
        root.minsize(1000, 640)
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        style = ttk.Style(root)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure("Treeview", rowheight=24)
        style.configure("Summary.TLabel", font=("TkDefaultFont", 12, "bold"))
        style.configure("Category.TLabel", font=("TkDefaultFont", 10, "bold"))

        self.tests = self.load_tests()
        self.build()
        root.after(100, self.poll)

    @staticmethod
    def load_tests():
        path = os.path.join(get_package_share_directory(PKG), "config", "test_cases.yaml")
        with open(path) as f:
            return yaml.safe_load(f).get("test_cases", [])

    # -- layout ---------------------------------------------------------------

    def build(self):
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)
        top = ttk.Frame(outer)
        top.pack(fill="x")

        # Test cases, by category
        cases = ttk.LabelFrame(top, text="Test cases", padding=8)
        cases.pack(side="left", fill="both", expand=True, padx=(0, 8))
        self.selected = {}
        categories = [c for c in CATEGORIES if any(t.get("category") == c for t in self.tests)]
        categories += sorted({t.get("category", "other") for t in self.tests} - set(categories))
        for col, category in enumerate(categories):
            frame = ttk.Frame(cases)
            frame.grid(row=0, column=col, sticky="nw", padx=(0, 18))
            head = ttk.Frame(frame)
            head.pack(fill="x", anchor="w")
            ttk.Label(head, text=category.capitalize(), style="Category.TLabel").pack(side="left")
            members = [t for t in self.tests if t.get("category", "other") == category]
            ttk.Button(head, text="none", width=5,
                       command=lambda m=members: self.select(m, False)).pack(side="right")
            ttk.Button(head, text="all", width=4,
                       command=lambda m=members: self.select(m, True)).pack(side="right", padx=2)
            for test in members:
                var = tk.BooleanVar(value=True)
                self.selected[test["name"]] = var
                cb = ttk.Checkbutton(frame, text=test["name"], variable=var)
                cb.pack(anchor="w", pady=1)
                Tooltip(cb, test.get("description", ""))
        buttons = ttk.Frame(cases)
        buttons.grid(row=1, column=0, columnspan=max(1, len(categories)), sticky="w",
                     pady=(8, 0))
        ttk.Button(buttons, text="Select all",
                   command=lambda: self.select(self.tests, True)).pack(side="left")
        ttk.Button(buttons, text="Select none",
                   command=lambda: self.select(self.tests, False)).pack(side="left", padx=4)
        ttk.Label(buttons, text="Hover a test for its description.",
                  foreground="#777").pack(side="left", padx=8)

        # Settings
        settings = ttk.LabelFrame(top, text="Settings", padding=8)
        settings.pack(side="left", fill="y")
        self.tolerance = tk.StringVar(value="0.3")
        self.timeout = tk.StringVar(value="")
        self.gui = tk.BooleanVar(value=False)
        ttk.Label(settings, text="Localization tolerance (m)").grid(row=0, column=0, sticky="w")
        ttk.Entry(settings, textvariable=self.tolerance, width=10).grid(row=0, column=1, padx=6)
        ttk.Label(settings, text="Timeout (sim s)").grid(row=1, column=0, sticky="w")
        ttk.Entry(settings, textvariable=self.timeout, width=10).grid(row=1, column=1, padx=6)
        ttk.Label(settings, text="blank = per-test default",
                  foreground="#777").grid(row=2, column=0, columnspan=2, sticky="w")
        ttk.Checkbutton(settings, text="Show Gazebo GUI",
                        variable=self.gui).grid(row=3, column=0, columnspan=2, sticky="w",
                                                pady=(4, 0))
        actions = ttk.Frame(settings)
        actions.grid(row=4, column=0, columnspan=2, sticky="we", pady=(10, 0))
        self.run_button = ttk.Button(actions, text="▶ Run", command=self.on_run)
        self.run_button.pack(side="left", fill="x", expand=True)
        self.stop_button = ttk.Button(actions, text="■ Stop", command=self.on_stop,
                                      state="disabled")
        self.stop_button.pack(side="left", fill="x", expand=True, padx=(4, 0))
        self.open_button = ttk.Button(settings, text="Open results folder",
                                      command=self.on_open_folder, state="disabled")
        self.open_button.grid(row=5, column=0, columnspan=2, sticky="we", pady=(4, 0))

        # Status line
        status = ttk.Frame(outer)
        status.pack(fill="x", pady=(10, 4))
        self.status = tk.StringVar(value="Idle. Select test cases and press Run.")
        ttk.Label(status, textvariable=self.status).pack(side="left")
        self.progress = ttk.Progressbar(status, mode="indeterminate", length=160)
        self.progress.pack(side="right")
        self.summary = tk.StringVar(value="")
        ttk.Label(status, textvariable=self.summary,
                  style="Summary.TLabel").pack(side="right", padx=12)

        # Results and log, split vertically
        panes = ttk.PanedWindow(outer, orient="vertical")
        panes.pack(fill="both", expand=True)

        results = ttk.LabelFrame(panes, text="Results (double-click a row for details)",
                                 padding=4)
        self.table = ttk.Treeview(results, columns=[c[0] for c in COLUMNS], show="headings",
                                  height=10)
        for key, title, width, anchor in COLUMNS:
            self.table.heading(key, text=title)
            self.table.column(key, width=width, anchor=anchor,
                              stretch=key in ("metrics", "note"))
        self.table.tag_configure("pass", background="#d4f4d4")
        self.table.tag_configure("fail", background="#f8d0d0")
        self.table.bind("<Double-1>", self.on_details)
        self.table.pack(fill="both", expand=True)
        panes.add(results, weight=2)

        logs = ttk.LabelFrame(panes, text="Launch output", padding=4)
        self.log = tk.Text(logs, height=12, wrap="none", font=("TkFixedFont", 9),
                           background="#1e1e1e", foreground="#d4d4d4", state="disabled")
        scroll = ttk.Scrollbar(logs, command=self.log.yview)
        self.log.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self.log.pack(fill="both", expand=True)
        panes.add(logs, weight=3)

    def select(self, tests, value):
        for test in tests:
            self.selected[test["name"]].set(value)

    # -- actions --------------------------------------------------------------

    def on_run(self):
        tests = [t for t in self.tests if self.selected[t["name"]].get()]
        if not tests:
            messagebox.showwarning("Nothing selected", "Select at least one test case.")
            return
        try:
            tolerance = float(self.tolerance.get())
            if tolerance <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid tolerance",
                                 "Localization tolerance must be a positive number of metres.")
            return
        timeout = None
        if self.timeout.get().strip():
            try:
                timeout = float(self.timeout.get())
                if timeout <= 0:
                    raise ValueError
            except ValueError:
                messagebox.showerror("Invalid timeout",
                                     "Timeout must be a positive number of seconds, or blank.")
                return
        if not self.check_stale_servers():
            return

        for row in self.table.get_children():
            self.table.delete(row)
        self.reports.clear()
        self.passed = self.total = 0
        self.summary.set("")
        settings = {"tolerance": tolerance, "timeout": timeout, "gui": self.gui.get()}
        self.runner = Runner(tests, settings, self.events)
        self.session_dir = self.runner.session_dir
        self.runner.start()
        self.run_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.open_button.configure(state="normal")
        self.progress.start(12)

    def check_stale_servers(self):
        """Offer to kill Gazebo servers already running. False cancels the run."""
        stale = gz_servers()
        if not stale:
            return True
        listing = "\n".join(f"  pid {pid}: {cmd[:90]}" for pid, cmd in stale[:8])
        answer = messagebox.askyesnocancel(
            "Gazebo already running",
            f"{len(stale)} Gazebo process(es) are already running:\n\n{listing}\n\n"
            "Each test isolates its own Gazebo traffic, but a leftover server still has "
            "a /controller_manager on the ROS graph that collides with the test's, and "
            "the rover will not move.\n\n"
            "Yes: kill them and run.   No: run anyway.   Cancel: do nothing.")
        if answer is None:
            return False
        if answer:
            for pid, _ in stale:
                try:
                    os.kill(pid, signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
            time.sleep(0.5)
        return True

    def on_stop(self):
        if self.runner:
            self.status.set("Stopping, tearing down the current test...")
            self.stop_button.configure(state="disabled")
            threading.Thread(target=self.runner.stop, daemon=True).start()

    def on_open_folder(self):
        if self.session_dir and os.path.isdir(self.session_dir):
            subprocess.Popen(["xdg-open", self.session_dir], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)

    def on_close(self):
        if self.runner and self.runner.is_alive():
            if not messagebox.askyesno("Tests running",
                                       "A test is running. Stop it and quit?"):
                return
            self.runner.stop()
        self.root.destroy()

    def on_details(self, _event):
        row = self.table.focus()
        report = self.reports.get(row)
        if report is not None:
            DetailsWindow(self.root, report, self.images)

    # -- events from the runner ----------------------------------------------

    def poll(self):
        lines = []
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "log":
                    lines.append(payload)
                elif kind == "status":
                    self.status.set(payload)
                elif kind == "result":
                    self.add_result(payload)
                elif kind == "done":
                    self.finish(payload)
        except queue.Empty:
            pass
        if lines:
            self.append_log("".join(lines))
        self.root.after(100, self.poll)

    def append_log(self, text):
        at_end = self.log.yview()[1] >= 0.999
        self.log.configure(state="normal")
        self.log.insert("end", text)
        excess = int(self.log.index("end-1c").split(".")[0]) - LOG_LINES
        if excess > 0:
            self.log.delete("1.0", f"{excess + 1}.0")
        self.log.configure(state="disabled")
        if at_end:
            self.log.see("end")

    def add_result(self, report):
        m = report.get("metrics", {})
        passed = report.get("passed", False)
        nav = report.get("navigation", {})
        note = report.get("failure_reason") or (
            f"Nav2 {nav['status']}" if nav.get("status") else "")
        duration = m.get("time_to_goal_s", m.get("drive_time_s"))
        row = self.table.insert("", "end", tags=("pass" if passed else "fail",), values=(
            report.get("test_name", "?"),
            report.get("kind", "-"),
            "PASS" if passed else "FAIL",
            fmt(duration, 1),
            fmt(m.get("startup_wall_s"), 1),
            fmt(m.get("initial_loc_error_m")),
            fmt(m.get("final_loc_error_m")),
            key_metrics(report),
            note,
        ))
        self.reports[row] = report
        self.total += 1
        self.passed += int(passed)
        self.summary.set(f"{self.passed}/{self.total} passed")

    def finish(self, session_dir):
        self.progress.stop()
        self.run_button.configure(state="normal")
        self.stop_button.configure(state="disabled")
        self.status.set(f"Done. Results saved in {session_dir}")
        self.runner = None


class DetailsWindow:
    """Checks, metrics, any images (map overlays), then the raw report."""

    def __init__(self, root, report, keep_alive):
        win = tk.Toplevel(root)
        win.title(f"{report.get('test_name', '')}: "
                  f"{'PASS' if report.get('passed') else 'FAIL'}")
        win.geometry("900x820")
        notebook = ttk.Notebook(win)
        notebook.pack(fill="both", expand=True)

        summary = tk.Text(notebook, wrap="word", font=("TkFixedFont", 10), padx=8, pady=8)
        summary.tag_configure("pass", foreground="#1a7f37")
        summary.tag_configure("fail", foreground="#c62828")
        summary.tag_configure("head", font=("TkDefaultFont", 11, "bold"))
        if report.get("failure_reason"):
            summary.insert("end", report["failure_reason"] + "\n\n", "fail")
        summary.insert("end", "Checks\n", "head")
        for name, ok in report.get("checks", {}).items():
            summary.insert("end", f"  {'PASS' if ok else 'FAIL'}  ", "pass" if ok else "fail")
            summary.insert("end", f"{name}\n")
        summary.insert("end", "\nMetrics\n", "head")
        for name, value in report.get("metrics", {}).items():
            text = f"{value:.4f}" if isinstance(value, float) else str(value)
            summary.insert("end", f"  {name:28} {text}\n")
        legend = report.get("mapping", {}).get("legend")
        if legend:
            summary.insert("end", "\nMap overlay colours\n", "head")
            for colour, meaning in legend.items():
                summary.insert("end", f"  {colour:12} {meaning}\n")
        summary.insert("end", f"\nLog: {report.get('log_file', '-')}\n"
                              f"Report: {report.get('result_file', '-')}\n")
        summary.configure(state="disabled")
        notebook.add(summary, text="Summary")

        for path in report.get("images", []):
            frame = ttk.Frame(notebook)
            try:
                from PIL import Image, ImageTk
                image = Image.open(path)
                image.thumbnail((880, 760))
                photo = ImageTk.PhotoImage(image, master=win)
                keep_alive.append(photo)
                ttk.Label(frame, image=photo).pack(padx=4, pady=4)
            except Exception as e:  # noqa: BLE001 -- a missing image is not fatal
                ttk.Label(frame, text=f"could not load {path}: {e}").pack()
            ttk.Label(frame, text=path, foreground="#777").pack()
            notebook.add(frame, text=os.path.basename(path).replace(".png", ""))

        raw = tk.Text(notebook, wrap="none", font=("TkFixedFont", 9))
        trimmed = {k: v for k, v in report.items() if k != "drive"}
        if "drive" in report:   # the per-sample trace is long; keep it out of the way
            trimmed["drive"] = {k: v for k, v in report["drive"].items() if k != "samples"}
            trimmed["drive"]["samples"] = f"<{len(report['drive'].get('samples', []))} samples>"
        raw.insert("end", json.dumps(trimmed, indent=2))
        raw.configure(state="disabled")
        notebook.add(raw, text="Raw report")


class Tooltip:
    def __init__(self, widget, text):
        self.widget, self.text, self.tip = widget, text, None
        if text:
            widget.bind("<Enter>", self.show)
            widget.bind("<Leave>", self.hide)

    def show(self, _event):
        x = self.widget.winfo_rootx() + 20
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + 2
        self.tip = tk.Toplevel(self.widget)
        self.tip.wm_overrideredirect(True)
        self.tip.wm_geometry(f"+{x}+{y}")
        tk.Label(self.tip, text=self.text, background="#ffffe0", relief="solid",
                 borderwidth=1, padx=4, pady=2).pack()

    def hide(self, _event):
        if self.tip:
            self.tip.destroy()
            self.tip = None


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
