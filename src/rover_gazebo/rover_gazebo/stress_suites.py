"""Stress-test scenarios for stability_test.py: the terrain worlds written by
tools/generate_stress_worlds.py, plus command abuse on flat ground.

Data only. Each suite is a list of scenario dicts built with stability_test.S(); the
extra keys used here are:

    start     (x, y, z, roll, pitch, yaw) to teleport the rover to before the scenario
              -- lane geometry below mirrors generate_stress_worlds.py
    program   [(seconds, (vx, vy, wz) or None, mode or None), ...] driven in order
              instead of one constant Twist; None for the Twist means publish nothing
              (cmd_vel dropout)
    recover   after the scenario, switch to ackermann and drive 0.5 m/s straight for
              6 s, judged against the strict flat-ground limits: does the rover still
              behave normally after what was just done to it?

Every scenario is also checked for "survive" (no NaN, not flipped, no runaway joint or
body speed) and "health" (expected controllers active, every control node alive).
"""

import math

import numpy as np

NAN, INF = float("nan"), float("inf")


def build(S, OBSTACLE_LIMITS):
    relaxed = dict(OBSTACLE_LIMITS)
    suites = {}

    # -- slopes.sdf: lane y = 4k at 10/15/20/25 deg, 3 m ramps from x = 2 -----------
    def lane(deg):
        a = math.radians(deg)
        y, run, h = 4.0 * [10, 15, 20, 25].index(deg), 3.0 * math.cos(a), 3.0 * math.sin(a)
        return a, y, run, h

    sl = [S("settle", "ackermann", drive=0.0, hold=4.0, kind="hold")]
    for deg in (15, 20, 25):
        a, y, run, h = lane(deg)
        for mode in ("ackermann", "explicit_pid"):
            sl.append(S(f"{mode}_climb_{deg}", mode, vx=0.4, drive=14.0, kind="climb",
                        aim=0.0 if mode == "explicit_pid" else None,
                        start=(0.0, y, 0.03, 0, 0, 0), limits=relaxed))
    for deg in (20, 25):
        a, y, run, h = lane(deg)
        mid = (2.0 + run / 2, y, 1.5 * math.sin(a) + 0.04, 0, -a, 0)
        for mode in ("ackermann", "explicit_pid"):
            # Parked nose-up mid-ramp, nothing commanded: does it stay put?
            sl.append(S(f"{mode}_hold_on_{deg}", mode, hold=6.0, drive=0.0, kind="hold",
                        aim=0.0 if mode == "explicit_pid" else None, start=mid,
                        limits=dict(relaxed, hold_move=0.05, hold_yaw=2.0)))
            # Stop and restart going uphill.
            sl.append(S(f"{mode}_restart_up_{deg}", mode, vx=0.3, drive=6.0, kind="climb",
                        aim=0.0 if mode == "explicit_pid" else None, start=mid,
                        skip=1.0, limits=relaxed))
    for deg in (20, 25):
        a, y, run, h = lane(deg)
        top = (2.0 + run + 0.6, y, h + 0.03, 0, 0, 0)
        for mode in ("ackermann", "explicit_pid"):
            sl.append(S(f"{mode}_descend_{deg}", mode, vx=0.4, drive=12.0, kind="descend",
                        aim=0.0 if mode == "explicit_pid" else None, start=top,
                        limits=relaxed))
    suites["slopes"] = sl

    # -- side_slope.sdf: centre lines at y = 0 (12 deg) and y = 6 (18 deg), x 2..14 -
    ss = []
    for deg, y in ((12, 0.0), (18, 6.0)):
        r = math.radians(deg)
        start = (2.8, y, 2.0 * math.sin(r) + 0.04, r, 0, 0)
        for mode in ("ackermann", "explicit_pid"):
            aim = 0.0 if mode == "explicit_pid" else None
            ss.append(S(f"{mode}_hold_side_{deg}", mode, hold=5.0, drive=0.0, kind="hold",
                        aim=aim, start=start, limits=dict(relaxed, hold_move=0.05,
                                                          hold_yaw=2.0)))
            ss.append(S(f"{mode}_traverse_side_{deg}", mode, vx=0.4, drive=12.0,
                        kind="sideslope", aim=aim, start=start, limits=relaxed))
    suites["side_slope"] = ss

    # -- rocks.sdf: corridor y = 0 (x 1.5..13.5), dense corridor y = 5 --------------
    rk = []
    for y, tag, dur in ((0.0, "sparse", 22.0), (5.0, "dense", 18.0)):
        for mode in ("ackermann", "explicit_pid"):
            rk.append(S(f"{mode}_rocks_{tag}", mode, vx=0.4, drive=dur, kind="obstacle",
                        aim=0.0 if mode == "explicit_pid" else None,
                        start=(0.0, y, 0.03, 0, 0, 0),
                        limits=dict(relaxed, straight_yaw=15.0)))
    rk.append(S("ackermann_rocks_arc", "ackermann", vx=0.4, wz=0.15, drive=15.0,
                kind="survive", start=(0.0, 0.0, 0.03, 0, 0, 0), limits=relaxed))
    rk.append(S("explicit_rocks_aim_20", "explicit", vx=0.4, aim=20.0, hold=1.0, drive=15.0,
                kind="survive", start=(0.0, -0.5, 0.03, 0, 0, 0), limits=relaxed))
    suites["rocks"] = rk

    # -- washboard.sdf: 0.25 m pitch at y = 0, 0.40 m at y = 4 ----------------------
    wb = []
    for y, tag in ((0.0, "025"), (4.0, "040")):
        for mode, v in (("ackermann", 0.4), ("ackermann", 0.9), ("explicit_pid", 0.4),
                        ("explicit_pid", 0.9)):
            wb.append(S(f"{mode}_washboard_{tag}_v{int(v * 10):02d}", mode, vx=v,
                        drive=min(22.0, 11.0 / v), kind="obstacle",
                        aim=0.0 if mode == "explicit_pid" else None,
                        start=(0.0, y, 0.03, 0, 0, 0), limits=relaxed))
    suites["washboard"] = wb

    # -- low_friction.sdf: mu 0.3 (y = 0), split mu (y = 4), 15 deg mu 0.5 ramp (y = 8)
    fr = []
    for mode in ("ackermann", "explicit_pid"):
        aim = 0.0 if mode == "explicit_pid" else None
        fr.append(S(f"{mode}_mu03_straight", mode, vx=0.6, drive=12.0, kind="straight",
                    aim=aim, start=(0.5, 0.0, 0.03, 0, 0, 0),
                    limits=dict(relaxed, speed_frac=0.25, straight_yaw=6.0)))
        fr.append(S(f"{mode}_mu03_stop_go", mode, kind="survive", aim=aim,
                    start=(0.5, 0.0, 0.03, 0, 0, 0), limits=relaxed,
                    program=[(3.0, (1.0, 0, 0), None), (1.5, (-1.0, 0, 0), None),
                             (3.0, (1.0, 0, 0), None), (2.0, (0, 0, 0), None)]))
        fr.append(S(f"{mode}_split_mu", mode, vx=0.6, drive=12.0, kind="straight", aim=aim,
                    start=(0.5, 4.0, 0.03, 0, 0, 0),
                    limits=dict(relaxed, speed_frac=0.25, straight_yaw=6.0)))
        fr.append(S(f"{mode}_climb_15_mu05", mode, vx=0.4, drive=12.0, kind="climb",
                    aim=aim, start=(0.0, 8.0, 0.03, 0, 0, 0), limits=relaxed))
    fr.append(S("ackermann_mu03_arc", "ackermann", vx=0.6, wz=0.5, drive=8.0, kind="arc",
                start=(2.5, 0.0, 0.04, 0, 0, 0), limits=dict(relaxed, yaw_rate_frac=0.35)))
    suites["friction"] = fr

    # -- rough_terrain.sdf: x 2..22, y -10..10; three lanes in from x = 0 ----------
    rg = []
    for y in (-4.0, 0.0, 4.0):
        for mode in ("ackermann", "explicit_pid"):
            rg.append(S(f"{mode}_rough_y{int(y):+d}", mode, vx=0.5, drive=20.0,
                        kind="obstacle", aim=0.0 if mode == "explicit_pid" else None,
                        start=(0.0, y, 0.03, 0, 0, 0),
                        limits=dict(relaxed, straight_yaw=15.0)))
    rg.append(S("ackermann_rough_slalom", "ackermann", kind="survive",
                start=(0.0, 0.0, 0.03, 0, 0, 0), limits=relaxed,
                program=[(4.0, (0.5, 0, 0.5), None), (4.0, (0.5, 0, -0.5), None)] * 3))
    rg.append(S("spot_on_rough", "spot", wz=0.5, drive=8.0, kind="survive",
                start=(8.0, 0.0, 0.25, 0, 0, 0), limits=dict(relaxed, residual=20.0)))
    rg.append(S("crab_on_rough", "crab", vx=0.2, vy=0.3, drive=8.0, kind="survive",
                start=(6.0, -2.0, 0.25, 0, 0, 0), limits=dict(relaxed, residual=20.0)))
    suites["rough"] = rg

    # -- flat.sdf: command abuse -------------------------------------------------
    rng = np.random.default_rng(3)
    modes = ("ackermann", "crab", "spot", "explicit", "explicit_pid")
    storm = [(0.5, (float(rng.uniform(-1, 1)), float(rng.uniform(-0.5, 0.5)),
                    float(rng.uniform(-1, 1))), modes[rng.integers(5)]) for _ in range(30)]
    ab = [
        S("settle", "ackermann", drive=0.0, hold=4.0, kind="hold"),
        S("abuse_step_reversal", "ackermann", limits=relaxed, kind="survive", recover=True,
          program=[(1.5, (1.0, 0, 0), None), (1.5, (-1.0, 0, 0), None)] * 8),
        S("abuse_step_reversal_pid", "explicit_pid", limits=relaxed, aim=0.0, kind="survive", recover=True,
          program=[(1.5, (1.0, 0, 0), None), (1.5, (-1.0, 0, 0), None)] * 8),
        S("abuse_boost_2_5", "ackermann", vx=2.5, drive=8.0, kind="clamp"),
        S("abuse_huge_values", "ackermann", limits=relaxed, vx=1e9, vy=-1e9, wz=1e9, drive=6.0,
          kind="clamp", recover=True),
        S("abuse_full_speed_pid", "explicit_pid", vx=1.0, aim=0.0, drive=8.0,
          kind="explicit"),
        S("abuse_mode_storm", "ackermann", limits=relaxed, kind="survive", recover=True, program=storm),
        S("abuse_pid_toggle", "explicit", limits=relaxed, aim=0.0, kind="survive", recover=True,
          program=[(0.4, (0.5, 0, 0), "explicit_pid"), (0.4, (0.5, 0, 0), "explicit")] * 20),
        S("abuse_aim_sweep_driving_pid", "explicit_pid", limits=relaxed, aim=0.0, kind="survive",
          recover=True,
          program=[(2.0, (0.5, 0, 1.0), None), (4.0, (0.5, 0, -1.0), None),
                   (2.0, (0.5, 0, 1.0), None), (2.0, (0.5, 0, 0), None)]),
        S("abuse_cmd_dropout", "ackermann", limits=relaxed, kind="dropout",
          program=[(4.0, (0.8, 0, 0), None), (4.0, None, None)]),
        S("abuse_cmd_dropout_pid", "explicit_pid", limits=relaxed, aim=0.0, kind="dropout",
          program=[(4.0, (0.8, 0, 0), None), (4.0, None, None)]),
        S("abuse_nan_linear", "ackermann", limits=relaxed, kind="reject", recover=True,
          program=[(4.0, (NAN, 0, 0), None)]),
        S("abuse_nan_steer_rate_pid", "explicit_pid", limits=relaxed, aim=0.0, kind="reject", recover=True,
          program=[(4.0, (0, 0, NAN), None)]),
        S("abuse_inf_ackermann", "ackermann", limits=relaxed, kind="reject", recover=True,
          program=[(4.0, (INF, 0, -INF), None)]),
        S("abuse_unknown_mode", "ackermann", vx=0.5, drive=6.0, kind="straight",
          program=[(0.5, (0, 0, 0), "warp_drive"), (6.0, (0.5, 0, 0), None)]),
        # What an operator does with the keyboard: hold A in explicit until the aim
        # stops at its 90 deg clamp, then press 1 and drive. Found by the NaN case.
        S("abuse_explicit_aim_clamp_then_ackermann", "explicit", kind="survive",
          recover=True, program=[(8.0, (0, 0, 0.2618), None)]),
        S("endurance_explicit_pid_120s", "explicit_pid", vx=0.5, aim=0.0, drive=120.0,
          kind="explicit"),
        S("endurance_pid_hold_60s", "explicit_pid", aim=0.0, hold=60.0, drive=0.0,
          kind="hold"),
    ]
    suites["abuse"] = ab

    return suites


# Which world each suite expects (tools/run_stress_suite.sh reads the same mapping).
WORLD = {"slopes": "slopes", "side_slope": "side_slope", "rocks": "rocks",
         "washboard": "washboard", "friction": "low_friction", "rough": "rough_terrain",
         "abuse": "flat"}
