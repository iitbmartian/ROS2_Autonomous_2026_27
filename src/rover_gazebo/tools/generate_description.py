#!/usr/bin/env python3
"""Generate urdf/rover_body.xacro from the original SolidWorks export.

Link masses, inertia tensors and joint origins are transcribed straight from the
export so they cannot drift. Everything the export got wrong for Gazebo is fixed
here, in one place, with a comment saying why:

  * the chassis link is renamed, so a ROS-convention base_link can sit above it
  * every joint gets real effort and velocity limits, because the export writes 0
  * loop-closing joints get effort interfaces and are held by rocker_coupling_node,
    since Gazebo has no solver-level way to enforce them
  * mesh collision is replaced by primitives on the wheels and chassis
  * the eight-millimetre ball links become fixed joints, so sdformat folds them away

Run:  python3 generate_description.py
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET

import derive_ratios as D

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "..", "..", "reference", "urdf_full_withdifferential",
                    "urdf", "urdf_full_withdifferential.urdf")
OUT = os.path.join(HERE, "..", "urdf", "rover_body.xacro")

CHASSIS = "chassis_link"          # the export calls it base_link; we need that name free
WHEELS = ["FLD", "FRD", "BLD", "BRD"]
ROCKERS = ["FLS", "FRS", "BLS", "BRS"]
STEERS = ["FLE", "FRE", "BLE", "BRE"]
# Purely decorative: they carry no load once the loops are replaced by constraints.
# Always fixed; nothing drives them once the loop is gone.
DECOR_FOLLOWERS = ["LCON", "RCON", "Differential"]
DECOR_FIXED = ["Ldiff", "Rdiff", "Ldiffcon", "Rdiffcon", "Ldiffcon2", "Rdiffcon2"]

ROCKER_LIMIT = 0.45               # rad, suspension travel stop on the free joint
FOLLOWER_LIMIT = 0.60             # rad, generous so the coupling never fights its own stop
STEER_LIMIT = 1.75                # rad, just past a quarter turn for spot mode
EFFORT = 500.0                    # N m, steering and drive motors
# The rockers are not motors. Their "actuator" stands in for a steel connector bar
# and a differential, so the limit has to be structural rather than motor sized, or
# the coupling saturates and the suspension sags.
ROCKER_EFFORT = 5000.0            # N m
VELOCITY = 20.0                   # rad/s


def fmt(x, n=10):
    """Trim the exporter's 15 significant figures to something readable."""
    s = f"{float(x):.{n}g}"
    return "0" if s in ("-0", "0", "-0.0", "0.0") else s


def vec(s, n=10):
    return " ".join(fmt(v, n) for v in s.split())


class Gen:
    def __init__(self):
        self.u = D.Urdf(URDF)
        self.d = D.derive(URDF)
        self.tree = ET.parse(URDF).getroot()
        self.out = []

    def w(self, s=""):
        self.out.append(s)

    # -- links ------------------------------------------------------------
    def link_name(self, n):
        return CHASSIS if n == "base_link" else n

    def emit_link(self, el):
        raw = el.get("name")
        name = self.link_name(raw)
        inertial = el.find("inertial")
        o = inertial.find("origin")
        mass = inertial.find("mass").get("value")
        i = inertial.find("inertia")

        self.w(f'  <link name="{name}">')
        self.w("    <inertial>")
        self.w(f'      <origin xyz="{vec(o.get("xyz"))}" rpy="{vec(o.get("rpy"))}"/>')
        self.w(f'      <mass value="${{{fmt(mass)} * mass_scale}}"/>')
        self.w("      <inertia")
        self.w(f'        ixx="${{{fmt(i.get("ixx"))} * mass_scale}}"'
               f' ixy="${{{fmt(i.get("ixy"))} * mass_scale}}"'
               f' ixz="${{{fmt(i.get("ixz"))} * mass_scale}}"')
        self.w(f'        iyy="${{{fmt(i.get("iyy"))} * mass_scale}}"'
               f' iyz="${{{fmt(i.get("iyz"))} * mass_scale}}"'
               f' izz="${{{fmt(i.get("izz"))} * mass_scale}}"/>')
        self.w("    </inertial>")

        self.w("    <visual>")
        self.w('      <origin xyz="0 0 0" rpy="0 0 0"/>')
        self.w(f'      <geometry><mesh filename="package://rover_gazebo/meshes/{raw}.STL"/></geometry>')
        self.w('      <material name="rover_grey"/>')
        self.w("    </visual>")

        self.emit_collision(raw)
        self.w("  </link>")
        self.w()

    def emit_collision(self, raw):
        r = self.d["drivetrain"]["wheel_radius_m"]
        h = self.d["drivetrain"]["wheel_width_m"]

        if raw in WHEELS:
            # The wheel STL is a plain cylinder with no tread, so a primitive loses
            # nothing and gives the contact solver something it can handle well.
            sign = 1.0 if raw == "FLD" else -1.0
            self.w("    <!-- cylinder matches the STL to a tenth of a millimetre -->")
            self.w("    <collision>")
            self.w(f'      <origin xyz="0 0 {fmt(sign * h / 2.0, 6)}" rpy="0 0 0"/>')
            self.w(f'      <geometry><cylinder radius="{fmt(r, 6)}" length="{fmt(h, 6)}"/></geometry>')
            self.w("    </collision>")
            return

        if raw == "base_link":
            self.w("    <!-- the chassis STL is exactly a 0.4 m cube -->")
            self.w("    <collision>")
            self.w('      <origin xyz="0 -0.2 0" rpy="0 0 0"/>')
            self.w('      <geometry><box size="0.4 0.4 0.4"/></geometry>')
            self.w("    </collision>")
            return

        if raw in ROCKERS or raw in STEERS:
            # Arms only ever touch the ground in a rollover. Off by default so the
            # solver is not asked to resolve mesh-on-mesh contact every step.
            self.w('    <xacro:if value="${mesh_collision}">')
            self.w("      <collision>")
            self.w('        <origin xyz="0 0 0" rpy="0 0 0"/>')
            self.w(f'        <geometry><mesh filename="package://rover_gazebo/meshes/{raw}.STL"/></geometry>')
            self.w("      </collision>")
            self.w("    </xacro:if>")
            return

        # Decorative links never collide: the connector bars and the differential
        # chain overlap their neighbours by design and would self-collide.

    # -- joints -----------------------------------------------------------
    def emit_joint(self, el):
        raw = el.get("name")
        child = el.find("child").get("link")
        parent = self.link_name(el.find("parent").get("link"))
        o = el.find("origin")
        ax = el.find("axis")
        axis = ax.get("xyz") if ax is not None else "0 0 1"
        ratios = self.d["coupling_ratios"]

        # SDF keeps links, joints and frames in one namespace, so the export's habit
        # of naming a joint after its own child link makes the model unloadable in
        # Gazebo. Every joint gets the conventional suffix, which also sidesteps the
        # export naming one joint "Rdifcon2" while its link is "Rdiffcon2".
        name = f"{child}_joint"

        def head(jtype):
            self.w(f'  <joint name="{name}" type="{jtype}">')
            self.w(f'      <origin xyz="{vec(o.get("xyz"))}" rpy="{vec(o.get("rpy"), 12)}"/>')
            self.w(f'      <parent link="{parent}"/>')
            self.w(f'      <child link="{child}"/>')

        def limits(lim, eff=EFFORT, vel=VELOCITY):
            self.w(f'      <axis xyz="{vec(axis)}"/>')
            self.w(f'      <limit lower="{-lim}" upper="{lim}" effort="{eff}" velocity="{vel}"/>')

        if child in DECOR_FIXED:
            self.w("  <!-- decorative; fixed so sdformat folds it into its parent -->")
            head("fixed")
            self.w("  </joint>")
            self.w()
            return

        if child in DECOR_FOLLOWERS:
            self.w("  <!-- decorative; fixed so sdformat folds it into its parent. Nothing")
            self.w("       drives it once the loop it closed is replaced by a torque")
            self.w("       coupling on the rockers themselves. -->")
            self.w(f'  <joint name="{name}" type="fixed">')
            self.w(f'      <origin xyz="{vec(o.get("xyz"))}" rpy="{vec(o.get("rpy"), 12)}"/>')
            self.w(f'      <parent link="{parent}"/>')
            self.w(f'      <child link="{child}"/>')
            self.w("  </joint>")
            self.w()
            return

        if raw in WHEELS:
            self.w("  <!-- drive; continuous, the export's effort=0 would lock it solid -->")
            head("continuous")
            self.w(f'      <axis xyz="{vec(axis)}"/>')
            self.w(f'      <limit effort="{EFFORT}" velocity="30.0"/>')
            self.w('      <dynamics damping="0.05" friction="0.1"/>')
            self.w("  </joint>")
            self.w()
            return

        if raw in STEERS:
            head("revolute")
            limits(STEER_LIMIT)
            self.w('      <dynamics damping="1.0" friction="0.5"/>')
            self.w("  </joint>")
            self.w()
            return

        if raw == "FLS":
            self.w("  <!-- the one free suspension coordinate; everything else mimics it -->")
            head("revolute")
            limits(ROCKER_LIMIT, ROCKER_EFFORT)
            self.w('      <dynamics damping="5.0" friction="0.5"/>')
            self.w("  </joint>")
            self.w()
            return

        if raw in ROCKERS:
            self.w(f"  <!-- follower: {raw} = {ratios[raw]:+.5f} x FLS, held there by "
                   f"rocker_coupling_node -->")
            head("revolute")
            limits(FOLLOWER_LIMIT, ROCKER_EFFORT)
            self.w('      <dynamics damping="5.0" friction="0.5"/>')
            self.w("  </joint>")
            self.w()
            return

        raise SystemExit(f"unhandled joint {raw}")

    # -- driver -----------------------------------------------------------
    def run(self):
        m = self.d["coupling_ratios"]
        self.w('<?xml version="1.0"?>')
        self.w("<!--")
        self.w("  GENERATED by tools/generate_description.py. Do not edit by hand.")
        self.w("")
        self.w("  Links, masses and inertia tensors are transcribed from the SolidWorks")
        self.w("  export. The suspension's four closed loops, which URDF cannot express,")
        self.w("  are replaced by linear joint couplings derived in tools/derive_ratios.py")
        self.w("  and held at runtime by rocker_coupling_node, which applies them as")
        self.w("  torques, since Gazebo has no solver-level way to enforce them:")
        self.w("")
        for k in ("BLS", "BRS", "FRS"):
            self.w(f"      {k}_joint = {m[k]:+.5f} x FLS_joint")
        self.w("")
        self.w("  leaving FLS_joint as the single free suspension coordinate.")
        self.w("")
        self.w("  Joint names carry a _joint suffix because SDF keeps links and joints in")
        self.w("  one namespace and the export gave them matching names. Link and frame")
        self.w("  names are unchanged from the CAD.")
        self.w("-->")
        self.w('<robot xmlns:xacro="http://www.ros.org/wiki/xacro">')
        self.w()
        self.w('  <material name="rover_grey"><color rgba="0.792 0.820 0.933 1"/></material>')
        self.w()
        for el in self.tree.findall("link"):
            self.emit_link(el)
        for el in self.tree.findall("joint"):
            self.emit_joint(el)
        self.w("</robot>")

        with open(OUT, "w") as fh:
            fh.write("\n".join(self.out) + "\n")
        print(f"wrote {os.path.abspath(OUT)}  ({len(self.out)} lines)")


if __name__ == "__main__":
    Gen().run()
