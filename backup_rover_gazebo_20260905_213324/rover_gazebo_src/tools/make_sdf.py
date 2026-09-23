#!/usr/bin/env python3
"""Build a native SDF model whose mimic constraints cannot be lost in translation.

Why this exists.

Gazebo reaches a URDF through sdformat's URDF-to-SDF converter. Older converters drop
<mimic> silently: no warning, no error, and a rover whose suspension quietly falls
apart. This tool goes around the converter. It runs the conversion, then writes the
mimic constraints straight into the resulting SDF, and asks Gazebo to validate the
result before you ever load it.

It does not guess at the schema. It reads the <mimic> definition out of the sdformat
specification installed on this machine and emits exactly what that specification asks
for. If no installed specification mentions mimic, this version of Gazebo cannot do
solver-level coupling at all, and the tool says so and points at the fallback.

    python3 make_sdf.py                       # writes models/rover/model.sdf
    python3 make_sdf.py --check-only          # just report what this Gazebo supports
"""

from __future__ import annotations

import argparse
import glob
import os
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))


def package_share():
    """Where the package's data lives, whether we are run from source or installed."""
    here = os.path.dirname(os.path.abspath(__file__))
    # From source, tools/ sits beside urdf/. Installed, the scripts land in lib/ and
    # the data in share/, so ask the index.
    if os.path.isdir(os.path.join(here, "..", "urdf")):
        return os.path.abspath(os.path.join(here, ".."))
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("rover_gazebo")
    except Exception:
        return os.path.abspath(os.path.join(here, ".."))


PKG = package_share()
XACRO = os.path.join(PKG, "urdf", "rover.urdf.xacro")
OUT_DIR = os.path.join(PKG, "models", "rover")

SPEC_DIRS = ("/usr/share/sdformat*", "/usr/local/share/sdformat*",
             os.path.expanduser("~/.local/share/sdformat*"))


def gz_tool():
    """The command that converts and validates SDF, whatever it is called here."""
    for exe in ("gz", "ign"):
        if shutil.which(exe):
            return exe
    return None


def find_mimic_spec(extra_dirs=()):
    """Locate the sdformat spec that defines <mimic>, and read its shape from it.

    Returns (spec_version, {'attrs': [...], 'elems': [...]}) or None.
    """
    best = None
    for pattern in tuple(extra_dirs) + SPEC_DIRS:
        for spec_root in sorted(glob.glob(pattern)):
            for version_dir in sorted(glob.glob(os.path.join(spec_root, "1.*"))):
                for f in glob.glob(os.path.join(version_dir, "*.sdf")):
                    try:
                        root = ET.parse(f).getroot()
                    except ET.ParseError:
                        continue
                    for el in root.iter("element"):
                        if el.get("name") != "mimic":
                            continue
                        shape = {
                            "attrs": [a.get("name") for a in el.findall("attribute")],
                            "elems": [c.get("name") for c in el.findall("element")],
                            "file": f,
                        }
                        version = os.path.basename(version_dir)
                        if best is None or _newer(version, best[0]):
                            best = (version, shape)
    return best


def _newer(a, b):
    def key(v):
        return tuple(int(p) for p in v.split(".") if p.isdigit())
    return key(a) > key(b)


def mimic_element(shape, leader, multiplier, offset=0.0, reference=0.0):
    """Build a <mimic> node in whatever form the installed specification defines."""
    el = ET.Element("mimic")
    values = {"joint": leader, "multiplier": multiplier,
              "offset": offset, "reference": reference}

    for name in shape["attrs"]:
        if name in values:
            el.set(name, _str(values[name]))
    for name in shape["elems"]:
        if name in values:
            child = ET.SubElement(el, name)
            child.text = _str(values[name])
    if "joint" not in shape["attrs"] and "joint" not in shape["elems"]:
        # Every known spelling names the leader somehow; fall back to an attribute.
        el.set("joint", leader)
    return el


def _str(v):
    return v if isinstance(v, str) else f"{float(v):.9g}"


def read_mimics(urdf_path):
    """Pull the mimic table straight out of the generated URDF."""
    root = ET.parse(urdf_path).getroot()
    table = {}
    for j in root.findall("joint"):
        m = j.find("mimic")
        if m is not None:
            table[j.get("name")] = (m.get("joint"),
                                    float(m.get("multiplier", 1.0)),
                                    float(m.get("offset", 0.0)))
    return table


def run(cmd, **kw):
    return subprocess.run(cmd, check=True, capture_output=True, text=True, **kw)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=OUT_DIR)
    ap.add_argument("--sim", default="harmonic", choices=("harmonic", "fortress"))
    ap.add_argument("--mass-scale", default="1.0")
    ap.add_argument("--mesh-collision", default="false")
    ap.add_argument("--controllers-file",
                    default=os.path.join(PKG, "config", "controllers.yaml"))
    ap.add_argument("--check-only", action="store_true")
    ap.add_argument("--spec-dir", action="append", default=[],
                    help="extra sdformat specification directory to search; useful for "
                         "trying the model against a newer Gazebo's schema than the one "
                         "installed here")
    a = ap.parse_args()

    gz = gz_tool()
    print(f"gz tool            : {gz or 'NOT FOUND'}")

    found = find_mimic_spec(a.spec_dir)
    if not found:
        print("mimic in sdformat  : NOT SUPPORTED by any installed specification")
        print()
        print("  This Gazebo cannot enforce joint coupling in the solver. Run the")
        print("  simulation with coupling:=controller instead, which applies the same")
        print("  constraints as torques and works on every version:")
        print()
        print("      ros2 launch rover_gazebo rover_sim.launch.py coupling:=controller")
        return 2

    version, shape = found
    print(f"mimic in sdformat  : yes, spec {version}")
    print(f"  defined in       : {shape['file']}")
    print(f"  attributes       : {shape['attrs'] or 'none'}")
    print(f"  child elements   : {shape['elems'] or 'none'}")
    if a.check_only:
        return 0
    if not gz:
        print("\nCannot convert without the gz or ign command line tool.")
        return 1

    tmp = tempfile.mkdtemp(prefix="rover_sdf_")
    urdf = os.path.join(tmp, "rover.urdf")
    with open(urdf, "w") as fh:
        fh.write(run(["xacro", XACRO,
                      "coupling:=mimic",
                      f"sim:={a.sim}",
                      f"mass_scale:={a.mass_scale}",
                      f"mesh_collision:={a.mesh_collision}",
                      f"controllers_file:={a.controllers_file}"]).stdout)

    mimics = read_mimics(urdf)
    print(f"\nmimic constraints in the URDF: {len(mimics)}")

    sdf_text = run([gz, "sdf", "-p", urdf]).stdout
    root = ET.fromstring(sdf_text)
    model = root.find("model")
    if model is None:
        print("conversion produced no <model>; aborting")
        return 1

    by_name = {j.get("name"): j for j in model.findall("joint")}
    survived = sum(1 for j in by_name.values() if j.find("axis/mimic") is not None)
    print(f"survived the converter unaided: {survived}")

    added = 0
    for name, (leader, mult, off) in mimics.items():
        joint = by_name.get(name)
        if joint is None:
            print(f"  ! joint {name} vanished in conversion, most likely lumped away")
            continue
        axis = joint.find("axis")
        if axis is None:
            print(f"  ! joint {name} has no <axis>")
            continue
        for old in axis.findall("mimic"):
            axis.remove(old)
        axis.append(mimic_element(shape, leader, mult, off))
        added += 1
    print(f"written in directly           : {added}")

    root.set("version", version)
    os.makedirs(a.out, exist_ok=True)
    model_path = os.path.join(a.out, "model.sdf")
    _indent(root)
    ET.ElementTree(root).write(model_path, encoding="utf-8", xml_declaration=True)

    with open(os.path.join(a.out, "model.config"), "w") as fh:
        fh.write(CONFIG)

    print(f"\nwrote {model_path}")

    # Let Gazebo itself say whether the file is acceptable. This is the check that
    # matters: it runs against the parser that will load the model.
    check = subprocess.run([gz, "sdf", "-k", model_path], capture_output=True, text=True)
    if check.returncode == 0:
        if a.spec_dir:
            print("NOTE: --spec-dir was used, so the schema came from somewhere other than")
            print("      the Gazebo installed here. This check only means something when")
            print("      it is run on the machine that will load the model.")
        print("Gazebo accepts the model. Spawn it with:")
        print(f"    ros2 launch rover_gazebo rover_sim.launch.py spawn_from:=sdf \\")
        print(f"        sdf_file:={model_path}")
        return 0
    print("Gazebo REJECTED the model:")
    print(check.stdout.strip() or check.stderr.strip())
    print("\nFall back to the torque-applied coupling, which needs no schema support:")
    print("    ros2 launch rover_gazebo rover_sim.launch.py coupling:=controller")
    return 1


def _indent(el, level=0):
    pad = "\n" + "  " * level
    if len(el):
        if not (el.text or "").strip():
            el.text = pad + "  "
        for child in el:
            _indent(child, level + 1)
        if not (el.tail or "").strip():
            el.tail = pad
        if not (el[-1].tail or "").strip():
            el[-1].tail = pad
    elif level and not (el.tail or "").strip():
        el.tail = pad


CONFIG = """<?xml version="1.0"?>
<model>
  <name>rover</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>
    Four-wheel-steer rocker-differential rover. Generated by tools/make_sdf.py;
    edit urdf/rover.urdf.xacro and regenerate rather than editing model.sdf.
  </description>
</model>
"""


if __name__ == "__main__":
    sys.exit(main())
