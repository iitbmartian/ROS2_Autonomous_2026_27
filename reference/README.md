# The original export, untouched

`urdf_full_withdifferential` is the SolidWorks URDF exporter's output, copied here
verbatim so the generated model can always be traced back to it. Nothing reads from this
folder at runtime; `tools/derive_ratios.py` and `tools/generate_description.py` read it
at generation time, and only for that.

The exporter log is truncated to its first few hundred lines. The complete one is in the
Unity project, at `unity/Assets/urdf_full_withdifferential/export.log`.

The ROS 1 launch files the exporter wrote are kept for the record. They do not work:
they target Gazebo Classic under ROS 1, and they spawn the URDF as exported, which
Gazebo will not load for the reasons set out in `rover_gazebo/doc/DESIGN.md`.
