Generated models land here.

`tools/make_sdf.py` writes `rover/model.sdf` into this directory: a native SDF whose
mimic constraints were written in directly rather than being left to survive the
URDF converter. Nothing here is checked in, because the correct contents depend on
the sdformat version installed on the machine that will run it.
