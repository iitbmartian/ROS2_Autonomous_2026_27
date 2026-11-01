"""Loader for config/rover_physical.yaml, the single source of physical parameters.

Same rules as the C++ loader in core/physical_params.cpp: every key must exist and be a number,
and a key is a placeholder when its own line carries the "# PLACEHOLDER" marker.
"""

import os
import re

import yaml

KEYS = (
    'wheelbase', 'track', 'wheel_radius', 'wheel_width', 'footprint_length', 'footprint_width',
    'cg_height', 'cg_offset_x', 'cg_offset_y', 'mass', 'ground_clearance', 'rocker_max',
    'step_max', 'stability_margin_min', 'roughness_max',
)

_KEY_LINE = re.compile(r'^\s*([A-Za-z_][A-Za-z0-9_]*)\s*:.*#.*\bPLACEHOLDER\b')


def default_path():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory('rover_traversability'), 'config', 'rover_physical.yaml')


def load(path=None):
    """Return (values, placeholders): a dict of floats and the placeholder keys in file order."""
    path = path or default_path()
    with open(path) as f:
        text = f.read()
    data = yaml.safe_load(text) or {}
    values = {}
    for key in KEYS:
        if key not in data:
            raise ValueError(f"rover_physical: {path}: missing key '{key}'")
        v = data[key]
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise ValueError(f"rover_physical: {path}: key '{key}' is not a number")
        values[key] = float(v)
    placeholders = []
    for line in text.splitlines():
        m = _KEY_LINE.search(line)
        if m and m.group(1) in KEYS:
            placeholders.append(m.group(1))
    return values, placeholders


def placeholder_warning(placeholders):
    if not placeholders:
        return ''
    return ('rover_physical.yaml: placeholder values in use, not measured on the rover: '
            + ' '.join(placeholders))
