"""Deep-merge an overlay Nav2 parameter file over a base one.

The 3D stack's parameter files hold only what differs from the 2D files. Mappings merge key by
key, recursively. Any other value in the overlay (scalar or list) replaces the base value.
"""

import copy

import yaml


def merge(base, overlay):
    """Return base with overlay merged in. Neither argument is modified."""
    out = copy.deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def merge_files(base_path, overlay_paths, out_path):
    """Write base with each overlay merged over it in order, return out_path. overlay_paths is
    one path or a list of paths."""
    if isinstance(overlay_paths, str):
        overlay_paths = [overlay_paths]
    with open(base_path) as f:
        merged = yaml.safe_load(f) or {}
    for path in overlay_paths:
        with open(path) as f:
            merged = merge(merged, yaml.safe_load(f) or {})
    with open(out_path, 'w') as f:
        f.write(f"# Generated: {', '.join(overlay_paths)} merged over {base_path}. Do not edit.\n")
        yaml.safe_dump(merged, f, sort_keys=False, default_flow_style=None)
    return out_path
