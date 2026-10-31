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


def merge_files(base_path, overlay_path, out_path):
    """Write the merge of two YAML files to out_path and return out_path."""
    with open(base_path) as f:
        base = yaml.safe_load(f) or {}
    with open(overlay_path) as f:
        overlay = yaml.safe_load(f) or {}
    with open(out_path, 'w') as f:
        f.write(f'# Generated: {overlay_path} merged over {base_path}. Do not edit.\n')
        yaml.safe_dump(merge(base, overlay), f, sort_keys=False, default_flow_style=None)
    return out_path
