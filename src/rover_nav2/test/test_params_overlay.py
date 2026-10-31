import os

import yaml

from rover_nav2.params_overlay import merge, merge_files

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')


def test_mappings_merge_and_values_replace():
    base = {'a': {'x': 1, 'y': [1, 2], 'z': {'p': 1}}, 'b': 2}
    over = {'a': {'y': [3], 'z': {'q': 2}}, 'c': 3}
    assert merge(base, over) == {
        'a': {'x': 1, 'y': [3], 'z': {'p': 1, 'q': 2}}, 'b': 2, 'c': 3}


def test_inputs_not_modified():
    base = {'a': {'x': 1}}
    over = {'a': {'x': 2}}
    merge(base, over)
    assert base == {'a': {'x': 1}} and over == {'a': {'x': 2}}


def test_sim_3d_overlay_only_adds_the_layer(tmp_path):
    base_path = os.path.join(CONFIG, 'nav2_params_sim.yaml')
    out = merge_files(base_path, os.path.join(CONFIG, 'nav2_params_sim_3d.yaml'),
                      str(tmp_path / 'merged.yaml'))
    with open(base_path) as f:
        base = yaml.safe_load(f)
    with open(out) as f:
        merged = yaml.safe_load(f)
    local = merged['local_costmap']['local_costmap']['ros__parameters']
    assert local['plugins'] == ['voxel_layer', 'traversability_layer', 'inflation_layer']
    assert local['traversability_layer']['plugin'] == 'rover_traversability/TraversabilityLayer'
    assert local['traversability_layer']['map_topic'] == '/terrain/local_features'
    # Everything else is the 2D tuning, untouched.
    base_local = base['local_costmap']['local_costmap']['ros__parameters']
    for key, value in base_local.items():
        if key != 'plugins':
            assert local[key] == value, key
    for section in base:
        if section != 'local_costmap':
            assert merged[section] == base[section], section
