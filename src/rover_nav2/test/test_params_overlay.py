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


def test_sim_3d_overlay_swaps_in_the_terrain_layer(tmp_path):
    base_path = os.path.join(CONFIG, 'nav2_params_sim.yaml')
    out = merge_files(base_path, os.path.join(CONFIG, 'nav2_params_sim_3d.yaml'),
                      str(tmp_path / 'merged.yaml'))
    with open(base_path) as f:
        base = yaml.safe_load(f)
    with open(out) as f:
        merged = yaml.safe_load(f)
    local = merged['local_costmap']['local_costmap']['ros__parameters']
    assert local['plugins'] == ['traversability_layer', 'inflation_layer']
    assert local['traversability_layer']['cost_model'] == 'utilisation'
    assert local['traversability_layer']['map_topic'] == '/terrain/local_features'
    glob = merged['global_costmap']['global_costmap']['ros__parameters']
    assert glob['plugins'] == ['traversability_layer', 'inflation_layer']
    assert glob['traversability_layer']['map_topic'] == '/terrain/global_features'
    # The 2D footprint and inflation tuning carry over untouched.
    base_local = base['local_costmap']['local_costmap']['ros__parameters']
    for key in ('footprint', 'footprint_padding', 'inflation_layer', 'resolution', 'global_frame'):
        assert local[key] == base_local[key], key
    cm = merged['collision_monitor']['ros__parameters']
    assert cm['observation_sources'] == ['terrain_hazards']
    assert cm['terrain_hazards']['topic'] == '/terrain/hazard_points'
    # Everything else is the 2D tuning, untouched.
    for section in base:
        if section not in ('local_costmap', 'global_costmap', 'collision_monitor'):
            assert merged[section] == base[section], section


def test_swerve_overlay_chains_on_the_3d_overlay(tmp_path):
    out = merge_files(os.path.join(CONFIG, 'nav2_params_sim.yaml'),
                      [os.path.join(CONFIG, 'nav2_params_sim_3d.yaml'),
                       os.path.join(CONFIG, 'nav2_params_sim_3d_swerve.yaml')],
                      str(tmp_path / 'merged.yaml'))
    with open(out) as f:
        merged = yaml.safe_load(f)
    fp = merged['controller_server']['ros__parameters']['FollowPath']
    assert fp['motion_model'] == 'Omni'
    assert 'TerrainCritic' in fp['critics'] and 'PathAngleCritic' not in fp['critics']
    assert fp['TerrainCritic']['enabled'] is True
    assert 'vx_max' in fp and 'batch_size' in fp           # the 2D MPPI tuning carries over
    assert merged['planner_server']['ros__parameters']['GridBased']['plugin'] == \
        'nav2_smac_planner::SmacPlanner2D'
    # The 3D costmaps from the first overlay survive the second.
    local = merged['local_costmap']['local_costmap']['ros__parameters']
    assert local['traversability_layer']['cost_model'] == 'utilisation'

