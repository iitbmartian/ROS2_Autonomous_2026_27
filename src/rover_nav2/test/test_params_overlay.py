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
    # Configuration space: utilisation verdicts already place the whole rover, so both costmaps
    # get a point footprint, inflation only past its 0.071 m circumscribed radius, no dilation.
    base_local = base['local_costmap']['local_costmap']['ros__parameters']
    for cm_params in (local, glob):
        assert cm_params['footprint'].startswith('[[0.05, 0.05]')
        assert cm_params['footprint_padding'] == 0.0
        assert cm_params['inflation_layer']['inflation_radius'] == 0.08
        # The base's inflation plugin type and the rest of its keys still merge in.
        assert cm_params['inflation_layer']['plugin'] == 'nav2_costmap_2d::InflationLayer'
        assert cm_params['traversability_layer']['dilate_terrain_lethal'] is False
        assert cm_params['traversability_layer']['braking_from_physical_footprint'] is True
    for key in ('resolution', 'global_frame'):
        assert local[key] == base_local[key], key
    # The 2D file itself keeps its full footprint.
    assert base_local['footprint'].startswith('[[0.55, 0.45]')
    cm = merged['collision_monitor']['ros__parameters']
    assert cm['observation_sources'] == ['terrain_hazards']
    assert cm['polygons'] == ['CentreApproach']
    assert cm['terrain_hazards']['topic'] == '/terrain/hazard_points'
    # bt_navigator differs only in the tree, the 3D one keeps a valid path instead of 1 Hz replans.
    bt, base_bt = merged['bt_navigator']['ros__parameters'], base['bt_navigator']['ros__parameters']
    assert bt['default_nav_to_pose_bt_xml'].endswith('navigate_to_pose_ackermann_3d.xml')
    diff = {k for k in set(bt) | set(base_bt) if bt.get(k) != base_bt.get(k)}
    assert diff <= {'default_nav_to_pose_bt_xml', 'plugin_lib_names'}, diff
    # Everything else is the 2D tuning, untouched.
    for section in base:
        if section not in ('local_costmap', 'global_costmap', 'collision_monitor', 'bt_navigator'):
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

