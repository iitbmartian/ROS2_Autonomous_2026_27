import os

import pytest

from rover_traversability import physical_params

HERE = os.path.dirname(os.path.abspath(__file__))
FIX = os.path.join(HERE, 'fixtures')
SHIPPED = os.path.join(HERE, '..', 'config', 'rover_physical.yaml')


def test_values_and_placeholders():
    values, ph = physical_params.load(os.path.join(FIX, 'physical_two_placeholders.yaml'))
    assert values['wheelbase'] == 1.0
    assert values['mass'] == 40.0
    assert values['cg_offset_y'] == -0.02
    assert ph == ['cg_height', 'step_max']


def test_missing_key():
    with pytest.raises(ValueError, match="'mass'"):
        physical_params.load(os.path.join(FIX, 'physical_missing_mass.yaml'))


def test_bad_number():
    with pytest.raises(ValueError, match="'track'"):
        physical_params.load(os.path.join(FIX, 'physical_bad_number.yaml'))


def test_shipped_file():
    values, ph = physical_params.load(SHIPPED)
    assert values['cg_height'] == 0.45
    assert values['track'] == 0.436
    assert ph == ['cg_height', 'cg_offset_x', 'cg_offset_y', 'mass', 'ground_clearance',
                  'rocker_max', 'step_max', 'stability_margin_min', 'roughness_max']
    assert 'cg_height' in physical_params.placeholder_warning(ph)
    assert physical_params.placeholder_warning([]) == ''
