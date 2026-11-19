"""The real-rover launch (rover_3d.launch.py) runs the stack simulation proved.

Every launch argument of pipeline_3d.launch.py that changes the navigation stack's behaviour must
exist in rover_3d.launch.py with the same default, unless it is listed below as simulation-only
or as a deliberate hardware difference. On 2026-11-18 global_fine_step was missing on the rover
side, so the rover would have planned on the coarse global step that failed the bars in sim.
"""
import os
import re

SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
LAUNCH = os.path.join(SRC, 'rover_bringup', 'launch')

# Simulator, sensor model and benchmark plumbing, no counterpart on the rover.
SIM_ONLY = {'lidar_model', 'lidar_inverted', 'lidar_tilt', 'lidar_xyz', 'drive', 'mapping_params',
            'terrain_rviz'}
# Deliberate differences, each with its reason.
DIFFERENT = {
    # Every simulation run passes governor_u_free:=0.3 (benchmark and bag scripts), the
    # pipeline's own default of 0.0 is the pre-v1 value. The rover starts at the tuned 0.3.
    'governor_u_free': ('0.0', '0.3'),
}


def defaults(name):
    with open(os.path.join(LAUNCH, name)) as f:
        s = f.read()
    return dict(re.findall(r"DeclareLaunchArgument\(\s*'([a-z_]+)',\s*default_value='([^']*)'", s))


def test_real_launch_declares_every_behaviour_argument_with_the_sim_default():
    sim, real = defaults('pipeline_3d.launch.py'), defaults('rover_3d.launch.py')
    assert sim and real
    for name, value in sim.items():
        if name in SIM_ONLY:
            continue
        assert name in real, f'{name} missing from rover_3d.launch.py'
        if name in DIFFERENT:
            assert (value, real[name]) == DIFFERENT[name], name
        else:
            assert real[name] == value, f'{name}: sim {value!r}, real {real[name]!r}'


def test_real_launch_passes_the_mapping_arguments_on():
    with open(os.path.join(LAUNCH, 'rover_3d.launch.py')) as f:
        s = f.read()
    for name in ('global_fine_step', 'mask_tip_margin', 'fill_holes'):
        assert f"'{name}': arg('{name}')" in s, name
