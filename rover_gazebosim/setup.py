import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rover_gazebosim'


def package_files(directory):
    paths = []
    for root, _, files in os.walk(directory):
        if files:
            paths.append((
                os.path.join('share', package_name, root),
                [os.path.join(root, f) for f in files]
            ))
    return paths

model_data_files = package_files('model')
models_data_files = package_files('models')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/rover_gazebosim/launch', ['launch/spawn_rover.launch.py']),
        ('share/rover_gazebosim/launch', ['launch/slam_and_rover.launch.py']),
        ('share/rover_gazebosim/launch', ['launch/nav2_launch.launch.py']),
        ('share/rover_gazebosim/launch', ['launch/pipeline_launch.launch.py']),
        ('share/rover_gazebosim/launch', ['launch/rtlab_map.launch.py']),
        ('share/rover_gazebosim/world', ['world/rover.world']),
        ('share/rover_gazebosim/world', ['world/husarion_office.sdf']),
        ('share/rover_gazebosim/world', ['world/husarion_world.sdf']),
        # ('share/rover_gazebosim/world', ['world/warehouse.world']),   
        # ('share/rover_gazebosim/urdf', ['urdf/rrbot.urdf']),
        # ('share/rover_gazebosim/config', ['config/parameter_bridge.yaml']),
        ('share/rover_gazebosim/config', ['config/joint_names_mobility urdf adaptation.yaml']),
        ('share/rover_gazebosim/config', ['config/slam_mapping.yaml']),
        ('share/rover_gazebosim/config', ['config/nav2_params.yaml']),
        ('share/rover_gazebosim/config', ['config/rtlab_params.ini']),
        ('share/rover_gazebosim/urdf', ['urdf/rover.urdf']),
        ('share/rover_gazebosim/world', ['world/new.sdf']),
        ('share/rover_gazebosim/urdf', ['urdf/rover.gazebo']),
        ('share/rover_gazebosim/meshes', glob('meshes/*.STL')),
        ('share/rover_gazebosim/meshes', glob('meshes/panther*.STL')),
        ('share/rover_gazebosim/maps', glob('maps/my_map_final.yaml')),
        ('share/rover_gazebosim/maps', glob('maps/my_map_final.pgm')),
        *models_data_files,
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='apratim',
    maintainer_email='anand.apratim336@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'twist_to_stamped=rover_gazebosim.twist_to_stamped:main',
            'cmd_vel_relay=rover_gazebosim.cmd_vel_relay:main',
        ],
    },
)
