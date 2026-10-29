import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rover_testing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch', 'tests'),
            glob('launch/tests/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nishant',
    maintainer_email='software.mrt.iitb@gmail.com',
    description='Benchmark test cases and a test-runner UI for the simulated rover stack.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'goal_benchmark = rover_testing.goal_benchmark:main',
            'drive_benchmark = rover_testing.drive_benchmark:main',
            'make_test_worlds = rover_testing.worldgen:main',
            'make_reference_map = rover_testing.reference_map:main',
            'test_runner_ui = rover_testing.test_runner_ui:main',
        ],
    },
)
