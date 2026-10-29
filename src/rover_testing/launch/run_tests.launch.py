"""Run the rover_testing benchmarks from a UI.

    ros2 launch rover_testing run_tests.launch.py

Starts only the test-runner window, no simulation. Tick the test cases to run,
set the localization tolerance, and press Run: each selected test (a file in
launch/tests/, listed in config/test_cases.yaml) launches the full pipeline,
runs, shuts down, and its result lands in the table before the next one starts.
Results are also saved under ~/.ros/rover_testing/<timestamp>/.
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    ui = Node(package="rover_testing", executable="test_runner_ui", name="test_runner_ui",
              output="screen")
    return LaunchDescription([
        ui,
        # Closing the window ends the launch.
        RegisterEventHandler(OnProcessExit(target_action=ui, on_exit=[Shutdown()])),
    ])
