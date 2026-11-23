# Copyright 2026 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import os
from pathlib import Path
import shutil
import signal
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from clearpath_config.common.types.platform import Platform
from controller_manager_msgs.srv import (
    ListControllers,
    ListHardwareComponents,
    ListHardwareInterfaces,
)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events.process import ProcessExited
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import launch_config
from launch_ros.actions import SetRemap
import launch_testing
from launch_testing.actions import ReadyToTest
from lifecycle_msgs.msg import State
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from velocity_checks import LIMITS, verify_velocity_limits


PLATFORMS = {'dd100', 'dd150', 'do100', 'do150', 'j100', 'a200', 'a300', 'r100', 'w200'}
GENERATORS = (
    'generate_description',
    'generate_semantic_description',
    'generate_launch',
    'generate_param',
)
FRONT_WHEELS = {'front_left_wheel_joint', 'front_right_wheel_joint'}
ALL_WHEELS = FRONT_WHEELS | {'rear_left_wheel_joint', 'rear_right_wheel_joint'}
DIFF_CONTROLLER = 'diff_drive_controller/DiffDriveController'
OMNI_CONTROLLER = 'mecanum_drive_controller/MecanumDriveController'
EXPECTED = {
    'dd100': (DIFF_CONTROLLER, FRONT_WHEELS),
    'dd150': (DIFF_CONTROLLER, FRONT_WHEELS),
    'do100': (OMNI_CONTROLLER, ALL_WHEELS),
    'do150': (OMNI_CONTROLLER, ALL_WHEELS),
    'j100': (DIFF_CONTROLLER, ALL_WHEELS),
    'a200': (DIFF_CONTROLLER, ALL_WHEELS),
    'a300': (DIFF_CONTROLLER, ALL_WHEELS),
    'r100': (OMNI_CONTROLLER, ALL_WHEELS),
    'w200': (DIFF_CONTROLLER, ALL_WHEELS),
}


def generate_test_description():
    platform = os.environ['MOCK_TEST_PLATFORM']
    if platform not in PLATFORMS:
        raise ValueError(f'No mock platform expectations for {platform}')
    root = Path(os.environ['MOCK_TEST_RUN_ROOT'])
    root.mkdir(parents=True, exist_ok=True)
    run_directory = Path(tempfile.mkdtemp(prefix=f'{platform}-', dir=root))
    setup = run_directory / 'setup'
    setup.mkdir()
    logs = run_directory / 'ros_logs'
    logs.mkdir()
    launch_config.log_dir = str(logs)
    namespace = f'mock_test_{platform}'
    serial = f'{platform}-{"00000" if platform == "a300" else "0000"}'
    (setup / 'robot.yaml').write_text(
        f'serial_number: {serial}\n'
        'version: 0\n'
        'system:\n'
        '  ros2:\n'
        f'    namespace: {namespace}\n'
        'manipulators:\n'
        '  arms: []\n'
        '  lifts: []\n',
        encoding='utf-8',
    )
    print(f'Mock {platform} test artifacts: {run_directory}', flush=True)
    state = {'started': [], 'exited': [], 'passed': False}

    def started(event, context):
        state['started'].append(event)

    def exited(event, context):
        state['exited'].append(event)

    mock_launch = (
        Path(get_package_share_directory('clearpath_mock_hardware')) / 'launch' / 'mock.launch.py'
    )
    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOG_DIR', str(run_directory / 'ros_logs')),
        SetEnvironmentVariable('ROS_HOME', str(run_directory / 'ros_home')),
        RegisterEventHandler(OnProcessStart(on_start=started)),
        RegisterEventHandler(OnProcessExit(on_exit=exited)),
        *([SetRemap(
            src='platform_velocity_controller:~/'
                + ('cmd_vel' if EXPECTED[platform][0] == DIFF_CONTROLLER else 'reference'),
            dst=f'/{namespace}/platform/test_cmd_vel',
        )] if os.environ.get('MOCK_TEST_VELOCITY') == 'true' else []),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(mock_launch)),
            launch_arguments={'setup_path': str(setup), 'generate': 'true'}.items(),
        ),
        ReadyToTest(),
    ]), {
        'platform': platform,
        'namespace': namespace,
        'setup': setup,
        'run_directory': run_directory,
        'state': state,
    }


class TestPlatformBringup(unittest.TestCase):
    """Exercise the real generators, generated launches and controller manager."""

    def test_platform(self, platform, namespace, setup, state, proc_info, run_directory):
        self.assertEqual(set(Platform.ALL) - {'generic'}, PLATFORMS,
                         'Update the platform test matrix when platform support changes')
        self.assertEqual(set(EXPECTED), PLATFORMS)
        self.assertEqual(set(LIMITS), PLATFORMS)
        self.assertEqual('linear.y' in LIMITS[platform], EXPECTED[platform][0] == OMNI_CONTROLLER)
        controller_type, wheels = EXPECTED[platform]
        deadline = time.monotonic() + 90
        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node('mock_bringup_test', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        samples = []
        subscription = node.create_subscription(
            JointState, f'/{namespace}/platform/joint_states',
            samples.append, qos_profile_sensor_data,
        )

        def remaining():
            return max(0.0, deadline - time.monotonic())

        def call(service_type, name, request):
            client = node.create_client(service_type, name)
            try:
                self.assertTrue(client.wait_for_service(timeout_sec=remaining()),
                                f'Timed out waiting for {name}; setup: {setup}')
                future = client.call_async(request)
                executor.spin_until_future_complete(future, timeout_sec=remaining())
                self.assertTrue(future.done(), f'Timed out calling {name}')
                self.assertIsNone(future.exception())
                return future.result()
            finally:
                node.destroy_client(client)

        try:
            manager = f'/{namespace}/controller_manager'
            expected_controllers = {
                'joint_state_broadcaster': 'joint_state_broadcaster/JointStateBroadcaster',
                'platform_velocity_controller': controller_type,
            }
            while True:
                response = call(ListControllers, manager + '/list_controllers',
                                ListControllers.Request())
                controllers = {controller.name: controller for controller in response.controller}
                if all(name in controllers and controllers[name].state == 'active'
                       for name in expected_controllers):
                    break
                self.assertGreater(remaining(), 0, f'Controllers did not activate: {response}')
                executor.spin_once(timeout_sec=min(0.1, remaining()))
            self.assertEqual(set(controllers), set(expected_controllers))
            for name, expected_type in expected_controllers.items():
                self.assertEqual(controllers[name].type, expected_type)

            hardware = call(ListHardwareComponents, manager + '/list_hardware_components',
                            ListHardwareComponents.Request()).component
            self.assertEqual(len(hardware), 1)
            self.assertEqual(hardware[0].name, f'{platform}_hardware')
            self.assertEqual(hardware[0].plugin_name, 'mock_components/GenericSystem')
            self.assertEqual(hardware[0].state.id, State.PRIMARY_STATE_ACTIVE)

            interfaces = call(ListHardwareInterfaces, manager + '/list_hardware_interfaces',
                              ListHardwareInterfaces.Request())
            command = {interface.name: interface for interface in interfaces.command_interfaces}
            expected_interfaces = {f'{joint}/velocity' for joint in wheels}
            self.assertEqual(
                {interface.name for interface in hardware[0].command_interfaces},
                expected_interfaces,
            )
            self.assertTrue(expected_interfaces <= set(command))
            for name in expected_interfaces:
                self.assertTrue(command[name].is_available, name)
                self.assertTrue(command[name].is_claimed, name)

            for name in ('controller_manager', 'robot_state_publisher',
                         'platform_velocity_controller'):
                parameters = call(GetParameters, f'/{namespace}/{name}/get_parameters',
                                  GetParameters.Request(names=['use_sim_time']))
                self.assertEqual(parameters.values[0].type, ParameterType.PARAMETER_BOOL)
                self.assertFalse(parameters.values[0].bool_value)

            while len(samples) < 3 and remaining() > 0:
                executor.spin_once(timeout_sec=min(0.1, remaining()))
            self.assertGreaterEqual(len(samples), 3, 'No sustained joint state publication')
            stamps = []
            for sample in samples[-3:]:
                self.assertTrue(wheels <= set(sample.name), sample.name)
                self.assertEqual(len(sample.velocity), len(sample.name))
                self.assertTrue(all(math.isfinite(value) for value in sample.velocity))
                stamps.append((sample.header.stamp.sec, sample.header.stamp.nanosec))
            self.assertGreater(stamps[-1], stamps[0], 'Joint state timestamps did not advance')
            self.assertFalse(any('manipulators' in ns
                                 for _, ns in node.get_node_names_and_namespaces()))

            for relative_path in (
                'robot.urdf.xacro', 'robot.srdf.xacro', 'robot.srdf',
                'platform/config/control.yaml',
                'platform/launch/platform-service.launch.py',
                'manipulators/launch/manipulators-service.launch.py',
                'sensors/launch/sensors-service.launch.py',
                'platform-extras/launch/platform-extras-service.launch.py',
            ):
                path = setup / relative_path
                self.assertTrue(path.is_file(), str(path))
                self.assertGreater(path.stat().st_size, 0, str(path))

            spawners = [
                event for event in state['started'] if event.process_name.startswith('spawner-')
            ]
            self.assertEqual(len(spawners), 1)
            proc_info.assertWaitForShutdown(spawners[0].action, timeout=remaining())
            exits = list(state['exited'])
            for generator in GENERATORS:
                matches = [
                    event for event in exits if event.process_name.startswith(generator + '-')
                ]
                self.assertEqual(len(matches), 1, f'{generator} did not finish exactly once')
                self.assertEqual(matches[0].returncode, 0)
            for event in exits:
                self.assertTrue(
                    event.process_name.startswith(GENERATORS + ('spawner-',)),
                    f'Runtime process exited before shutdown: '
                    f'{event.process_name}: {event.returncode}',
                )
                self.assertEqual(event.returncode, 0, event.process_name)
            if os.environ.get('MOCK_TEST_VELOCITY') == 'true':
                state['passed'] = verify_velocity_limits(
                    self, node, executor, platform, namespace, setup, run_directory)
                self.assertTrue(state['passed'], 'One or more velocity scenarios failed')
            else:
                state['passed'] = True
        finally:
            executor.shutdown()
            node.destroy_subscription(subscription)
            node.destroy_node()
            context.shutdown()


@launch_testing.post_shutdown_test()
class TestPlatformShutdown(unittest.TestCase):
    """Keep failed setups and logs, and remove successful runs after shutdown."""

    def test_shutdown(self, proc_info, state, run_directory):
        started = list(state['started'])
        self.assertTrue(started, 'No processes were launched')
        for event in started:
            result = proc_info[event.action]
            self.assertIsInstance(result, ProcessExited, f'{event.process_name} did not terminate')
            allowed = (0,) if event.process_name.startswith(GENERATORS + ('spawner-',)) else (
                0, -signal.SIGINT,
            )
            self.assertIn(result.returncode, allowed, event.process_name)
        if state['passed']:
            shutil.rmtree(run_directory)
