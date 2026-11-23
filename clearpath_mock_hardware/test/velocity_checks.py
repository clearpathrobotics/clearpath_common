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

from collections import deque
import csv
import math
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
import yaml


# Independent expectations for the shipped defaults: minimum, maximum, acceleration.
LIMITS = {
    'dd100': {'linear.x': (-1.3, 1.3, 1.0), 'angular.z': (-4.0, 4.0, 2.0)},
    'dd150': {'linear.x': (-1.3, 1.3, 1.0), 'angular.z': (-4.0, 4.0, 2.0)},
    'do100': {
        'linear.x': (-1.3, 1.3, 1.0),
        'linear.y': (-1.3, 1.3, 1.0),
        'angular.z': (-4.0, 4.0, 2.0),
    },
    'do150': {
        'linear.x': (-1.3, 1.3, 1.0),
        'linear.y': (-1.3, 1.3, 1.0),
        'angular.z': (-4.0, 4.0, 2.0),
    },
    'j100': {'linear.x': (-2.0, 2.0, 2.0), 'angular.z': (-4.0, 4.0, 4.0)},
    'a200': {'linear.x': (-1.0, 1.0, 1.0), 'angular.z': (-1.0, 1.0, 1.0)},
    'a300': {'linear.x': (-2.0, 2.0, 2.0), 'angular.z': (-2.0, 2.0, 4.0)},
    'r100': {
        'linear.x': (-1.3, 1.3, 1.0),
        'linear.y': (-1.3, 1.3, 1.0),
        'angular.z': (-4.0, 4.0, 2.0),
    },
    'w200': {'linear.x': (-5.0, 5.0, 2.5), 'angular.z': (-4.0, 4.0, 2.0)},
}


class VelocityProbe:
    """Publish fresh commands and measure raw controller odometry in one executor."""

    def __init__(self, test, node, executor, namespace, trace):
        self.test = test
        self.node = node
        self.executor = executor
        self.command = [0.0, 0.0, 0.0]
        self.enabled = True
        self.scenario = 'readiness'
        self.expected = [0.0, 0.0, 0.0]
        self.samples = deque(maxlen=2000)
        self.last_stamp = -1
        self.trace = trace
        self.writer = csv.writer(trace)
        self.writer.writerow([
            'scenario', 'receive_time', 'odom_time', 'publishing',
            'command_x', 'command_y', 'command_yaw', 'odom_x', 'odom_y', 'odom_yaw',
            'expected_x', 'expected_y', 'expected_yaw',
        ])
        self.topic = f'/{namespace}/platform/test_cmd_vel'
        self.publisher = node.create_publisher(TwistStamped, self.topic, 10)
        self.subscription = node.create_subscription(
            Odometry, f'/{namespace}/platform/odom', self.receive, qos_profile_sensor_data)
        self.timer = node.create_timer(0.02, self.publish)

    def publish(self):
        if not self.enabled:
            return
        self.test.assertEqual(self.node.count_publishers(self.topic), 1,
                              'Another publisher is driving the controller')
        message = TwistStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.twist.linear.x, message.twist.linear.y, message.twist.angular.z = self.command
        self.publisher.publish(message)

    def receive(self, message):
        now = time.monotonic()
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        velocity = (message.twist.twist.linear.x, message.twist.twist.linear.y,
                    message.twist.twist.angular.z)
        self.test.assertTrue(all(math.isfinite(value) for value in velocity), velocity)
        self.test.assertGreater(stamp, self.last_stamp, 'Stale or duplicate odometry')
        self.test.assertLess(abs(self.node.get_clock().now().nanoseconds - stamp), 1_000_000_000,
                             'Odometry is more than one second old')
        self.last_stamp = stamp
        self.samples.append((now, velocity))
        self.writer.writerow([self.scenario, now, stamp / 1e9, self.enabled,
                              *self.command, *velocity, *self.expected])

    def wait(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        self.test.fail(f'{description}; last odometry: {list(self.samples)[-5:]}')

    def settled(self, target, since):
        recent = [(stamp, values) for stamp, values in self.samples if stamp >= since]
        if len(recent) < 5:
            return False
        # Select a duration, not a fixed count: platform odometry rates differ.
        cutoff = recent[-1][0] - 0.3
        start = next((index for index, (stamp, _) in enumerate(recent) if stamp >= cutoff), 0)
        window = recent[max(0, start - 1):]
        return len(window) >= 5 and window[-1][0] - window[0][0] >= 0.3 and all(
            abs(value - expected) <= 0.02 + 0.03 * abs(expected)
            for _, values in window for value, expected in zip(values, target)
        )

    def stop(self):
        self.enabled = True
        self.command = [0.0, 0.0, 0.0]
        self.expected = [0.0, 0.0, 0.0]
        start = time.monotonic()
        self.wait(lambda: self.settled(self.command, start), 8, 'Robot did not return to rest')

    def close(self):
        self.node.destroy_timer(self.timer)
        self.node.destroy_subscription(self.subscription)
        self.node.destroy_publisher(self.publisher)
        self.trace.flush()


def verify_parameters(test, node, executor, platform, namespace, setup):
    with (setup / 'platform/config/control.yaml').open(encoding='utf-8') as stream:
        generated = yaml.safe_load(stream)
    parameters = generated[namespace]['platform_velocity_controller']['ros__parameters']
    expected = {}
    for axis, (minimum, maximum, acceleration) in LIMITS[platform].items():
        expected.update({
            axis + '.min_velocity': minimum,
            axis + '.max_velocity': maximum,
            axis + '.max_acceleration': acceleration,
        })
    mecanum = 'linear.y' in LIMITS[platform]
    timeout_name = 'reference_timeout' if mecanum else 'cmd_vel_timeout'
    expected[timeout_name] = 0.1 if mecanum else (0.25 if platform == 'w200' else 0.5)
    if not mecanum:
        expected['open_loop'] = False
    if platform == 'w200':
        expected['position_feedback'] = False
    for name, value in expected.items():
        test.assertEqual(parameters[name], value, f'Generated {name}')
    client = node.create_client(
        GetParameters, f'/{namespace}/platform_velocity_controller/get_parameters')
    try:
        test.assertTrue(client.wait_for_service(timeout_sec=5))
        future = client.call_async(GetParameters.Request(names=list(expected)))
        executor.spin_until_future_complete(future, timeout_sec=5)
        test.assertTrue(future.done(), 'Controller parameter query timed out')
        test.assertIsNone(future.exception())
        values = future.result().values
        test.assertEqual(len(values), len(expected))
        for (name, expected_value), actual in zip(expected.items(), values):
            if isinstance(expected_value, bool):
                test.assertEqual(actual.type, ParameterType.PARAMETER_BOOL, name)
                test.assertEqual(actual.bool_value, expected_value, name)
            else:
                test.assertEqual(actual.type, ParameterType.PARAMETER_DOUBLE, name)
                test.assertAlmostEqual(actual.double_value, expected_value, msg=name)
    finally:
        node.destroy_client(client)


def verify_velocity_limits(test, node, executor, platform, namespace, setup, run_directory):
    """Run the same tracking, saturation and stale-command scenarios on every axis."""
    verify_parameters(test, node, executor, platform, namespace, setup)
    passed = []
    axes = {'linear.x': 0, 'linear.y': 1, 'angular.z': 2}
    with (run_directory / 'velocity_trace.csv').open('w', encoding='utf-8', newline='') as trace:
        probe = VelocityProbe(test, node, executor, namespace, trace)
        try:
            probe.wait(lambda: probe.publisher.get_subscription_count() > 0 and
                       len(probe.samples) >= 3, 10, 'Command/odometry topics are not connected')
            for axis, (minimum, maximum, acceleration) in LIMITS[platform].items():
                index = axes[axis]
                for scenario, requested, target in (
                    ('tracking', maximum * 0.4, maximum * 0.4),
                    ('positive_saturation', maximum * 1.5, maximum),
                    ('negative_saturation', minimum * 1.5, minimum),
                    ('timeout', maximum * 0.4, maximum * 0.4),
                ):
                    with test.subTest(platform=platform, axis=axis, scenario=scenario):
                        probe.scenario = f'{axis}/{scenario}'
                        try:
                            probe.stop()
                            command = [0.0, 0.0, 0.0]
                            command[index] = requested
                            expected = [0.0, 0.0, 0.0]
                            expected[index] = target
                            probe.command = command
                            probe.expected = expected
                            start = time.monotonic()
                            probe.wait(lambda: probe.settled(expected, start),
                                       abs(target) / acceleration + 4,
                                       f'{axis} did not converge to {target}')
                            measured = [values[index] for stamp, values in probe.samples
                                        if stamp >= start]
                            tolerance = 0.02 + 0.03 * max(abs(minimum), maximum)
                            test.assertTrue(all(minimum - tolerance <= value <= maximum + tolerance
                                                for value in measured),
                                            f'{axis} exceeded bounds: {min(measured)}, '
                                            f'{max(measured)}; expected [{minimum}, {maximum}]')
                            if scenario == 'timeout':
                                probe.enabled = False
                                probe.expected = [0.0, 0.0, 0.0]
                                expired = time.monotonic()
                                probe.wait(lambda: probe.settled([0.0] * 3, expired),
                                           abs(target) / acceleration + 2,
                                           f'{axis} did not stop after commands expired')
                        finally:
                            probe.stop()
                        passed.append(True)
        finally:
            try:
                probe.stop()
            finally:
                probe.close()
    return len(passed) == len(LIMITS[platform]) * 4
