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

from functools import partial

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def on_generation_exit(event, context, *, next_action):
    if event.returncode != 0:
        raise RuntimeError(
            f'Mock hardware generation failed: {event.action.name} '
            f'exited with code {event.returncode}')
    return [next_action]


def generate_launch_description():
    setup_path = LaunchConfiguration('setup_path')
    generate = LaunchConfiguration('generate')

    bringup = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                setup_path, directory, 'launch', filename,
            ]))
        )
        for directory, filename in [
            ('platform', 'platform-service.launch.py'),
            ('manipulators', 'manipulators-service.launch.py'),
            ('sensors', 'sensors-service.launch.py'),
            ('platform-extras', 'platform-extras-service.launch.py'),
        ]
    ])

    generators = [
        Node(
            package=package,
            executable=executable,
            name=executable,
            output='screen',
            arguments=['-s', setup_path],
        )
        for package, executable in [
            ('clearpath_generator_common', 'generate_description'),
            ('clearpath_generator_common', 'generate_semantic_description'),
            ('clearpath_generator_mock', 'generate_launch'),
            ('clearpath_generator_mock', 'generate_param'),
        ]
    ]

    handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=generator,
            on_exit=partial(on_generation_exit, next_action=next_action),
        ))
        for generator, next_action in zip(generators, generators[1:] + [bringup])
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'setup_path',
            default_value=[EnvironmentVariable('HOME'), '/clearpath/'],
            description='Clearpath setup path containing robot.yaml',
        ),
        DeclareLaunchArgument(
            'generate',
            default_value='true',
            choices=['true', 'false'],
            description='Generate descriptions, parameters and launch files',
        ),
        GroupAction(
            actions=handlers + [generators[0]],
            condition=IfCondition(generate),
        ),
        GroupAction(actions=[bringup], condition=UnlessCondition(generate)),
    ])
