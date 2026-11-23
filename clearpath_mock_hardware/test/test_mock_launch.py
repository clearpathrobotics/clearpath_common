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

import importlib.util
from pathlib import Path
import sys
from unittest.mock import patch

from launch import LaunchDescription, LaunchService
from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    OpaqueFunction,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
import pytest


GENERATORS = [
    'generate_description',
    'generate_semantic_description',
    'generate_launch',
    'generate_param',
]
SERVICES = [
    ('platform', 'platform-service.launch.py'),
    ('manipulators', 'manipulators-service.launch.py'),
    ('sensors', 'sensors-service.launch.py'),
    ('platform-extras', 'platform-extras-service.launch.py'),
]


@pytest.fixture
def mock_launch():
    path = Path(__file__).parents[1] / 'launch' / 'mock.launch.py'
    spec = importlib.util.spec_from_file_location('mock_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def service_sources(setup_path, trace):
    sources = {}
    for directory, filename in SERVICES:
        sources[setup_path / directory / 'launch' / filename] = (
            'from pathlib import Path\n'
            'from launch import LaunchDescription\n'
            'from launch.actions import OpaqueFunction\n'
            'def record(context):\n'
            f'    with Path({str(trace)!r}).open("a") as stream:\n'
            f'        stream.write({directory + chr(10)!r})\n'
            '    return []\n'
            'def generate_launch_description():\n'
            '    return LaunchDescription([OpaqueFunction(function=record)])\n'
        )
    return sources


def run_launch(mock_launch, setup_path, trace, *, generate=True, fail_at=None):
    sources = service_sources(setup_path, trace)

    def generator_node(*, package, executable, name, output, arguments):
        expected_package = (
            'clearpath_generator_common' if executable in GENERATORS[:2]
            else 'clearpath_generator_mock')
        assert package == expected_package
        code = (
            'from pathlib import Path\n'
            'import sys\n'
            f'with Path({str(trace)!r}).open("a") as stream:\n'
            f'    stream.write({executable + chr(10)!r})\n'
            f'assert sys.argv[1:] == ["-s", {str(setup_path)!r}]\n'
        )
        if executable == fail_at:
            code += 'sys.exit(1)\n'
        elif executable == GENERATORS[-1]:
            for path, source in sources.items():
                code += (
                    f'path = Path({str(path)!r})\n'
                    'path.parent.mkdir(parents=True, exist_ok=True)\n'
                    f'path.write_text({source!r})\n'
                )
        return ExecuteProcess(
            cmd=[sys.executable, '-c', code, *arguments],
            name=name,
            output=output,
        )

    with patch.object(mock_launch, 'Node', side_effect=generator_node):
        description = mock_launch.generate_launch_description()
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        SetLaunchConfiguration('setup_path', str(setup_path)),
        SetLaunchConfiguration('generate', str(generate).lower()),
        description,
    ]))
    return service.run()


def test_generates_before_bringup(mock_launch, tmp_path):
    # A space in the path also checks that generator arguments remain separate.
    setup_path = tmp_path / 'mock setup'
    trace = tmp_path / 'trace'
    assert run_launch(mock_launch, setup_path, trace) == 0
    assert trace.read_text().splitlines() == GENERATORS + [name for name, _ in SERVICES]


def test_uses_existing_generated_files(mock_launch, tmp_path):
    trace = tmp_path / 'trace'
    for path, source in service_sources(tmp_path, trace).items():
        path.parent.mkdir(parents=True)
        path.write_text(source)
    assert run_launch(mock_launch, tmp_path, trace, generate=False) == 0
    assert trace.read_text().splitlines() == [name for name, _ in SERVICES]


@pytest.mark.parametrize('fail_at', GENERATORS)
def test_stops_after_generation_failure(mock_launch, tmp_path, fail_at):
    trace = tmp_path / 'trace'
    assert run_launch(mock_launch, tmp_path, trace, fail_at=fail_at) != 0
    assert trace.read_text().splitlines() == GENERATORS[:GENERATORS.index(fail_at) + 1]


def test_manipulator_timers_keep_scoped_delays():
    seen = []

    def record(context):
        seen.append(LaunchConfiguration('control_delay').perform(context))
        return []

    path = (Path(__file__).parents[2] / 'clearpath_manipulators' /
            'launch' / 'manipulators.launch.py')
    spec = importlib.util.spec_from_file_location('manipulators_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with patch.object(module, 'IncludeLaunchDescription',
                      side_effect=lambda *args, **kwargs: OpaqueFunction(
                          function=record, condition=kwargs.get('condition'))):
        description = module.generate_launch_description()
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        GroupAction([
            SetLaunchConfiguration('control_delay', '0.01'),
            SetLaunchConfiguration('moveit_delay', '0.02'),
            SetLaunchConfiguration('launch_moveit', 'true'),
            description,
        ]),
        TimerAction(period=0.2, actions=[EmitEvent(event=Shutdown(reason='Test complete'))]),
    ]))
    assert service.run() == 0
    assert seen == ['0.01', '0.01', '0.01']
