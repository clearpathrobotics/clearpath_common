# BSD 3-Clause License
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2026, Clearpath Robotics, Inc., All rights reserved.
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
import os

from clearpath_generator_common.common import LaunchFile, Package
from clearpath_generator_common.launch.generator import LaunchGenerator
from clearpath_generator_common.launch.writer import LaunchWriter


class MockLaunchGenerator(LaunchGenerator):

    def __init__(self, setup_path: str = '/etc/clearpath/') -> None:
        super().__init__(setup_path)
        self.platform_launch_file.args.append(('use_mock_hardware', 'true'))
        self.manipulators_launch_file.args.append(('use_mock_hardware', 'true'))

    def generate_sensors(self) -> None:
        # Sensors are not mocked yet; the empty service file keeps the bring-up layout intact.
        sensors_service_launch_writer = LaunchWriter(self.sensors_service_launch_file)
        sensors_service_launch_writer.generate_file()

    def generate_platform(self) -> None:
        platform_service_launch_writer = LaunchWriter(self.platform_service_launch_file)
        platform_service_launch_writer.add(self.platform_launch_file)
        platform_service_launch_writer.generate_file()

        os.makedirs(os.path.dirname(self.platform_extras_launch_path), exist_ok=True)
        platform_extras_service_launch_writer = LaunchWriter(
            self.platform_extras_service_launch_file)
        platform_extras_service_launch_writer.add(self.platform_extras_launch_file)

        for launch in self.clearpath_config.platform.extras.launch:
            extra_launch = LaunchFile(
                name=(os.path.basename(launch.path)).split('.')[0],
                path=os.path.dirname(launch.path),
                package=Package(launch.package),
                args=[
                    (key, launch.args[key]) for key in launch.args
                ]
            )
            platform_extras_service_launch_writer.add(extra_launch)

        platform_extras_service_launch_writer.generate_file()

    def generate_manipulators(self) -> None:
        manipulators_service_launch_writer = LaunchWriter(self.manipulators_service_launch_file)
        if self.clearpath_config.manipulators.get_all_manipulators():
            manipulators_service_launch_writer.add(self.manipulators_launch_file)
        manipulators_service_launch_writer.generate_file()
