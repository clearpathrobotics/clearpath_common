#!/usr/bin/env python3

# Software License Agreement (BSD)
#
# @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of Clearpath Robotics.

from clearpath_generator_common.common import BaseGenerator
from clearpath_generator_common.description.writer import XacroWriter
from clearpath_generator_common.description.mounts import MountDescription
from clearpath_generator_common.description.platform import PlatformDescription
from clearpath_generator_common.description.accessories import AccessoryDescription
from clearpath_generator_common.description.decorations import DecorationsDescription
from clearpath_generator_common.description.sensors import SensorDescription


class DescriptionGenerator(BaseGenerator):
    def __init__(self,
                 setup_path: str = '/etc/clearpath/') -> None:
        super().__init__(setup_path)
        self.xacro_writer = XacroWriter(self.setup_path, self.serial_number)

    def generate(self) -> None:
        # Common macros
        self.xacro_writer.write_include(
            package=self.pkg_clearpath_platform_description.get_name(),
            file='common',
            path='urdf/')

        # Platform
        self.generate_platform()
        self.xacro_writer.write_newline()

        # Decorations
        self.generate_decorations()
        self.xacro_writer.write_newline()

        # Accessories
        self.generate_accessories()
        self.xacro_writer.write_newline()

        # Mounts
        self.generate_mounts()
        self.xacro_writer.write_newline()

        # Sensors
        self.generate_sensors()
        self.xacro_writer.write_newline()

        # Extras
        self.generate_extras()
        self.xacro_writer.write_newline()

        self.xacro_writer.close_file()

    def generate_platform(self) -> None:
        self.platform = self.clearpath_config.platform.get_model()
        platform_description = PlatformDescription(self.platform)

        # Platform macro
        self.xacro_writer.write_comment('Platform')
        self.xacro_writer.write_include(
            package=platform_description.get_package(),
            file=platform_description.get_file(),
            path=platform_description.get_path())
        self.xacro_writer.write_macro(platform_description.get_macro())

    def generate_decorations(self) -> None:
        self.xacro_writer.write_comment('Decorations')
        self.xacro_writer.write_newline()
        decorations = self.clearpath_config.platform.decorations.get_all()

        for decoration in decorations:
            if decoration.get_enabled():
                decoration_description = DecorationsDescription(self.platform, decoration)
                self.xacro_writer.write_include(
                    package=decoration_description.get_package(),
                    file=decoration_description.get_file(),
                    path=decoration_description.get_path())

                self.xacro_writer.write_macro(
                    macro=decoration_description.get_file(),
                    parameters=decoration_description.get_parameters(),
                    blocks=XacroWriter.add_origin(
                        decoration_description.get_xyz(),
                        decoration_description.get_rpy()))
                self.xacro_writer.write_newline()

    def generate_accessories(self) -> None:
        self.xacro_writer.write_comment('Accessories')
        self.xacro_writer.write_newline()
        accessories = self.clearpath_config.accessories.get_all_accessories()

        for accessory in accessories:
            accessory_description = AccessoryDescription(accessory)
            self.xacro_writer.write_include(
                package=accessory_description.get_package(),
                file=accessory_description.get_file(),
                path=accessory_description.get_path())

            self.xacro_writer.write_macro(
                macro=accessory_description.get_file(),
                parameters=accessory_description.get_parameters(),
                blocks=XacroWriter.add_origin(
                    accessory_description.get_xyz(),
                    accessory_description.get_rpy()))
            self.xacro_writer.write_newline()

    def generate_mounts(self) -> None:
        self.xacro_writer.write_comment('Mounts')
        self.xacro_writer.write_newline()
        mounts = self.clearpath_config.mounts.get_all_mounts()
        for mount in mounts:
            mount_description = MountDescription(mount)

            self.xacro_writer.write_comment(
                '{0}'.format(mount_description.get_name())
            )

            self.xacro_writer.write_include(
                package=mount_description.get_package(),
                file=mount_description.get_model(),
                path=mount_description.get_path()
            )

            self.xacro_writer.write_macro(
                macro='{0}'.format(mount_description.get_model()),
                parameters=mount_description.get_parameters(),
                blocks=XacroWriter.add_origin(
                    mount_description.get_xyz(), mount_description.get_rpy())
            )

            self.xacro_writer.write_newline()

    def generate_sensors(self) -> None:
        self.xacro_writer.write_comment('Sensors')
        self.xacro_writer.write_newline()
        sensors = self.clearpath_config.sensors.get_all_sensors()
        for sensor in sensors:
            sensor_description = SensorDescription(sensor)

            self.xacro_writer.write_comment(
                '{0}'.format(sensor_description.get_name())
            )

            self.xacro_writer.write_include(
                package=sensor_description.get_package(),
                file=sensor_description.get_model(),
                path=sensor_description.get_path()
            )

            self.xacro_writer.write_macro(
                macro='{0}'.format(sensor_description.get_model()),
                parameters=sensor_description.get_parameters(),
                blocks=XacroWriter.add_origin(
                    sensor_description.get_xyz(), sensor_description.get_rpy())
            )

            self.xacro_writer.write_newline()

    def generate_extras(self) -> None:
        self.xacro_writer.write_comment('Extras')
        self.xacro_writer.write_newline()
        urdf_extras = self.clearpath_config.platform.extras.urdf
        if urdf_extras:
            self.xacro_writer.write_include(file=urdf_extras)
