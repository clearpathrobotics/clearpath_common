#!/usr/bin/env python3

# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.
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
from clearpath_config.common.types.platform import Platform
from clearpath_config.platform.can import (
    PhysicalCANAdapter,
    SerialCANAdapter,
    VirtualCANAdapter,
)
from clearpath_generator_common.bash.writer import BashWriter
from clearpath_generator_common.common import BaseGenerator, BashFile

PLATFORMS = [
    Platform.A300,
    Platform.DD100,
    Platform.DD150,
    Platform.DO100,
    Platform.DO150,
    Platform.R100,
]


class VirtualCANGenerator(BaseGenerator):

    ROS_DISTRO_PATH = '/opt/ros/humble/'

    def generate(self) -> None:
        # Generate vcan start up script
        self.generate_vcan_start()

    def generate_vcan_start(self) -> None:
        # Generate vcan start up script
        vcan_start = BashFile(filename='vcan-start', path=self.setup_path)
        bash_writer = BashWriter(vcan_start)

        for adapter in self.clearpath_config.platform.can_adapters.get_all():
            if adapter.TYPE == VirtualCANAdapter.TYPE:
                bash_writer.write(
                    f'/bin/bash -e /usr/sbin/clearpath-vcan-bridge '
                    f'-t {adapter.TYPE} '
                    f'-p {adapter.port} '
                    f'-s {adapter.serial_dev} '
                    f'-c {adapter.can_dev} '
                    f'-b {adapter.baud}'
                )
            if adapter.TYPE == SerialCANAdapter.TYPE:
                bash_writer.write(
                    f'/bin/bash -e /usr/sbin/clearpath-vcan-bridge '
                    f'-t {adapter.TYPE} '
                    f'-s {adapter.serial_dev} '
                    f'-c {adapter.can_dev} '
                    f'-b {adapter.baud}'
                )
            if adapter.TYPE == PhysicalCANAdapter.TYPE:
                bash_writer.write(
                    f'/bin/bash -e /usr/sbin/clearpath-vcan-bridge '
                    f'-t {adapter.TYPE} '
                    f'-c {adapter.can_dev} '
                    f'-b {adapter.baud}'
                )
        if not self.clearpath_config.platform.can_adapters.get_all():
            bash_writer.add_echo(
                'No vcan bridge required.' +
                'If this was launched as a service then the service will now end.'
            )

        bash_writer.close()
