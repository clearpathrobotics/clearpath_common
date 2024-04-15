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
import os
import shutil

from clearpath_generator_common.common import BaseGenerator, LaunchFile


class LaunchGenerator(BaseGenerator):

    def __init__(self,
                 setup_path: str = '/etc/clearpath/') -> None:
        super().__init__(setup_path)

        # Remove existing directories
        try:
            shutil.rmtree(self.sensors_launch_path)
        except FileNotFoundError:
            pass

        try:
            shutil.rmtree(self.platform_launch_path)
        except FileNotFoundError:
            pass

        # Make new directories
        os.makedirs(os.path.dirname(self.sensors_launch_path), exist_ok=True)
        os.makedirs(os.path.dirname(self.platform_launch_path), exist_ok=True)

        self.platform_launch_file = LaunchFile(
            name='platform',
            package=self.pkg_clearpath_platform,
            args=[
              ('setup_path', self.setup_path),
              ('use_sim_time', 'false'),
              ('namespace', self.namespace),
            ])

        self.sensors_service_launch_file = LaunchFile(
            name='sensors-service',
            path=self.sensors_launch_path)

        self.platform_service_launch_file = LaunchFile(
            'platform-service',
            path=self.platform_launch_path)

    def generate(self) -> None:
        self.generate_sensors()
        self.generate_platform()

    # This method should be overwritten by the child class
    def generate_sensors(self) -> None:
        raise NotImplementedError()

    def generate_platform(self) -> None:
        raise NotImplementedError()
