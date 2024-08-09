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

from clearpath_generator_common.common import BaseGenerator
from clearpath_generator_common.param.manipulators import ManipulatorParam
from clearpath_generator_common.param.platform import PlatformParam


class ParamGenerator(BaseGenerator):
    def __init__(self,
                 setup_path: str = '/etc/clearpath/') -> None:
        super().__init__(setup_path)

        # Remove existing directories
        try:
            shutil.rmtree(self.sensors_params_path)
        except FileNotFoundError:
            pass

        try:
            shutil.rmtree(self.platform_params_path)
        except FileNotFoundError:
            pass

        try:
            shutil.rmtree(self.manipulators_params_path)
        except FileNotFoundError:
            pass

        # Make new directories
        os.makedirs(os.path.dirname(self.sensors_params_path), exist_ok=True)
        os.makedirs(os.path.dirname(self.platform_params_path), exist_ok=True)
        os.makedirs(os.path.dirname(self.manipulators_params_path), exist_ok=True)

    def generate(self) -> None:
        self.generate_sensors()
        self.generate_platform()
        self.generate_manipulators()

    def generate_sensors(self) -> None:
        raise NotImplementedError()

    def generate_platform(self) -> None:
        for param in PlatformParam.PARAMETERS:
            platform_param = PlatformParam(
                param,
                self.clearpath_config,
                self.platform_params_path)
            platform_param.generate_parameters()
            platform_param.generate_parameter_file()

    def generate_manipulators(self) -> None:
        for param in ManipulatorParam.PARAMETERS:
            manipulator_param = ManipulatorParam(
                param,
                self.clearpath_config,
                self.manipulators_params_path)
            manipulator_param.generate_parameters()
            manipulator_param.generate_parameter_file()
