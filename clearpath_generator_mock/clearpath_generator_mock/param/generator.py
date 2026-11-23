# BSD 3-Clause License
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2026, Clearpath Robotics, Inc., All rights reserved.
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
from clearpath_generator_common.param.generator import ParamGenerator


class MockParamGenerator(ParamGenerator):

    def generate_sensors(self) -> None:
        # Sensors are not mocked yet.
        pass
