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
from clearpath_config.manipulators.types.arms import (
    KinovaGen3Dof6,
    KinovaGen3Dof7,
    KinovaGen3Lite,
)
from clearpath_config.manipulators.types.manipulator import BaseManipulator

from typing import List


class ManipulatorDescription():

    class BaseDescription():
        pkg_clearpath_manipulators_description = 'clearpath_manipulators_description'

        NAME = 'name'
        PARENT = 'parent_link'
        XYZ = 'xyz'
        RPY = 'rpy'

        def __init__(self, manipulator: BaseManipulator) -> None:
            self.manipulator = manipulator
            self.package = self.pkg_clearpath_manipulators_description
            self.path = 'urdf/' + manipulator.get_manipulator_type()

            self.parameters = {
                self.NAME: manipulator.name,
                self.PARENT: manipulator.parent,
            }

        @property
        def name(self) -> str:
            return self.manipulator.name

        @property
        def model(self) -> str:
            return self.manipulator.MANIPULATOR_MODEL

        @property
        def xyz(self) -> List[float]:
            return self.manipulator.xyz

        @property
        def rpy(self) -> List[float]:
            return self.manipulator.rpy

    MODEL = {
        KinovaGen3Dof6.MANIPULATOR_MODEL: BaseDescription,
        KinovaGen3Dof7.MANIPULATOR_MODEL: BaseDescription,
        KinovaGen3Lite.MANIPULATOR_MODEL: BaseDescription,
    }

    def __new__(cls, manipulator: BaseManipulator) -> BaseManipulator:
        return ManipulatorDescription.MODEL.setdefault(
            manipulator.MANIPULATOR_MODEL,
            ManipulatorDescription.BaseDescription)(manipulator)
