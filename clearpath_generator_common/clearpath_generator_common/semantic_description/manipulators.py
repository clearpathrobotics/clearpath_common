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
from clearpath_config.manipulators.types.arms import Franka
from clearpath_config.manipulators.types.grippers import FrankaGripper
from clearpath_config.manipulators.types.manipulator import (
    BaseManipulator,
    ManipulatorPose
)


class ManipulatorPoseMacro():

    class BasePoseMacro():
        NAME = 'name'
        GROUP_STATE = 'group_state'
        JOINT_POSITIONS = 'joint_positions'

        def __init__(self, manipulator: BaseManipulator, pose: ManipulatorPose) -> None:
            self.manipulator = manipulator
            self.pose = pose
            # Extract Joint Values
            joints_string = [f'{joint:.4f}' for joint in self.pose.joints]
            # Macro
            self.macro = f'{self.manipulator.MANIPULATOR_MODEL}_group_state'
            # Parameters
            self.parameters = {
                self.NAME: self.manipulator.name,
                self.GROUP_STATE: self.pose.name,
                self.JOINT_POSITIONS: f'${{[{", ".join(joints_string)}]}}'
            }
            # Blocks
            self.blocks = None

    class FrankaPoseMacro(BasePoseMacro):

        def __init__(self, manipulator, pose):
            super().__init__(manipulator, pose)
            self.parameters[Franka.ARM_ID] = self.manipulator.arm_id

    MODEL = {
        Franka.MANIPULATOR_MODEL: FrankaPoseMacro,
        FrankaGripper.MANIPULATOR_MODEL: FrankaPoseMacro
    }

    def __new__(cls, manipulator: BaseManipulator, pose: ManipulatorPose) -> BaseManipulator:
        return ManipulatorPoseMacro.MODEL.setdefault(
            manipulator.MANIPULATOR_MODEL,
            ManipulatorPoseMacro.BasePoseMacro)(manipulator, pose)


class ManipulatorSemanticDescription():

    class BaseSemanticDescription():
        pkg_clearpath_manipulator_descritpion = 'clearpath_manipulators_description'

        NAME = 'name'

        def __init__(self, manipulator: BaseManipulator) -> None:
            self.manipulator = manipulator
            self.package = self.pkg_clearpath_manipulator_descritpion
            self.path = 'srdf/' + manipulator.get_manipulator_type()

            self.parameters = {
                self.NAME: manipulator.name,
            }

        @property
        def name(self) -> str:
            return self.manipulator.name

        @property
        def model(self) -> str:
            return self.manipulator.MANIPULATOR_MODEL

    class FrankaSemanticDescription(BaseSemanticDescription):

        def __init__(self, manipulator):
            super().__init__(manipulator)
            self.parameters[self.NAME] = f'{manipulator.name}'
            self.parameters[Franka.ARM_ID] = f'{manipulator.arm_id}'

    MODEL = {
        Franka.MANIPULATOR_MODEL: FrankaSemanticDescription,
        FrankaGripper.MANIPULATOR_MODEL: FrankaSemanticDescription
    }

    def __new__(cls, manipulator: BaseManipulator) -> BaseManipulator:
        return ManipulatorSemanticDescription.MODEL.setdefault(
            manipulator.MANIPULATOR_MODEL,
            ManipulatorSemanticDescription.BaseSemanticDescription)(manipulator)
