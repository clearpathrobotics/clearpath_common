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
from typing import List

from clearpath_config.manipulators.types.arms import (
    BaseArm,
    KinovaGen3Dof6,
    KinovaGen3Dof7,
    KinovaGen3Lite,
    UniversalRobots
)
from clearpath_config.manipulators.types.grippers import (
    Kinova2FLite,
    Robotiq2F140,
    Robotiq2F85
)
from clearpath_config.manipulators.types.lifts import (
    BaseLift,
    Ewellix
)
from clearpath_config.manipulators.types.manipulator import BaseManipulator


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
            self.parameters.update(manipulator.get_urdf_parameters())

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

    class ArmDescription(BaseDescription):
        IP = BaseArm.IP_ADDRESS
        PORT = BaseArm.IP_PORT

        def __init__(self, arm: BaseArm) -> None:
            super().__init__(arm)
            self.parameters[self.IP] = arm.ip
            self.parameters[self.PORT] = arm.port

    class KinovaArmDescription(ArmDescription):
        IP = 'robot_ip'
        PORT = 'port'
        GRIPPER_JOINT = 'gripper_joint_name'
        GRIPPER_COMM = 'use_internal_bus_gripper_comm'
        GRIPPER_NAMES = {
            Kinova2FLite.get_manipulator_model(): '_right_finger_bottom_joint',
            Robotiq2F140.get_manipulator_model(): '_finger_joint',
            Robotiq2F85.get_manipulator_model(): '_robotiq_85_left_knuckle_joint',
        }

        def __init__(self, arm: BaseArm) -> None:
            super().__init__(arm)
            if arm.gripper:
                self.parameters[self.GRIPPER_COMM] = True
                self.parameters[self.GRIPPER_JOINT] = (
                    arm.gripper.name + self.GRIPPER_NAMES[arm.gripper.get_manipulator_model()])

    class UniversalRobotsDescription(ArmDescription):
        UR_TYPE = 'ur_type'
        IP = UniversalRobots.IP_ADDRESS
        PORT = UniversalRobots.IP_PORT

        def __init__(self, arm: BaseArm) -> None:
            super().__init__(arm)
            self.parameters.pop(self.PORT)

    class LiftDescription(BaseDescription):

        def __init__(self, lift: BaseLift) -> None:
            super().__init__(lift)

    class EwellixDescription(LiftDescription):

        def __init__(self, lift: BaseLift) -> None:
            super().__init__(lift)
            self.parameters.update(lift.get_urdf_parameters())

    MODEL = {
        KinovaGen3Dof6.MANIPULATOR_MODEL: KinovaArmDescription,
        KinovaGen3Dof7.MANIPULATOR_MODEL: KinovaArmDescription,
        KinovaGen3Lite.MANIPULATOR_MODEL: KinovaArmDescription,
        UniversalRobots.MANIPULATOR_MODEL: UniversalRobotsDescription,
        Ewellix.MANIPULATOR_MODEL: EwellixDescription,
    }

    def __new__(cls, manipulator: BaseManipulator) -> BaseManipulator:
        return ManipulatorDescription.MODEL.setdefault(
            manipulator.MANIPULATOR_MODEL,
            ManipulatorDescription.BaseDescription)(manipulator)
