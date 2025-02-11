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
from clearpath_generator_common.common import BaseGenerator
from clearpath_generator_common.description.writer import XacroWriter
from clearpath_generator_common.semantic_description.manipulators import (
    ManipulatorPoseMacro,
    ManipulatorSemanticDescription
)


class SemanticDescriptionGenerator(BaseGenerator):

    def __init__(self, setup_path: str = '/etc/clearpath/') -> None:
        super().__init__(setup_path)
        self.xacro_writer = XacroWriter(self.setup_path, self.serial_number, '.srdf.xacro')

    def generate(self) -> None:
        # Arms
        self.generate_arms()
        self.xacro_writer.write_newline()

        # Grippers
        self.generate_grippers()
        self.xacro_writer.write_newline()

        # Lifts
        self.generate_lifts()
        self.xacro_writer.write_newline()

        self.xacro_writer.close_file()
        print(f'Generated {self.xacro_writer.file_path}robot.srdf.xacro')

    def generate_arms(self) -> None:
        self.xacro_writer.write_comment('Arms')
        self.xacro_writer.write_newline()
        arms = self.clearpath_config.manipulators.get_all_arms()
        for arm in arms:
            arm_semantic_description = ManipulatorSemanticDescription(arm)

            self.xacro_writer.write_comment(
                '{0}'.format(arm_semantic_description.name)
            )

            self.xacro_writer.write_include(
                package=arm_semantic_description.package,
                file=arm_semantic_description.model,
                path=arm_semantic_description.path,
            )

            for pose in arm.poses:
                pose_macro = ManipulatorPoseMacro(arm, pose)
                self.xacro_writer.write_macro(
                    macro=pose_macro.macro(),
                    parameters=pose_macro.parameters(),
                    blocks=pose_macro.blocks(),
                )

            self.xacro_writer.write_macro(
                macro='{0}'.format(arm_semantic_description.model),
                parameters=arm_semantic_description.parameters,
            )

            self.xacro_writer.write_newline()

    def generate_grippers(self) -> None:
        self.xacro_writer.write_comment('Grippers')
        self.xacro_writer.write_newline()
        arms = self.clearpath_config.manipulators.get_all_arms()
        for arm in arms:
            if not arm.gripper:
                continue
            gripper_semantic_description = ManipulatorSemanticDescription(arm.gripper)

            self.xacro_writer.write_comment(
                '{0}'.format(gripper_semantic_description.name)
            )

            self.xacro_writer.write_include(
                package=gripper_semantic_description.package,
                file=gripper_semantic_description.model,
                path=gripper_semantic_description.path,
            )

            for pose in arm.gripper.poses:
                pose_macro = ManipulatorPoseMacro(arm.gripper, pose)
                self.xacro_writer.write_macro(
                    macro=pose_macro.macro(),
                    parameters=pose_macro.parameters(),
                    blocks=pose_macro.blocks(),
                )

            self.xacro_writer.write_macro(
                macro='{0}'.format(gripper_semantic_description.model),
                parameters=gripper_semantic_description.parameters,
            )

            self.xacro_writer.write_newline()

    def generate_lifts(self) -> None:
        self.xacro_writer.write_comment('Lifts')
        self.xacro_writer.write_newline()
        lifts = self.clearpath_config.manipulators.get_all_lifts()
        for lift in lifts:
            lift_semantic_description = ManipulatorSemanticDescription(lift)

            self.xacro_writer.write_comment(
                '{0}'.format(lift_semantic_description.name)
            )

            self.xacro_writer.write_include(
                package=lift_semantic_description.package,
                file=lift_semantic_description.model,
                path=lift_semantic_description.path,
            )

            self.xacro_writer.write_macro(
                macro='{0}'.format(lift_semantic_description.model),
                parameters=lift_semantic_description.parameters,
            )

            self.xacro_writer.write_newline()
