from clearpath_generator_common.common import BaseGenerator
from clearpath_generator_common.description.writer import XacroWriter
from clearpath_generator_common.semantic_description.manipulators import (
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

            self.xacro_writer.write_macro(
                macro='{0}'.format(gripper_semantic_description.model),
                parameters=gripper_semantic_description.parameters,
            )

            self.xacro_writer.write_newline()

    def generate_lifts(self) -> None:
        pass
