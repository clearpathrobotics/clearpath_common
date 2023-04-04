from clearpath_config.parser import ClearpathConfigParser
from clearpath_config.common import File
from clearpath_description_generator.xacro_writer import XacroWriter
from clearpath_description_generator.description.mounts import MountDescription
from clearpath_description_generator.description.platform import PlatformDescription
from clearpath_description_generator.description.decorations import DecorationsDescription

import sys


class DescriptionGenerator():
    def __init__(self, config: File = None) -> None:
        if config:
            self.config_path = config.get_path()
        else:
            self.config_path = '/etc/clearpath/generic-0001.yaml'
        self.description_path = '/home/rkreinin/clearpath_ws/'

        self.pkg_clearpath_platform_description = 'clearpath_platform_description'
        self.pkg_clearpath_mounts_description = 'clearpath_mounts_description'
        self.pkg_clearpath_sensors_description = 'clearpath_sensors_description'

        self.config = ClearpathConfigParser.read_yaml(self.config_path)
        self.clearpath_config = ClearpathConfigParser(self.config)

        self.serial = self.clearpath_config.platform.get_serial_number()

        self.urdf_file = File(self.description_path + self.serial + '.urdf.xacro', creatable=True)

        self.xacro_writer = XacroWriter(self.urdf_file, self.serial)

    def generate(self) -> None:
        # Common macros
        self.xacro_writer.write_include(
            package=self.pkg_clearpath_platform_description,
            file='common',
            path='urdf/'
        )

        # Platform
        self.generate_platform()
        self.xacro_writer.write_newline()

        # Decorations
        self.generate_decorations()
        self.xacro_writer.write_newline()

        # Mounts
        self.generate_mounts()
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
        decorations = self.clearpath_config.platform.decorations.get_all_decorations()
        for decoration in decorations:
            if decoration.get_enabled():
                print(decoration.get_enabled())
                bumper_description = DecorationsDescription(self.platform, decoration)

                self.xacro_writer.write_include(
                    package=bumper_description.get_package(),
                    file=bumper_description.get_file(),
                    path=bumper_description.get_path())

                self.xacro_writer.write_macro(
                    macro=bumper_description.get_file(),
                    parameters=bumper_description.get_parameters())

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


def main():
    config = None
    if len(sys.argv) > 1:
        config = File(sys.argv[1], exists=True)
    dg = DescriptionGenerator(config)
    dg.generate()
