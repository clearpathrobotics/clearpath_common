from clearpath_config.parser import ClearpathConfigParser
from clearpath_config.common import Platform
from clearpath_config.platform.a200 import A200DecorationsConfig
from clearpath_config.platform.decorations import Decorations
from clearpath_config.mounts.mounts import Mount
from clearpath_description_generator.xacro_writer import XacroWriter
from clearpath_description_generator.mount_description import MountDescription


class DescriptionGenerator():
    def __init__(self) -> None:
        self.config_path = '/home/rkreinin/clearpath_ws/src/clearpath_config/clearpath_config/sample/a200_config.yaml'
        self.description_path = '/home/rkreinin/clearpath_ws/'

        self.pkg_clearpath_platform_description = 'clearpath_platform_description'
        self.pkg_clearpath_mounts_description = 'clearpath_mounts_description'
        self.pkg_clearpath_sensors_description = 'clearpath_sensors_description'

        self.config = ClearpathConfigParser.read_yaml(self.config_path)
        self.clearpath_config = ClearpathConfigParser(self.config)

        self.serial = self.clearpath_config.platform.get_serial_number()

        self.xacro_writer = XacroWriter(self.description_path + self.serial + '.urdf.xacro', self.serial)

    def generate(self) -> None:
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
        self.model = self.clearpath_config.platform.get_model()

        # Platform macro
        self.xacro_writer.write_comment('Platform')
        self.xacro_writer.write_include(
            self.pkg_clearpath_platform_description,
            self.model,
            path='urdf/' + self.model + '/')
        self.xacro_writer.write_macro(self.model)

    def generate_decorations(self) -> None:
        self.xacro_writer.write_comment('Decorations')
        if self.model == Platform.A200:
            decorations: A200DecorationsConfig = self.clearpath_config.platform.decorations
            if decorations.front_bumper.enabled:
                self.xacro_writer.write_include(
                    self.pkg_clearpath_platform_description,
                    'front_bumper',
                    path='urdf/' + self.model + '/decorations/')
                self.xacro_writer.write_macro('front_bumper')
            if decorations.rear_bumper.enabled:
                self.xacro_writer.write_include(
                    self.pkg_clearpath_platform_description,
                    'rear_bumper',
                    path='urdf/' + self.model + '/decorations/')
                self.xacro_writer.write_macro('rear_bumper')
            if decorations.top_plate.enabled:
                self.xacro_writer.write_include(
                    self.pkg_clearpath_platform_description,
                    'top_plate',
                    path='urdf/' + self.model + '/decorations/')
                self.xacro_writer.write_macro('top_plate',
                    parameters={'large': decorations.top_plate.model==Decorations.A200.TopPlate.LARGE})

    def generate_mounts(self) -> None:
        self.xacro_writer.write_comment('Mounts')
        self.xacro_writer.write_newline()
        mounts = self.clearpath_config.mounts.get_mounts()
        for mount in mounts:
            mount_description = MountDescription(mount)

            self.xacro_writer.write_comment(
                '{0}'.format(mount_description.get_parameter(MountDescription.Base.NAME))
            )

            self.xacro_writer.write_include(
                self.pkg_clearpath_mounts_description,
                mount.get_model(),
                path='urdf/'
            )

            self.xacro_writer.write_macro(
                macro='{0}'.format(mount.get_model()),
                parameters=mount_description.get_parameters(),
                blocks=XacroWriter.add_origin(
                    mount_description.xyz, mount_description.rpy)
            )

            self.xacro_writer.write_newline()


def main():
    dg = DescriptionGenerator()
    dg.generate()
