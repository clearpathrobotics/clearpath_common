from clearpath_config.parser import ClearpathConfigParser
from clearpath_config.common import Platform, File
from clearpath_config.platform.a200 import A200DecorationsConfig
from clearpath_config.platform.decorations import Decorations
from clearpath_config.mounts.mounts import Mount
from clearpath_description_generator.xacro_writer import XacroWriter
from clearpath_description_generator.description.mounts import MountDescription


import sys

class DescriptionGenerator():
    def __init__(self, config: File = None) -> None:
        if config:
            self.config_path = config.get_path()
        else:
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

        # self.xacro_writer.write_include(
        #     self.pkg_clearpath_mounts_description,
        #     file='row_riser',
        #     path='urdf/pacs/'
        # )
        # self.xacro_writer.write_macro(
        #     macro='row_riser',
        #     parameters={
        #         'name': 'row_riser_0',
        #         'parent_link': 'a1'
        #     }
        # )

        self.xacro_writer.write_include(
            self.pkg_clearpath_mounts_description,
            file='riser',
            path='urdf/pacs/'
        )
        self.xacro_writer.write_macro(
            macro='riser',
            parameters={
                'name': 'riser_0',
                'parent_link': 'top_plate_mount_a1',
                'columns': '7',
                'rows': '8',
            },
            blocks=XacroWriter.add_origin(
                xyz=[0, 0, 0.1],
                rpy=[0, 0, 0]
            )
        )

        self.xacro_writer.write_include(
            self.pkg_clearpath_mounts_description,
            file='riser_leg',
            path='urdf/pacs/'
        )
        self.xacro_writer.write_macro(
            macro='riser_leg',
            parameters={
                'name': 'riser_leg_0',
                'parent_link': 'top_plate_mount_a4',
                'height': '0.10'
            },
            blocks=XacroWriter.add_origin(
                xyz=[0, 0, 0],
                rpy=[0, 0, 0]
            )
        )

        self.xacro_writer.write_include(
            self.pkg_clearpath_mounts_description,
            file='quad_post',
            path='urdf/pacs/'
        )
        self.xacro_writer.write_macro(
            macro='quad_post',
            parameters={
                'name': 'quad_post_0',
                'parent_link': 'riser_0_mount_a3',
                'height': '0.10'
            },
            blocks=XacroWriter.add_origin(
                xyz=[0, 0, 0],
                rpy=[0, 0, 0]
            )
        )

        self.xacro_writer.write_include(
            self.pkg_clearpath_mounts_description,
            file='novatel_smart7_stand',
            path='urdf/'
        )
        self.xacro_writer.write_macro(
            macro='novatel_smart7_stand',
            parameters={
                'name': 'novatel_smart7_stand',
                'parent_link': 'riser_0_mount_g8',
            },
            blocks=XacroWriter.add_origin(
                xyz=[0, 0, 0],
                rpy=[0, 0, 0]
            )
        )

        self.xacro_writer.write_include(
            self.pkg_clearpath_sensors_description,
            file='novatel_smart7',
            path='urdf/'
        )
        self.xacro_writer.write_macro(
            macro='novatel_smart7',
            parameters={
                'name': 'gps_0',
                'parent_link': 'novatel_smart7_stand_mount'
            },
            blocks=XacroWriter.add_origin(
                xyz=[0, 0, 0],
                rpy=[0, 0, 0]
            )
        )

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
                self.xacro_writer.write_macro(
                    'top_plate',
                    parameters={'model': decorations.top_plate.model})

            # brackets = decorations.pacs.get_brackets()
            # for bracket in brackets:
            #     print(bracket.name)
            #     self.xacro_writer.write_include(
            #         self.pkg_clearpath_platform_description,
            #         'bracket',
            #         path='urdf/' + self.model + '/decorations/pacs/')
            #     self.xacro_writer.write_macro(
            #         macro='bracket',
            #         parameters={
            #             'name': bracket.get_name(),
            #             'model': bracket.get_model(),
            #             'parent_link': bracket.get_parent(),
            #         },
            #         blocks=XacroWriter.add_origin(
            #             bracket.get_xyz(),
            #             bracket.get_rpy()
            #         ))

            # full_risers = decorations.pacs.get_full_risers()
            # for full_riser in full_risers:
            #     print(full_riser.get_level())
            #     self.xacro_writer.write_include(
            #         self.pkg_clearpath_platform_description,
            #         'full_riser',
            #         path='urdf/' + self.model + '/decorations/pacs/')
            #     self.xacro_writer.write_macro(
            #         macro='full_riser',
            #         parameters={
            #             'level': full_riser.get_level(),
            #         })

            # row_risers = decorations.pacs.get_row_risers()
            # for row_riser in row_risers:
            #     print(row_riser.get_level())
            #     self.xacro_writer.write_include(
            #         self.pkg_clearpath_platform_description,
            #         'row_riser',
            #         path='urdf/' + self.model + '/decorations/pacs/')
            #     self.xacro_writer.write_macro(
            #         macro='row_riser',
            #         parameters={
            #             'level': row_riser.get_level(),
            #             'row': row_riser.get_row()
            #         })

    def generate_mounts(self) -> None:
        self.xacro_writer.write_comment('Mounts')
        self.xacro_writer.write_newline()
        mounts = self.clearpath_config.mounts.get_mounts()
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
                macro='{0}'.format(mount.get_model()),
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
