from clearpath_config.platform.decorations import BaseDecoration, Bumper, TopPlate


class DecorationsDescription():
    class BaseDescription():
        pkg_clearpath_platform_description = 'clearpath_platform_description'

        def __init__(self, platform: str, decoration: BaseDecoration) -> None:
            self.decoration = decoration
            self.package = self.pkg_clearpath_platform_description
            self.path = 'urdf/' + platform + '/decorations/'
            self.file = self.decoration.get_name()

            self.parameters = {}

        def get_parameters(self) -> dict:
            return self.parameters

        def get_parameter(self, parameter: str) -> str:
            return self.parameters[parameter]

        def get_package(self) -> str:
            return self.package

        def get_path(self) -> str:
            return self.path

        def get_file(self) -> str:
            return self.file

    class BumperDescription(BaseDescription):
        NAME = 'name'
        EXTENSION = 'extension'

        def __init__(self, platform: str, decoration: Bumper) -> None:
            super().__init__(platform, decoration)
            self.file = 'bumper'
            self.parameters.update({
                self.NAME: decoration.get_name(),
                self.EXTENSION: decoration.get_extension()
            })

    class TopPlateDescription(BaseDescription):
        MODEL = 'model'

        def __init__(self, platform: str, decoration: TopPlate) -> None:
            super().__init__(platform, decoration)
            self.parameters.update({
                self.MODEL: decoration.get_model(),
            })

    MODEL = {
        Bumper.DECORATION_MODEL: BumperDescription,
        TopPlate.DECORATION_MODEL: TopPlateDescription,
    }

    def __new__(cls, platform, decoration: BaseDecoration) -> BaseDescription:
        return DecorationsDescription.MODEL[decoration.DECORATION_MODEL](platform, decoration)
