from clearpath_config.platform.platform import Platform


class PlatformDescription():
    class Base():
        pkg_clearpath_platform_description = 'clearpath_platform_description'

        def __init__(self, model: Platform) -> None:
            self.package = self.pkg_clearpath_platform_description
            self.file = model
            self.macro = self.file
            self.path = 'urdf/' + model + '/'

        def get_file(self) -> str:
            return self.file

        def get_macro(self) -> str:
            return self.macro

        def get_package(self) -> str:
            return self.package

        def get_path(self) -> str:
            return self.path

    MODEL = {
        Platform.A200: Base,
    }

    def __new__(cls, model: Platform) -> Base:
        return PlatformDescription.MODEL[model](model)
