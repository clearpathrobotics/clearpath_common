from clearpath_config.mounts.mounts import Mount

from typing import List


class MountDescription():
    class Base():
        pkg_clearpath_mounts_description = 'clearpath_mounts_description'

        NAME = 'name'
        PARENT = 'parent_link'
        XYZ = 'xyz'
        RPY = 'rpy'

        def __init__(self, mount: Mount) -> None:
            self.mount = mount
            self.package = self.pkg_clearpath_mounts_description
            self.path = 'urdf/'

            self.parameters = {
                self.NAME: mount.get_name(),
                self.PARENT: mount.get_parent(),
            }

        def get_parameters(self) -> dict:
            return self.parameters

        def get_parameter(self, parameter: str) -> str:
            return self.parameters[parameter]

        def get_name(self) -> str:
            return self.parameters[self.NAME]

        def get_model(self) -> str:
            return self.mount.get_model()

        def get_package(self) -> str:
            return self.package

        def get_path(self) -> str:
            return self.path

        def get_xyz(self) -> List[float]:
            return self.mount.get_xyz()

        def get_rpy(self) -> List[float]:
            return self.mount.get_rpy()

    class FathPivot(Base):
        ANGLE = 'angle'

        def __init__(self, mount: Mount) -> None:
            super().__init__(mount)
            self.parameters[self.ANGLE] = mount.get_angle()

    class PACS(Base):

        def __init__(self, mount: Mount) -> None:
            super().__init__(mount)
            self.path = 'urdf/pacs/'

    MODEL = {
        Mount.FATH_PIVOT: FathPivot,
        Mount.FLIR_PTU: Base,
    }

    def __new__(cls, mount: Mount) -> Base:
        return MountDescription.MODEL[mount.get_model()](mount)
