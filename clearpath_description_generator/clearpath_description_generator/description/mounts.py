from clearpath_config.mounts.mounts import BaseMount
from clearpath_config.mounts.fath_pivot import FathPivot
from clearpath_config.mounts.flir_ptu import FlirPTU
from clearpath_config.mounts.pacs import PACS

from typing import List


class MountDescription():
    class BaseDescription():
        pkg_clearpath_mounts_description = 'clearpath_mounts_description'

        NAME = 'name'
        PARENT = 'parent_link'
        XYZ = 'xyz'
        RPY = 'rpy'

        def __init__(self, mount: BaseMount) -> None:
            self.mount = mount
            self.package = self.pkg_clearpath_mounts_description
            self.path = 'urdf/'

            self.parameters = {
                self.NAME: mount.get_name(),
                self.PARENT: mount.get_parent()
            }

        def get_parameters(self) -> dict:
            return self.parameters

        def get_parameter(self, parameter: str) -> str:
            return self.parameters[parameter]

        def get_name(self) -> str:
            return self.parameters[self.NAME]

        def get_model(self) -> str:
            return self.mount.MOUNT_MODEL

        def get_package(self) -> str:
            return self.package

        def get_path(self) -> str:
            return self.path

        def get_xyz(self) -> List[float]:
            return self.mount.get_xyz()

        def get_rpy(self) -> List[float]:
            return self.mount.get_rpy()

    class FathPivotDescription(BaseDescription):
        ANGLE = 'angle'

        def __init__(self, mount: FathPivot) -> None:
            super().__init__(mount)
            self.parameters[self.ANGLE] = mount.get_angle()

    class PACSRiserDescription(BaseDescription):
        ROWS = 'rows'
        COLUMNS = 'columns'
        THICKNESS = 'thickness'

        def __init__(self, mount: PACS.Riser) -> None:
            super().__init__(mount)
            self.path = 'urdf/pacs/'
            self.parameters.update({
                self.ROWS: mount.get_rows(),
                self.COLUMNS: mount.get_columns(),
                self.THICKNESS: mount.get_thickness()
            })

    class PACSBracketDescription(BaseDescription):
        MODEL = 'model'

        def __init__(self, mount: PACS.Bracket) -> None:
            super().__init__(mount)
            self.path = 'urdf/pacs/'
            self.parameters.update({
                self.MODEL: mount.get_model(),
            })

    MODEL = {
        FathPivot.MOUNT_MODEL: FathPivotDescription,
        FlirPTU.MOUNT_MODEL: BaseDescription,
        PACS.Bracket.MOUNT_MODEL: PACSBracketDescription,
        PACS.Riser.MOUNT_MODEL: PACSRiserDescription,
    }

    def __new__(cls, mount: BaseMount) -> BaseDescription:
        return MountDescription.MODEL[mount.MOUNT_MODEL](mount)
