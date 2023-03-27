from clearpath_config.mounts.mounts import Mount


class MountDescription():
    class Base():
        NAME = 'name'
        PARENT = 'parent_link'
        MOUNTING_LINK = 'mounting_link'
        XYZ = 'xyz'
        RPY = 'rpy'

        def __init__(self, mount: Mount) -> None:
            self.parameters = {
                self.NAME: mount.get_name(),
                self.PARENT: mount.get_parent(),
                self.MOUNTING_LINK: mount.get_mounting_link()
            }

            self.xyz = mount.get_xyz()
            self.rpy = mount.get_rpy()

        def get_parameters(self) -> list():
            return self.parameters

        def get_parameter(self, parameter: str) -> str:
            return self.parameters[parameter]

    class FathPivot(Base):
        ANGLE = 'angle'

        def __init__(self, mount: Mount) -> None:
            super().__init__(mount)
            self.parameters[self.ANGLE] = mount.get_angle()

    MODEL = {
        Mount.FATH_PIVOT: FathPivot,
        Mount.FLIR_PTU: Base,
    }

    def __new__(cls, mount: Mount) -> Base:
        return MountDescription.MODEL[mount.get_model()](mount)
