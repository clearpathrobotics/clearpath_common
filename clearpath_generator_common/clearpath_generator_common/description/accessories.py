# Software License Agreement (BSD)
#
# @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
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

from clearpath_config.accessories.accessories import URDFAccessory, BaseAccessory
from clearpath_config.accessories.types.box import Box
from clearpath_config.accessories.types.cylinder import Cylinder
from clearpath_config.accessories.types.mesh import Mesh
from clearpath_config.accessories.types.sphere import Sphere

from typing import List


class AccessoryDescription():
    class BaseDescription():
        pkg_clearpath_platform_description = 'clearpath_platform_description'

        NAME = 'name'
        PARENT_LINK = 'parent_link'

        def __init__(self, accessory: BaseAccessory) -> None:
            self.accessory = accessory
            self.package = self.pkg_clearpath_platform_description
            self.path = 'urdf/accessories/'
            self.file = self.accessory.get_accessory_type()

            self.parameters = {
                self.NAME: self.accessory.get_name(),
                self.PARENT_LINK: self.accessory.get_parent()
            }

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

        def get_xyz(self) -> List[float]:
            return self.accessory.get_xyz()

        def get_rpy(self) -> List[float]:
            return self.accessory.get_rpy()

    class BoxDescription(BaseDescription):
        SIZE = 'size'

        def __init__(self, accessory: Box) -> None:
            super().__init__(accessory)
            self.parameters.update({
                self.SIZE: str(accessory.get_size()).strip('[]').replace(',', '')
            })

    class CylinderDescription(BaseDescription):
        RADIUS = 'radius'
        LENGTH = 'length'

        def __init__(self, accessory: Cylinder) -> None:
            super().__init__(accessory)
            self.parameters.update({
                self.RADIUS: accessory.get_radius(),
                self.LENGTH: accessory.get_length()
            })

    class SphereDescription(BaseDescription):
        RADIUS = 'radius'

        def __init__(self, accessory: Sphere) -> None:
            super().__init__(accessory)
            self.parameters.update({
                self.RADIUS: accessory.get_radius()
            })

    class MeshDescription(BaseDescription):
        VISUAL = 'visual'

        def __init__(self, accessory: Mesh) -> None:
            super().__init__(accessory)
            self.parameters.update({
                self.VISUAL: accessory.get_visual()
            })

    MODEL = {
        URDFAccessory.BOX: BoxDescription,
        URDFAccessory.CYLINDER: CylinderDescription,
        URDFAccessory.LINK: BaseDescription,
        URDFAccessory.MESH: MeshDescription,
        URDFAccessory.SPHERE: SphereDescription,
    }

    def __new__(cls, accessory: BaseAccessory) -> BaseDescription:
        return AccessoryDescription.MODEL[accessory.ACCESSORY_TYPE](accessory)
