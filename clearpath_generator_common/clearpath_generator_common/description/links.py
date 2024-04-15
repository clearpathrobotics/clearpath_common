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
import os

from typing import List

from clearpath_config.common.types.file import File
from clearpath_config.links.links import Link
from clearpath_config.links.types.box import Box
from clearpath_config.links.types.cylinder import Cylinder
from clearpath_config.links.types.link import BaseLink
from clearpath_config.links.types.mesh import Mesh
from clearpath_config.links.types.sphere import Sphere


class LinkDescription():
    class BaseDescription():
        pkg_clearpath_platform_description = 'clearpath_platform_description'

        NAME = 'name'
        PARENT_LINK = 'parent_link'

        def __init__(self, link: BaseLink) -> None:
            self.link = link
            self.package = self.pkg_clearpath_platform_description
            self.path = 'urdf/links/'
            self.file = self.link.get_link_type()

            self.parameters = {
                self.NAME: self.link.name,
                self.PARENT_LINK: self.link.parent
            }

        @property
        def xyz(self) -> List[float]:
            return self.link.xyz

        @property
        def rpy(self) -> List[float]:
            return self.link.rpy

    class BoxDescription(BaseDescription):
        SIZE = 'size'

        def __init__(self, link: Box) -> None:
            super().__init__(link)
            self.parameters.update({
                self.SIZE: str(link.size).strip('[]').replace(',', '')
            })

    class CylinderDescription(BaseDescription):
        RADIUS = 'radius'
        LENGTH = 'length'

        def __init__(self, link: Cylinder) -> None:
            super().__init__(link)
            self.parameters.update({
                self.RADIUS: link.radius,
                self.LENGTH: link.length
            })

    class SphereDescription(BaseDescription):
        RADIUS = 'radius'

        def __init__(self, link: Sphere) -> None:
            super().__init__(link)
            self.parameters.update({
                self.RADIUS: link.radius
            })

    class MeshDescription(BaseDescription):
        VISUAL = 'visual'

        def __init__(self, link: Mesh) -> None:
            super().__init__(link)
            if (link.visual.package):
                self.parameters.update({
                    self.VISUAL: os.path.join('package://' + link.visual.package,
                                              File.clean(link.visual.path, make_abs=False))
                })
            else:
                self.parameters.update({
                    self.VISUAL: 'file://' + File.clean(link.visual.path, make_abs=False)
                })

    MODEL = {
        Link.BOX: BoxDescription,
        Link.CYLINDER: CylinderDescription,
        Link.MESH: MeshDescription,
        Link.SPHERE: SphereDescription,
    }

    def __new__(cls, link: BaseLink) -> BaseDescription:
        return LinkDescription.MODEL.setdefault(
            link.LINK_TYPE,
            LinkDescription.BaseDescription)(link)
