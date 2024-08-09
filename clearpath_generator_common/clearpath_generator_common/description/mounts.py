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
from typing import List

from clearpath_config.mounts.types.fath_pivot import FathPivot
from clearpath_config.mounts.types.mount import BaseMount
from clearpath_config.mounts.types.pacs import PACS
from clearpath_config.mounts.types.post import Post
from clearpath_config.mounts.types.sick import SICKStand


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
                self.NAME: mount.name,
                self.PARENT: mount.parent
            }

        @property
        def name(self) -> str:
            return self.parameters[self.NAME]

        @property
        def model(self) -> str:
            return self.mount.MOUNT_MODEL

        @property
        def xyz(self) -> List[float]:
            return self.mount.xyz

        @property
        def rpy(self) -> List[float]:
            return self.mount.rpy

    class FathPivotDescription(BaseDescription):
        ANGLE = 'angle'

        def __init__(self, mount: FathPivot) -> None:
            super().__init__(mount)
            self.parameters[self.ANGLE] = mount.angle

    class PACSRiserDescription(BaseDescription):
        ROWS = 'rows'
        COLUMNS = 'columns'
        THICKNESS = 'thickness'

        def __init__(self, mount: PACS.Riser) -> None:
            super().__init__(mount)
            self.path = 'urdf/pacs/'
            self.parameters.update({
                self.ROWS: mount.rows,
                self.COLUMNS: mount.columns,
                self.THICKNESS: mount.thickness
            })

    class PACSBracketDescription(BaseDescription):
        MODEL = 'model'

        def __init__(self, mount: PACS.Bracket) -> None:
            super().__init__(mount)
            self.path = 'urdf/pacs/'
            self.parameters.update({
                self.MODEL: mount.model,
            })

    class SICKStandDescription(BaseDescription):
        MODEL = 'model'

        def __init__(self, mount: SICKStand) -> None:
            super().__init__(mount)
            self.parameters.update({
                self.MODEL: mount.model,
            })

    class PostDescription(BaseDescription):
        MODEL = 'model'
        SPACING = 'spacing'
        HEIGHT = 'height'

        def __init__(self, mount: Post) -> None:
            super().__init__(mount)
            self.parameters.update({
                self.MODEL: mount.model,
                self.SPACING: mount.spacing,
                self.HEIGHT: mount.height
            })

    MODEL = {
        FathPivot.MOUNT_MODEL: FathPivotDescription,
        PACS.Bracket.MOUNT_MODEL: PACSBracketDescription,
        PACS.Riser.MOUNT_MODEL: PACSRiserDescription,
        SICKStand.MOUNT_MODEL: SICKStandDescription,
        Post.MOUNT_MODEL: PostDescription
    }

    def __new__(cls, mount: BaseMount) -> BaseDescription:
        return MountDescription.MODEL.setdefault(
            mount.MOUNT_MODEL,
            MountDescription.BaseDescription)(mount)
