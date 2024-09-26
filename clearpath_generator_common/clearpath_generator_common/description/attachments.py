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

from clearpath_config.platform.attachments.a200 import A200Attachment
from clearpath_config.platform.attachments.config import BaseAttachment
from clearpath_config.platform.attachments.j100 import J100Attachment
from clearpath_config.platform.attachments.w200 import W200Attachment
from clearpath_config.platform.types.bumper import Bumper


class AttachmentsDescription():
    class BaseDescription():
        NAME = 'name'
        MODEL = 'model'
        PARENT_LINK = 'parent_link'
        pkg_clearpath_platform_description = 'clearpath_platform_description'

        def __init__(self, attachment: BaseAttachment) -> None:
            self.attachment = attachment
            self.package = self.pkg_clearpath_platform_description
            self.path = f'urdf/{self.attachment.platform}/attachments/'
            self.file = self.attachment.file

            self.parameters = {
                self.NAME: attachment.name,
                self.MODEL: attachment.model,
                self.PARENT_LINK: attachment.parent,
            }

        @property
        def xyz(self) -> List[float]:
            return self.attachment.xyz

        @property
        def rpy(self) -> List[float]:
            return self.attachment.rpy

    class BumperDescription(BaseDescription):
        EXTENSION = 'extension'

        def __init__(self, attachment: Bumper) -> None:
            super().__init__(attachment)
            self.parameters.update({
                self.EXTENSION: attachment.extension
            })

    MODEL = {
        # A200
        A200Attachment.BUMPER: BumperDescription,
        A200Attachment.TOP_PLATE: BaseDescription,
        A200Attachment.SENSOR_ARCH: BaseDescription,
        # J100
        J100Attachment.FENDER: BaseDescription,
        J100Attachment.TOP_PLATE: BaseDescription,
        # W200
        W200Attachment.GENERATOR: BaseDescription,
        W200Attachment.BULKHEAD: BaseDescription,
        W200Attachment.ARM_PLATE: BaseDescription,
    }

    def __new__(cls, attachment: BaseAttachment) -> BaseDescription:
        return AttachmentsDescription.MODEL.setdefault(
            attachment.ATTACHMENT_MODEL,
            AttachmentsDescription.BaseDescription)(attachment)
