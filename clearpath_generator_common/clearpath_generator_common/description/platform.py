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

from clearpath_config.common.types.platform import Platform


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
        Platform.J100: Base,
    }

    def __new__(cls, model: Platform) -> Base:
        return PlatformDescription.MODEL[model](model)
