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

from clearpath_generator_common.common import ParamFile


class ParamWriter():
    tab = '  '

    def __init__(self, param_file: ParamFile):
        self.param_file = param_file
        self.file = open(self.param_file.full_path, 'w+')

    def write(self, string, indent_level=1):
        self.file.write('{0}{1}\n'.format(self.tab * indent_level, string))

    def write_key_value_pair(self, key: str, value, indent_level=1):
        self.write(f'{key}: {value}', indent_level=indent_level)

    def write_string(self, key: str, value: str, indent_level=1):
        self.write(f"{key}: '{value}'", indent_level=indent_level)

    def write_dictionary(self, key: str, dictionary: dict, indent_level=1):
        self.write(f'{key}:', indent_level=indent_level)
        for k in dictionary:
            self.write_obj(k, dictionary[k], indent_level + 1)

    def write_obj(self, key: str, obj: object, indent_level=1):
        if isinstance(obj, dict):
            self.write_dictionary(key, obj, indent_level)
        elif isinstance(obj, str):
            self.write_string(key, obj, indent_level)
        else:
            self.write_key_value_pair(key, obj, indent_level)

    def write_file(self):
        ros_parameters = self.param_file.to_ros_parameters()
        for k in ros_parameters:
            self.write_obj(k, ros_parameters[k], indent_level=0)
        self.file.close()
