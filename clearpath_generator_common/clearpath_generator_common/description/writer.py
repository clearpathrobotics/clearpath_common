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


class XacroWriter():
    tab = '  '

    def __init__(self, path: str, serial_number: str):
        self.file_path = path
        self.file_name = os.path.join(self.file_path, 'robot.urdf.xacro')
        self.file = open(self.file_name, 'w')
        self.initialize_file(serial_number)

    def write(self, string, indent_level=1):
        self.file.write('{0}{1}\n'.format(self.tab * indent_level, string))

    def write_include(self, file, package=None, path=None):
        # Append .urdf.xacro if missing
        if '.urdf.xacro' not in file:
            file = file + '.urdf.xacro'
        if path is None:
            path = ''
        file_path = os.path.join(path, file)
        if package:
            self.write('<xacro:include filename="$(find {0})/{1}"/>'.format(package, file_path))
        else:
            self.write('<xacro:include filename="{0}"/>'.format(file_path))

    def write_extras(self, path):
        self.write_comment('Extras')
        self.write('<xacro:include filename="{0}" />'.format(path))

    def write_macro(self, macro, parameters=None, blocks=None):
        params = ''
        if macro is None:
            return
        if parameters is not None:
            for p in parameters:
                params += ' {0}="{1}"'.format(p, parameters[p])

        if blocks is None:
            self.write('<xacro:{0}{1}/>\n'.format(macro, params))
        else:
            self.write('<xacro:{0}{1}>'.format(macro, params))
            self.write('{0}'.format(blocks), 2)
            self.write('</xacro:{0}>'.format(macro))

    def write_fixed_joint(self, name, parent, child, origin=None, indent_level=1):
        self.write('<joint name="{0}" type="fixed">'.format(name), indent_level)
        self.write('<parent link="{0}"/>'.format(parent), indent_level + 1)
        self.write('<child link="{0}"/>'.format(child), indent_level + 1)
        if origin is not None:
            self.write('{0}'.format(origin), indent_level + 1)
        self.write('</joint>', indent_level)

    def write_comment(self, comment, indent_level=1):
        self.write('<!-- {0} -->'.format(comment), indent_level)

    def write_newline(self):
        self.write('', 0)

    def add_origin(xyz, rpy):
        return '<origin xyz="{0}" rpy="{1}"/>'.format(
            str(xyz)[1:-1].replace(',', ''), str(rpy)[1:-1].replace(',', ''))

    def initialize_file(self, robot_name):
        self.write('<?xml version="1.0"?>', 0)
        self.write(
            '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="{0}">\n'.format(
                robot_name), 0)

    def close_file(self):
        self.write('</robot>', 0)
        self.file.close()
