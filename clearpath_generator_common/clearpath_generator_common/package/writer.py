#!/usr/bin/env python3

# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.
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
import shutil

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


class PackageWriter():

    @staticmethod
    def find_package_template(package, template):
        # Package directory
        try:
            pkg = get_package_share_directory(package)
        except PackageNotFoundError:
            print("Could not find package '%s' installed." % package)
            exit
        # Find directory
        return os.path.join(pkg, template)

    @staticmethod
    def generate_from_template(template, destination):
        template_name = os.path.basename(template)
        destination_name = os.path.basename(destination)
        # Copy template to destination
        print("Copying template, '%s', to destination, '%s'." % (
            template,
            destination
        ))
        shutil.copytree(src=template, dst=destination)
        # Rename all files
        print("Renaming all template, '%s', files to '%s'." % (
            template_name,
            destination_name
        ))
        for root, _, files in os.walk(destination):
            for file in files:
                if template_name in file:
                    src = os.path.join(root, file)
                    dst = src.replace(template_name, destination_name)
                    shutil.move(src, dst)
        # Replace all instances
        print("Replacing all instances of '%s' with '%s'." % (
            template_name,
            destination_name
        ))
        for root, _, files in os.walk(destination):
            for file in files:
                filepath = os.path.join(root, file)
                try:
                    file = open(filepath, 'r')
                    data = file.read()
                    file.close()
                    if template_name in data:
                        new = data.replace(template_name, destination_name)
                        file = open(filepath, 'w')
                        file.write(new)
                        file.close()
                except Exception as e:
                    print(e)
