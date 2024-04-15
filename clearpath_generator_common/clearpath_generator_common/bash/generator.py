#!/usr/bin/env python3

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

from clearpath_config.common.types.discovery import Discovery
from clearpath_generator_common.bash.writer import BashWriter
from clearpath_generator_common.common import BaseGenerator, BashFile


class BashGenerator(BaseGenerator):

    ROS_DISTRO_PATH = '/opt/ros/humble/'

    def generate(self) -> None:
        # Generate setup.bash
        self.generate_setup_bash()

    def generate_setup_bash(self) -> None:
        clearpath_setup_bash = BashFile(filename='setup.bash', path=self.setup_path)
        bash_writer = BashWriter(clearpath_setup_bash)

        workspaces = self.clearpath_config.system.workspaces

        # Source Humble
        humble_setup_bash = BashFile(filename='setup.bash', path=self.ROS_DISTRO_PATH)
        bash_writer.add_source(humble_setup_bash)

        # Additional workspaces
        for workspace in workspaces:
            bash_writer.add_source(
                BashFile(filename='setup.bash', path=workspace.strip('setup.bash')))

        # ROS_DOMAIN_ID
        domain_id = self.clearpath_config.system.domain_id
        bash_writer.add_export('ROS_DOMAIN_ID', domain_id)

        # RMW_IMPLEMENTATION
        rmw = self.clearpath_config.system.middleware.rmw_implementation
        bash_writer.add_export('RMW_IMPLEMENTATION', rmw)

        # Custom DDS Profile
        if self.clearpath_config.system.middleware.profile:
            bash_writer.add_export(
                'FASTRTPS_DEFAULT_PROFILES_FILE', self.clearpath_config.system.middleware.profile)

        # Fast DDS Discovery Server
        if self.clearpath_config.system.middleware.discovery == Discovery.SERVER:
            server_str = self.clearpath_config.system.middleware.get_servers_string()
            bash_writer.add_export('ROS_DISCOVERY_SERVER', f'"{server_str}"')

            # If this is not being called by systemd then set as super user (for ROS 2 cli tools)
            bash_writer.write('if [ -t 0 ]; then')
            bash_writer.add_export('ROS_SUPER_CLIENT', True, indent_level=1)
            bash_writer.write('else')
            bash_writer.add_unset('ROS_SUPER_CLIENT', indent_level=1)
            bash_writer.write('fi')

        else:
            bash_writer.add_unset('ROS_DISCOVERY_SERVER')

        bash_writer.close()
