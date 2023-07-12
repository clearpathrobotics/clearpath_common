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

from clearpath_generator_common.bash.writer import BashWriter
from clearpath_generator_common.common import BashFile, BaseGenerator


class BashGenerator(BaseGenerator):

    def generate(self) -> None:
        self.generate_setup_bash()

    def generate_setup_bash(self) -> None:
        clearpath_setup_bash = BashFile(name='setup', path=self.setup_path)
        bash_writer = BashWriter(clearpath_setup_bash)

        workspaces = self.clearpath_config.system.workspaces

        # Source Humble
        humble_setup_bash = BashFile(name='setup', path='/opt/ros/humble/')
        bash_writer.add_source(humble_setup_bash)

        # Additional workspaces
        for workspace in workspaces:
            bash_writer.add_source(
                BashFile(name='setup', path=workspace.strip('setup.bash')))

        # ROS_DOMAIN_ID
        domain_id = self.clearpath_config.system.domain_id
        bash_writer.add_export('ROS_DOMAIN_ID', domain_id)

        # RMW_IMPLEMENTATION
        rmw = self.clearpath_config.system.rmw_implementation
        bash_writer.add_export('RMW_IMPLEMENTATION', rmw)

        bash_writer.close()
