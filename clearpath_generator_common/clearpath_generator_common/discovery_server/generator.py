#!/usr/bin/env python3

# Software License Agreement (BSD)
#
# @author    Hilary Luo <hluo@clearpathrobotics.com>
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

from clearpath_config.common.types.discovery import Discovery
from clearpath_config.common.types.rmw_implementation import RMWImplementation
from clearpath_generator_common.bash.writer import BashWriter
from clearpath_generator_common.common import BaseGenerator, BashFile


class DiscoveryServerGenerator(BaseGenerator):

    ROS_DISTRO_PATH = '/opt/ros/humble/'

    def generate(self) -> None:
        # Generate the file that launches the FastDDS discovery server
        self.generate_server_start()

    def generate_server_start(self) -> None:
        # Generate script that launches the discovery server
        discovery_server_start = BashFile(filename='discovery-server-start', path=self.setup_path)
        bash_writer = BashWriter(discovery_server_start)

        # Source Humble
        humble_setup_bash = BashFile(filename='setup.bash', path=self.ROS_DISTRO_PATH)
        bash_writer.add_source(humble_setup_bash)

        # If Fast DDS Discovery Server is selected then check if a local server should be run
        middleware_config = self.clearpath_config.system.middleware
        if ((middleware_config.rmw_implementation == RMWImplementation.FAST_RTPS) and
                (middleware_config.discovery == Discovery.SERVER)):

            # For Debug:
            # for i, s in enumerate(self.clearpath_config.system.middleware.servers.get_all()):
            #     print(f"Server {i} is {str(s)}")
            # print(f"Localhost is {self.clearpath_config.system.localhost}")

            ls = middleware_config.get_local_server()
            if ls and ls.enabled:
                bash_writer.write(
                    f'fastdds discovery -i {ls.server_id} -p {ls.port}'
                )
            else:
                bash_writer.add_echo(
                    'No local discovery server enabled. ' +
                    'If this was launched as a service then the service will now end.'
                )
        else:
            bash_writer.add_echo(
                'Discovery server not enabled. ' +
                'If this was launched as a service then the service will now end.'
            )

        bash_writer.close()
