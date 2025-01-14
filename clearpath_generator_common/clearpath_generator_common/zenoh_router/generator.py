#!/usr/bin/env python3

# Software License Agreement (BSD)
#
# @author    Chris Iverach-Brereton <civerachb@clearpathrobotics.com>
# @copyright (c) 2025, Clearpath Robotics, Inc., All rights reserved.
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
from clearpath_generator_common.ros import ROS_DISTRO_PATH


class ZenohRouterGenerator(BaseGenerator):

    def generate(self) -> None:
        # Generate the file that launches the Zenoh router
        self.generate_server_start()

    def generate_server_start(self) -> None:
        # Generate script that launches Zenoh router
        zenoh_router_start = BashFile(filename='zenoh-router-start', path=self.setup_path)
        bash_writer = BashWriter(zenoh_router_start)

        # Source ROS
        ros_setup_bash = BashFile(filename='setup.bash', path=ROS_DISTRO_PATH)
        bash_writer.add_source(ros_setup_bash)

        # If Fast DDS Discovery Server is selected then check if a local server should be run
        middleware_config = self.clearpath_config.system.middleware
        if middleware_config.rmw_implementation == RMWImplementation.ZENOH_DDS:

            if middleware_config.zenoh_router_config_uri:
                # use the user-specified router config file
                bash_writer.write(
                    f'export ZENOH_ROUTER_CONFIG_URI={middleware_config.zenoh_router_config_uri}'  # noqa: E501
                )
            else:
                # use the default router config file
                bash_writer.write(
                    'export ZENOH_ROUTER_CONFIG_URI="$(ros2 pkg prefix rmw_zenoh_cpp)/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5"'  # noqa: E501
                )
            bash_writer.write(
                'ros2 run rmw_zenoh_cpp rmw_zenohd\n'
            )
        else:
            bash_writer.add_echo(
                'RMW Implementation is not Zenoh. ' +
                'If this was launched as a service then the service will now end.'
            )

        bash_writer.close()
