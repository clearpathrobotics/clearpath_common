# Software License Agreement (BSD)
#
# @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# @author    Hilary Luo <hluo@clearpathrobotics.com>
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    aggregator_parameters = LaunchConfiguration('aggregator_parameters')
    updater_parameters = LaunchConfiguration('updater_parameters')

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace, applied to diagnostic nodes and topics')

    arg_aggregator_params = DeclareLaunchArgument(
        'aggregator_parameters',
        default_value=PathJoinSubstitution([
          FindPackageShare('clearpath_diagnostics'),
          'config',
          'diagnostic_aggregator.yaml'
        ]))

    arg_updater_params = DeclareLaunchArgument(
        'updater_parameters',
        default_value=PathJoinSubstitution([
          FindPackageShare('clearpath_diagnostics'),
          'config',
          'diagnostic_updater.yaml'
        ]))

    diagnostics_action = GroupAction(
        [
            # Aggregator
            Node(
                package='diagnostic_aggregator',
                executable='aggregator_node',
                name='diagnostic_aggregator',
                namespace=namespace,
                output='screen',
                parameters=[aggregator_parameters],
                remappings=[
                    ('/diagnostics', 'diagnostics'),
                    ('/diagnostics_agg', 'diagnostics_agg'),
                    ('/diagnostics_toplevel_state', 'diagnostics_toplevel_state'),
                ],
            ),
            # Updater
            Node(
                package='clearpath_diagnostics',
                executable='clearpath_diagnostic_updater',
                name='clearpath_diagnostic_updater',
                namespace=namespace,
                output='screen',
                parameters=[updater_parameters],
                remappings=[
                    ('/diagnostics', 'diagnostics'),
                    ('/diagnostics_agg', 'diagnostics_agg'),
                    ('/diagnostics_toplevel_state', 'diagnostics_toplevel_state'),
                ],
            ),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(arg_namespace)
    ld.add_action(arg_aggregator_params)
    ld.add_action(arg_updater_params)
    ld.add_action(diagnostics_action)
    return ld
