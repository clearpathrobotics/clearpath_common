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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    # Launch Configurations
    setup_path = LaunchConfiguration('setup_path')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch Arguments
    arg_setup_path = DeclareLaunchArgument(
        'setup_path',
        default_value='/etc/clearpath/'
    )

    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        choices=['true', 'false'],
        default_value='false',
        description='Use simulation time'
    )

    # Paths
    dir_platform_config = PathJoinSubstitution([
        setup_path, 'platform/config'])

    # Configs
    config_twist_mux = [
        dir_platform_config,
        '/twist_mux.yaml'
    ]

    config_interactive_markers = [
        dir_platform_config,
        '/teleop_interactive_markers.yaml'
    ]

    node_interactive_marker_twist_server = Node(
        package='interactive_marker_twist_server',
        executable='marker_server',
        name='twist_server_node',
        remappings=[('cmd_vel', 'twist_marker_server/cmd_vel'),
                    ('twist_server/feedback', 'twist_marker_server/feedback'),
                    ('twist_server/update', 'twist_marker_server/update')],
        parameters=[
            config_interactive_markers,
            {'use_sim_time': use_sim_time}],
        output='screen',
    )

    node_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings={
            ('cmd_vel_out', 'platform/cmd_vel_unstamped'),
            ('/diagnostics', 'diagnostics'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        },
        parameters=[
            config_twist_mux,
            {'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()
    ld.add_action(arg_setup_path)
    ld.add_action(arg_use_sim_time)
    ld.add_action(node_interactive_marker_twist_server)
    ld.add_action(node_twist_mux)
    return ld
