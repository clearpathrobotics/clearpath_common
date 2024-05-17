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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from clearpath_config.common.utils.dictionary import unflatten_dict
from clearpath_config.common.utils.yaml import read_yaml


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace')
    setup_path = LaunchConfiguration('setup_path')

    # Controllers
    config_control = PathJoinSubstitution([
        setup_path, 'manipulators/config/control.yaml'])

    context_control = unflatten_dict(read_yaml(config_control.perform(context)))

    controllers = []

    # Add Controller Manager
    controllers.append(Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[config_control],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        remappings=[
            ('~/robot_description', 'robot_description'),
            ('dynamic_joint_states',
                PathJoinSubstitution([
                    '/', namespace, 'platform', 'dynamic_joint_states'
                ])),
            ('joint_states',
                PathJoinSubstitution([
                    '/', namespace, 'platform', 'joint_states'
                ])),
        ],
    ))
    # Add Joint State Broadcaster
    controllers.append(Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster', '--controller-manager-timeout', '60',
        ],
        output='screen',
    ))
    # If Simulation, Add All Listed Controllers
    for namespace in context_control:
        for controller in context_control[namespace]:
            if ('controller' not in controller or
                    'manager' in controller or
                    'platform' in controller):
                continue
            controllers.append(Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    controller, '--controller-manager-timeout', '60',
                ],
                output='screen',
            ))
    return [GroupAction(controllers)]


def generate_launch_description():
    # Launch Configurations
    arg_setup_path = DeclareLaunchArgument(
        'setup_path',
        default_value='/etc/clearpath/'
    )

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )

    ld = LaunchDescription([
        arg_namespace,
        arg_setup_path,
    ])
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
