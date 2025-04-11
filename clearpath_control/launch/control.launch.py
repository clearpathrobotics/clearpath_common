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
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from clearpath_config.common.utils.dictionary import unflatten_dict
from clearpath_config.common.utils.yaml import read_yaml

REMAPPINGS = [
    ('joint_states', 'platform/joint_states'),
    ('dynamic_joint_states', 'platform/dynamic_joint_states'),
    ('platform_velocity_controller/odom', 'platform/odom'),
    ('platform_velocity_controller/odometry', 'platform/odom'),
    ('platform_velocity_controller/cmd_vel', 'platform/cmd_vel'),
    ('platform_velocity_controller/cmd_vel_out', 'platform/cmd_vel_out'),
    ('platform_velocity_controller/reference', 'platform/cmd_vel'),
    ('platform_velocity_controller/transition_event', 'platform/transition_event'),
    ('/diagnostics', 'diagnostics'),
    ('/tf', 'tf'),
    ('/tf_static', 'tf_static'),
    ('~/robot_description', 'robot_description'),
]


def launch_setup(context, *args, **kwargs):
    setup_path = LaunchConfiguration('setup_path')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Controllers
    config_control = PathJoinSubstitution([
        setup_path, 'platform/config/control.yaml'])

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
        remappings=REMAPPINGS,
        condition=UnlessCondition(use_sim_time)
    ))
    # Add Joint State Broadcaster
    controllers.append(Node(
        package='controller_manager',
        executable='spawner',
        arguments=['--controller-manager-timeout', '60', 'joint_state_broadcaster'],
        output='screen',
        additional_env={'ROS_SUPER_CLIENT': 'True'},
    ))
    # Add Platform Velocity Controller
    controllers.append(Node(
        package='controller_manager',
        executable='spawner',
        arguments=['--controller-manager-timeout', '60', 'platform_velocity_controller'],
        output='screen',
        additional_env={'ROS_SUPER_CLIENT': 'True'},
    ))
    # If Simulation, Add All Listed Controllers
    for namespace in context_control:
        for controller in context_control[namespace]:
            if ('controller' not in controller or
                    'manager' in controller or
                    'platform' in controller):
                continue
            controllers.append(Node(
                name=controller,
                package='controller_manager',
                executable='spawner',
                arguments=['--controller-manager-timeout', '60', controller],
                output='screen',
                additional_env={'ROS_SUPER_CLIENT': 'True'},
                condition=IfCondition(use_sim_time),
            ))
    return [GroupAction(controllers)]


def generate_launch_description():
    # Launch Configurations
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

    ld = LaunchDescription([
        arg_setup_path,
        arg_use_sim_time
    ])
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
