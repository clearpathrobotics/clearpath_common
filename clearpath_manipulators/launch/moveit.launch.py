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
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Launch Configurations
    namespace = LaunchConfiguration('namespace')
    robot_urdf = LaunchConfiguration('robot_urdf')
    robot_srdf = LaunchConfiguration('robot_srdf')
    config_moveit = LaunchConfiguration('config_moveit')
    use_sim_time = LaunchConfiguration('use_sim_time')

    namespace_context = namespace.perform(context)
    robot_urdf_context = robot_urdf.perform(context)
    robot_srdf_context = robot_srdf.perform(context)
    config_moveit_context = config_moveit.perform(context)

    # Robot Description
    robot_description = {
        'robot_description': xacro.process_file(
            robot_urdf_context
        ).toxml()
    }

    # Semantic Robot Description
    robot_description_semantic = {
        'robot_description_semantic': xacro.process_file(
            robot_srdf_context
        ).toxml()
    }

    return [
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='log',
            namespace=namespace_context,
            parameters=[
                config_moveit_context,
                robot_description,
                robot_description_semantic,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('joint_states', 'platform/joint_states'),
            ]
        )
    ]


def generate_launch_description():
    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )
    arg_robot_urdf = DeclareLaunchArgument(
        'robot_urdf',
        description='Path to the robot URDF xacro file'
    )
    arg_robot_srdf = DeclareLaunchArgument(
        'robot_srdf',
        description='Path to the robot SRDF file'
    )
    arg_config_moveit = DeclareLaunchArgument(
        'config_moveit',
        description='Path to the MoveIt configuration YAML file'
    )
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='use_sim_time'
    )
    ld = LaunchDescription()
    ld.add_action(arg_namespace)
    ld.add_action(arg_robot_urdf)
    ld.add_action(arg_robot_srdf)
    ld.add_action(arg_config_moveit)
    ld.add_action(arg_use_sim_time)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
