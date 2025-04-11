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
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    # Packages
    pkg_clearpath_manipulators_description = FindPackageShare('clearpath_manipulators_description')
    pkg_clearpath_manipulators = FindPackageShare('clearpath_manipulators')

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

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )

    arg_launch_moveit = DeclareLaunchArgument(
        'launch_moveit',
        choices=['true', 'false'],
        default_value='false',
        description='Launch MoveIt'
    )

    arg_control_delay = DeclareLaunchArgument(
        'control_delay',
        default_value='0.0',
        description='Control launch delay in seconds.'
    )

    arg_moveit_delay = DeclareLaunchArgument(
        'moveit_delay',
        default_value='1.0',
        description='MoveIt launch delay in seconds.'
    )

    # Launch Configurations
    setup_path = LaunchConfiguration('setup_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    launch_moveit = LaunchConfiguration('launch_moveit')
    control_delay = LaunchConfiguration('control_delay')
    moveit_delay = LaunchConfiguration('moveit_delay')

    # Launch files
    launch_file_manipulators_description = PathJoinSubstitution([
      pkg_clearpath_manipulators_description,
      'launch',
      'description.launch.py'])

    launch_file_control = PathJoinSubstitution([
      pkg_clearpath_manipulators,
      'launch',
      'control.launch.py'])

    launch_file_moveit = PathJoinSubstitution([
        pkg_clearpath_manipulators,
        'launch',
        'moveit.launch.py'])

    group_manipulators_action = GroupAction(
        actions=[
            PushRosNamespace(PathJoinSubstitution([namespace, 'manipulators'])),
            IncludeLaunchDescription(
              PythonLaunchDescriptionSource(launch_file_manipulators_description),
              launch_arguments=[
                  ('namespace', namespace),
                  ('setup_path', setup_path),
                  ('use_sim_time', use_sim_time),
              ]
            ),

            # Launch clearpath_control/control.launch.py which is just robot_localization.
            IncludeLaunchDescription(
              PythonLaunchDescriptionSource(launch_file_control),
              launch_arguments=[
                  ('namespace', namespace),
                  ('setup_path', setup_path),
                  ('use_sim_time', use_sim_time),
              ]
            ),
        ]
    )

    control_delayed = TimerAction(
        period=control_delay,
        actions=[group_manipulators_action]
    )

    # Launch MoveIt
    moveit_node_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_moveit),
        launch_arguments=[
            ('setup_path', setup_path),
            ('use_sim_time', use_sim_time)
        ],
        condition=IfCondition(launch_moveit)
    )

    moveit_delayed = TimerAction(
        period=moveit_delay,
        actions=[moveit_node_action]
    )

    ld = LaunchDescription()
    ld.add_action(arg_setup_path)
    ld.add_action(arg_use_sim_time)
    ld.add_action(arg_namespace)
    ld.add_action(arg_launch_moveit)
    ld.add_action(arg_control_delay)
    ld.add_action(arg_moveit_delay)
    ld.add_action(control_delayed)
    ld.add_action(moveit_delayed)
    return ld
