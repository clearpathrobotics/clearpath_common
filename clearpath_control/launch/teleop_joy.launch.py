# Software License Agreement (BSD)
#
# @author    Chris Iverach-Brereton <civerachb@clearpathrobotics.com>
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Launch Configurations
    setup_path = LaunchConfiguration('setup_path')
    use_sim_time = LaunchConfiguration('use_sim_time')

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
    config_teleop_joy = [
        dir_platform_config,
        '/teleop_joy.yaml'
    ]

    #node_bt_cutoff = Node(
    #    package='clearpath_bt_joy',
    #    executable='clearpath_bt_joy_cutoff_node',
    #    name='bt_cutoff_node',
    #    parameters=[
    #        config_teleop_joy,
    #        {'use_sim_time': use_sim_time},
    #    ],
    #    remappings=[
    #        ('bt_quality_stop', 'joy_teleop/bt_quality_stop'),
    #        ('quality', 'joy_teleop/quality'),
    #    ],
    #    respawn=True,
    #)

    node_joy = Node(
        package='joy_linux',
        executable='joy_linux_node',
        output='screen',
        name='joy_node',
        parameters=[
            config_teleop_joy,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/diagnostics', 'diagnostics'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('joy', 'joy_teleop/joy'),
            ('joy/set_feedback', 'joy_teleop/joy/set_feedback'),
        ],
        respawn=True,
    )

    node_teleop_twist_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        output='screen',
        name='teleop_twist_joy_node',
        parameters=[
            config_teleop_joy,
            {'use_sim_time': use_sim_time},
            {'publish_stamped_twist': True},
        ],
        remappings=[
            ('joy', 'joy_teleop/joy'),
            ('cmd_vel', 'joy_teleop/cmd_vel'),
        ]
    )

    #node_twist_mux = Node(
    #    package='twist_mux',
    #    executable='twist_mux',
    #    output='screen',
    #    name='teleop_cutoff_mux',
    #    remappings={
    #        ('cmd_vel_out', 'joy_teleop/cmd_vel'),
    #        ('/diagnostics', 'diagnostics'),
    #        ('/tf', 'tf'),
    #        ('/tf_static', 'tf_static'),
    #    },
    #    parameters=[
    #        {'use_sim_time': use_sim_time},
    #        {'use_stamped': True},
    #        {'topics.joy.topic': 'joy_teleop/_cmd_vel'},
    #        {'topics.joy.timeout': 0.5},
    #        {'topics.joy.priority': 10},
    #        {'locks.bt_quality.topic': 'joy_teleop/bt_quality_stop'},
    #        {'locks.bt_quality.timeout': 1.0},
    #        {'locks.bt_quality.priority': 255},
    #    ]
    #)


    # Create launch description and add actions
    ld = LaunchDescription()
    ld.add_action(arg_setup_path)
    ld.add_action(arg_use_sim_time)
    #ld.add_action(node_bt_cutoff)
    ld.add_action(node_joy)
    ld.add_action(node_teleop_twist_joy)
    #ld.add_action(node_twist_mux)
    return ld
