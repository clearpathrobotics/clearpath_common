#!/usr/bin/env -S ros2 launch
import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from clearpath_config.clearpath_config import ClearpathConfig


def launch_setup(context, *args, **kwargs):
    # Launch Configurations
    setup_path = LaunchConfiguration('setup_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path_context = setup_path.perform(context)

    # Namespace
    namespace = ClearpathConfig(
        os.path.join(setup_path_context, 'robot.yaml')
    ).get_namespace()

    # Robot Description
    robot_description = {
        'robot_description': xacro.process_file(
            os.path.join(setup_path_context, 'robot.urdf.xacro')
        ).toxml()
    }

    # Semantic Robot Description
    robot_description_semantic = {
        'robot_description_semantic': xacro.process_file(
            os.path.join(setup_path_context, 'robot.srdf')
        ).toxml()
    }

    return [
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='log',
            namespace=namespace,
            parameters=[
                os.path.join(setup_path_context, 'manipulators', 'config', 'moveit.yaml'),
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
    arg_setup_path = DeclareLaunchArgument(
        'setup_path',
        default_value=[EnvironmentVariable('HOME'), '/clearpath/'],
        description='Clearpath setup path'
    )
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='use_sim_time'
    )
    ld = LaunchDescription()
    ld.add_action(arg_setup_path)
    ld.add_action(arg_use_sim_time)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
