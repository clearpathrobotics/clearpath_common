
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import (Command, FindExecutable,
                                  PathJoinSubstitution, LaunchConfiguration)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Launch Configurations
    output_path = LaunchConfiguration(
        'output_path',
        default='/etc/clearpath/')
    robot_description_command = LaunchConfiguration(
        'robot_description_command',
        default=[
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            output_path,
            'robot.urdf.xacro'
        ])

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch Arguments
    arg_output_path = DeclareLaunchArgument(
        'output_path',
        default_value='/etc/clearpath/'
    )

    # Get URDF via xacro
    arg_robot_description_command = DeclareLaunchArgument(
        'robot_description_command',
        default_value=[
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            output_path,
            'robot.urdf.xacro',
            ' ',
            'is_sim:=',
            use_sim_time
        ]
    )

    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='use_sim_time')

    robot_description_content = ParameterValue(
        Command(robot_description_command),
        value_type=str
    )

    group_action_state_publishers = GroupAction([

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_content,
            }],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('joint_states', 'platform/joint_states')]
        ),
    ])

    ld = LaunchDescription()
    # Args
    ld.add_action(arg_use_sim_time)
    ld.add_action(arg_output_path)
    ld.add_action(arg_robot_description_command)
    # Nodes
    ld.add_action(group_action_state_publishers)
    return ld
