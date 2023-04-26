
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, RegisterEventHandler
from launch.substitutions import (Command, FindExecutable,
                                  PathJoinSubstitution, LaunchConfiguration)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # Launch Configurations
    config_file = LaunchConfiguration(
        'config_file',
        default='/etc/clearpath/robot.yaml')
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

    # Launch Arguments
    arg_config_file = DeclareLaunchArgument(
        'config_file',
        default_value='/etc/clearpath/robot.yaml'
    )

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
            'robot.urdf.xacro'
        ]
    )

    robot_description_content = ParameterValue(
        Command(robot_description_command),
        value_type=str
    )

    node_description_generator = Node(
        package='clearpath_description_generator',
        executable='description_generator',
        name='description_generator',
        output='screen',
        arguments=['-c', config_file, '-o', output_path]
    )

    group_action_state_publishers = GroupAction([

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_content,
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),
    ])

    event_generate_description = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_description_generator,
            on_exit=[group_action_state_publishers]
        )
    )

    ld = LaunchDescription()
    # Args
    ld.add_action(arg_config_file)
    ld.add_action(arg_output_path)
    ld.add_action(arg_robot_description_command)
    # Nodes
    ld.add_action(node_description_generator)
    ld.add_action(event_generate_description)
    return ld
