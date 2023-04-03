
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, GroupAction, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Packages
    pkg_clearpath_viz = FindPackageShare('clearpath_viz')

    # Launch Configurations
    config_file = LaunchConfiguration('config_file', default='/home/rkreinin/clearpath_ws/src/clearpath_config/clearpath_config/sample/a200_config.yaml')
    output_path = LaunchConfiguration('output_path', default='/home/rkreinin/clearpath_ws/')
    robot_description_command = LaunchConfiguration('robot_description_command', default=[
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            output_path,
            'a200-0001.urdf.xacro'
        ])
    # Launch Arguments
    arg_config_file = DeclareLaunchArgument(
        'config_file',
        default_value='/home/rkreinin/clearpath_ws/src/clearpath_config/clearpath_config/sample/a200_config.yaml'
    )

    arg_output_path = DeclareLaunchArgument(
        'output_path',
        default_value='/home/rkreinin/clearpath_ws/'
    )

    # Get URDF via xacro
    arg_robot_description_command = DeclareLaunchArgument(
        'robot_description_command',
        default_value=[
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            output_path,
            'a200-0001.urdf.xacro'
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
        arguments=[config_file, output_path]
    )

    group_action_state_publishers = GroupAction([

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_content,
            }],
            remappings=[('/tf', 'tf'),('/tf_static', 'tf_static')]
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            #parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([pkg_clearpath_viz, 'launch', 'view_model.launch.py'])
        #     ])
        # )
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
