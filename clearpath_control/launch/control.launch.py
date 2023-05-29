from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import LaunchConfigurationEquals, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Launch Configurations
    platform_model = LaunchConfiguration('platform_model')
    setup_path = LaunchConfiguration('setup_path')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch Arguments
    arg_platform_model = DeclareLaunchArgument(
        'platform_model',
        choices=['a200', 'j100'],
        default_value='a200'
    )

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

    # Packages
    pkg_clearpath_control = FindPackageShare('clearpath_control')

    # Paths
    dir_robot_config = PathJoinSubstitution([
        pkg_clearpath_control, 'config', platform_model])

    # Configs
    config_platform_velocity_controller = [
        dir_robot_config,
        '/control.yaml'
    ]

    arg_robot_description_command = DeclareLaunchArgument(
        'robot_description_command',
        default_value=[
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            setup_path,
            'robot.urdf.xacro',
            ' ',
            'is_sim:=',
            use_sim_time
        ]
    )

    robot_description_content = ParameterValue(
        Command(LaunchConfiguration('robot_description_command')),
        value_type=str
    )

    # ROS2 Controllers
    action_control_group = GroupAction([
        # ROS2 Control Node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description_content},
                        config_platform_velocity_controller],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            remappings=[
              ('platform_velocity_controller/odom', 'platform/odom'),
              ('platform_velocity_controller/cmd_vel_unstamped', 'platform/cmd_vel_unstamped'),
              ('joint_states', 'platform/joint_states'),
              ('dynamic_joint_states', 'platform/dynamic_joint_states'),
            ],
            condition=UnlessCondition(use_sim_time)
        ),

        # Joint State Broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),

        # Velocity Controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['platform_velocity_controller'],
            output='screen',
        )
    ])

    ld = LaunchDescription()
    ld.add_action(arg_platform_model)
    ld.add_action(arg_setup_path)
    ld.add_action(arg_robot_description_command)
    ld.add_action(arg_use_sim_time)
    ld.add_action(action_control_group)
    return ld
