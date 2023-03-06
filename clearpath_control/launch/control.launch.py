from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Packages
    pkg_clearpath_control = FindPackageShare('clearpath_control')
    pkg_clearpath_description = FindPackageShare('clearpath_description')

    # Launch Configurations
    robot_model = LaunchConfiguration('robot_model')
    is_sim = LaunchConfiguration('is_sim')

    # Paths
    robot_config_dir = PathJoinSubstitution([
        pkg_clearpath_control, 'config', robot_model])
    robot_description_dir = PathJoinSubstitution([
        pkg_clearpath_description, 'urdf', robot_model])

    # Configs
    config_platform_ekf = [
        robot_config_dir,
        'localization.yaml'
    ]

    config_imu_filter = [
        robot_config_dir,
        'imu_filter.yaml'
    ]

    config_platform_velocity_controller = [
        robot_config_dir,
        'control.yaml'
    ]

    # Launch Arguments
    robot_model_command_arg = DeclareLaunchArgument(
        'robot_model',
        choices=['husky', 'jackal'],
        default_value='husky'
    )

    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        choices=['true', 'false'],
        default_value='false'
    )

    robot_description_command_arg = DeclareLaunchArgument(
        'robot_description_command',
        default_value=[
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([robot_description_dir, robot_model]),
            '.urdf.xacro'
        ]
    )

    robot_description_content = ParameterValue(
        Command(LaunchConfiguration('robot_description_command')),
        value_type=str
    )

    # Localization
    localization_group_action = GroupAction([
        # Extended Kalman Filter
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[config_platform_ekf],
        ),

        # Madgwick Filter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_node',
            output='screen',
            parameters=[config_imu_filter]
        )
    ])

    # ROS2 Controllers
    control_group_action = GroupAction([
        # ROS2 Control
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description_content},
                        config_platform_velocity_controller],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            condition=UnlessCondition(is_sim)
        ),

        # Joint State Broadcaster
        Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),

        # Velocity Controller
        Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=['platform_velocity_controller'],
            output='screen',
        )
    ])

    ld = LaunchDescription()
    ld.add_action(robot_model_command_arg)
    ld.add_action(robot_description_command_arg)
    ld.add_action(is_sim_arg)
    ld.add_action(localization_group_action)
    ld.add_action(control_group_action)
    return ld
