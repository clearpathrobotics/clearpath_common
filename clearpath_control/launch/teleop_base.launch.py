from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Launch Configurations
    robot_model = LaunchConfiguration('robot_model')

    # Launch Arguments
    arg_robot_model = DeclareLaunchArgument(
        'robot_model',
        choices=['a200', 'j100'],
        default_value='a200'
    )


    # Packages
    pkg_clearpath_control = FindPackageShare('clearpath_control')

    # Paths
    dir_robot_config = PathJoinSubstitution([
        pkg_clearpath_control, 'config', robot_model])

    # Common Configs
    config_twist_mux = PathJoinSubstitution([
        pkg_clearpath_control,
        'config',
        'twist_mux.yaml']
    )

    # Platform Configs
    config_interactive_markers = [
        dir_robot_config,
        '/teleop_interactive_markers.yaml'
    ]

    node_interactive_marker_twist_server = Node(
        package='interactive_marker_twist_server',
        executable='marker_server',
        name='twist_server_node',
        remappings={('cmd_vel', 'twist_marker_server/cmd_vel')},
        parameters=[config_interactive_markers],
        output='screen',
    )

    node_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings={('/cmd_vel_out', '/platform_velocity_controller/cmd_vel_unstamped')},
        parameters=[config_twist_mux]
    )

    ld = LaunchDescription()
    ld.add_action(arg_robot_model)
    ld.add_action(node_interactive_marker_twist_server)
    ld.add_action(node_twist_mux)
    return ld
