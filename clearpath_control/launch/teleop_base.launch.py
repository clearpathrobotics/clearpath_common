from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Launch Configurations
    platform_model = LaunchConfiguration('platform_model')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch Arguments
    arg_platform_model = DeclareLaunchArgument(
        'platform_model',
        choices=['a200', 'j100'],
        default_value='a200'
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
        remappings=[('cmd_vel', 'twist_marker_server/cmd_vel'),
                    ('twist_server/feedback', 'twist_marker_server/feedback'),
                    ('twist_server/update', 'twist_marker_server/update')],
        parameters=[
            config_interactive_markers,
            {'use_sim_time': use_sim_time}],
        output='screen',
    )

    node_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings={('cmd_vel_out', 'platform/cmd_vel_unstamped')},
        parameters=[
            config_twist_mux,
            {'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()
    ld.add_action(arg_platform_model)
    ld.add_action(arg_use_sim_time)
    ld.add_action(node_interactive_marker_twist_server)
    ld.add_action(node_twist_mux)
    return ld
