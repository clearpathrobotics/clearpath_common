from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    lc = LaunchContext()
    ld = LaunchDescription()

    # Launch Arguments
    arg_robot_model = DeclareLaunchArgument(
        'robot_model',
        choices=['a200', 'j100'],
        default_value='a200'
    )

    arg_joy_type = DeclareLaunchArgument(
        'joy_type',
        choices=['logitech', 'ps4'],
        default_value='ps4'
    )

    # Launch Configurations
    robot_model = LaunchConfiguration('robot_model')
    joy_type = EnvironmentVariable('CPR_JOY_TYPE', default_value='ps4')

    # Packages
    pkg_clearpath_control = FindPackageShare('clearpath_control')

    # Paths
    dir_robot_config = PathJoinSubstitution([
        pkg_clearpath_control, 'config', robot_model])
    
    file_config_joy = 'teleop_' + joy_type.perform(lc) + '.yaml'

    filepath_config_joy = PathJoinSubstitution([
        dir_robot_config,
        file_config_joy]
    )

    node_joy = Node(
        namespace='joy_teleop',
        package='joy',
        executable='joy_node',
        output='screen',
        name='joy_node',
        parameters=[filepath_config_joy]
    )

    node_teleop_twist_joy = Node(
        namespace='joy_teleop',
        package='teleop_twist_joy',
        executable='teleop_node',
        output='screen',
        name='teleop_twist_joy_node',
        parameters=[filepath_config_joy]
    )


    ld.add_action(arg_robot_model)
    ld.add_action(arg_joy_type)
    ld.add_action(node_joy)
    ld.add_action(node_teleop_twist_joy)
    return ld
