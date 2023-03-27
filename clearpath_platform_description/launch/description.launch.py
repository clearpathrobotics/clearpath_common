
from launch import  LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Configurations
    robot_model = LaunchConfiguration('robot_model')
    is_sim = LaunchConfiguration('is_sim')

    # Launch Arguments
    arg_robot_model = DeclareLaunchArgument(
        'robot_model',
        choices=['a200', 'j100'],
        default_value='a200'
    )

    arg_is_sim = DeclareLaunchArgument(
        'is_sim',
        choices=['true', 'false'],
        default_value='false'
    )

    # Packages
    pkg_clearpath_platform_description = FindPackageShare('clearpath_platform_description')
    robot_model = LaunchConfiguration('robot_model', default='a200')

    # Paths
    dir_robot_description = PathJoinSubstitution([
        pkg_clearpath_platform_description, 'urdf', robot_model])


    # Get URDF via xacro
    arg_robot_description_command = DeclareLaunchArgument(
        'robot_description_command',
        default_value=[
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            '/home/rkreinin/clearpath_ws/a200-0001',
            #PathJoinSubstitution([dir_robot_description, robot_model]),
            '.urdf.xacro'
        ]
    )

    robot_description_content = ParameterValue(
        Command(LaunchConfiguration('robot_description_command')),
        value_type=str
    )

    node_robot_state_publisher = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                      }],
                                      remappings=[('/tf', 'tf'),('/tf_static', 'tf_static')])

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        #parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    ld = LaunchDescription()
    # Args
    ld.add_action(arg_robot_model)
    ld.add_action(arg_robot_description_command)
    ld.add_action(arg_is_sim)
    ld.add_action(arg_robot_description_command)
    # Nodes
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_joint_state_publisher)
    return ld