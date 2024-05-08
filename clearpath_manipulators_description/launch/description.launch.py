
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, FindExecutable,
                                  PathJoinSubstitution, LaunchConfiguration)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Launch Configurations
    setup_path = LaunchConfiguration('setup_path')
    robot_description_command = LaunchConfiguration('robot_description_command')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    # Launch Arguments
    arg_setup_path = DeclareLaunchArgument(
        'setup_path',
        default_value='/etc/clearpath/'
    )

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )

    # Paths
    robot_urdf = PathJoinSubstitution([
        setup_path, 'robot.urdf.xacro'])

    # Get URDF via xacro
    arg_robot_description_command = DeclareLaunchArgument(
        'robot_description_command',
        default_value=[
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            robot_urdf,
            ' ',
            'is_sim:=',
            use_sim_time,
            ' ',
            'namespace:=',
            namespace,
            ' ',
            'use_fake_hardware:=',
            'false',
            ' ',
            'use_manipulation_controllers:=',
            'true',
            ' ',
            'use_platform_controllers:=',
            'false',
        ]
    )

    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        choices=['true', 'false'],
        default_value='false',
        description='Use simulation time'
    )

    robot_description_content = ParameterValue(
        Command(robot_description_command),
        value_type=str
    )

    # Manipulator State Publisher
    manipulator_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('dynamic_joint_states',
                PathJoinSubstitution([
                    '/', namespace, 'platform', 'dynamic_joint_states'
                ])),
            ('joint_states',
                PathJoinSubstitution([
                    '/', namespace, 'platform', 'joint_states'
                ])),
        ]
    )

    ld = LaunchDescription()
    # Args
    ld.add_action(arg_use_sim_time)
    ld.add_action(arg_setup_path)
    ld.add_action(arg_namespace)
    ld.add_action(arg_robot_description_command)
    # Nodes
    ld.add_action(manipulator_state_publisher)
    return ld
