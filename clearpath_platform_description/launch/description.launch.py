
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
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
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_manipulation_controllers = LaunchConfiguration('use_manipulation_controllers')
    use_platform_controllers = LaunchConfiguration('use_platform_controllers')

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

    arg_use_fake_hardware = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use fake hardware if true'
    )

    arg_use_manipulation_controllers = DeclareLaunchArgument(
        'use_manipulation_controllers',
        default_value='false',
        description='Use manipulation controllers if true'
    )

    arg_use_platform_controllers = DeclareLaunchArgument(
        'use_platform_controllers',
        default_value='true',
        description='Use platform controllers if true'
    )

    # Paths
    robot_urdf = PathJoinSubstitution([
        setup_path, 'robot.urdf.xacro'])
    config_control = PathJoinSubstitution([
        setup_path, 'platform/config/control.yaml'])

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
            'gazebo_controllers:=',
            config_control,
            ' ',
            'namespace:=',
            namespace,
            ' ',
            'use_fake_hardware:=',
            use_fake_hardware,
            ' ',
            'use_manipulation_controllers:=',
            use_manipulation_controllers,
            ' ',
            'use_platform_controllers:=',
            use_platform_controllers,
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

    group_action_state_publishers = GroupAction([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': use_sim_time,
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
    ld.add_action(arg_setup_path)
    ld.add_action(arg_namespace)
    ld.add_action(arg_use_fake_hardware)
    ld.add_action(arg_use_manipulation_controllers)
    ld.add_action(arg_use_platform_controllers)
    ld.add_action(arg_robot_description_command)
    # Nodes
    ld.add_action(group_action_state_publishers)
    return ld
