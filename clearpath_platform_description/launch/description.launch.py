
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
    tf_topic = LaunchConfiguration('tf_topic')
    frame_prefix = LaunchConfiguration('frame_prefix')

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

    arg_tf_topic = DeclareLaunchArgument(
        'tf_topic',
        default_value='/tf',
        description='TF topic'
    )

    arg_frame_prefix = DeclareLaunchArgument(
        'frame_prefix',
        default_value=[namespace,'/'],
        description='TF prefix for all TFs generated')

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
            namespace
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
                'frame_prefix': frame_prefix
            }],
            remappings=[
                ('/tf', tf_topic),
                ('/tf_static', [tf_topic, '_static']),
                ('joint_states', 'platform/joint_states')]
        ),
    ])

    ld = LaunchDescription()
    # Args
    ld.add_action(arg_use_sim_time)
    ld.add_action(arg_setup_path)
    ld.add_action(arg_namespace)
    ld.add_action(arg_tf_topic)
    ld.add_action(arg_frame_prefix)
    ld.add_action(arg_robot_description_command)
    # Nodes
    ld.add_action(group_action_state_publishers)
    return ld
