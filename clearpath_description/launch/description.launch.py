
from launch import  LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Xacro isn't used directly here but it is called to generate the URDF.
import xacro



def generate_launch_description():
    lc = LaunchContext()
    ld = LaunchDescription()

    robot_model = LaunchConfiguration('robot_model', default='husky')

    # Get URDF via xacro
    arg_robot_description_command = DeclareLaunchArgument(
        'robot_description_command',
        default_value=[
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(
                    'clearpath_description'),
                    'urdf',
                    robot_model.perform(lc) + '/' + robot_model.perform(lc) + '.urdf.xacro'
                ]
            )
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


    ld.add_action(arg_robot_description_command)
    ld.add_action(node_robot_state_publisher)
    return ld