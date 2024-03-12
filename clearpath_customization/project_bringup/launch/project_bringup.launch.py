from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package Directory
    pkg_project_bringup = FindPackageShare('project_bringup')

    # Launch File
    device_launch = PathJoinSubstitution([pkg_project_bringup, 'launch', 'device.launch.py'])

    # Include Launch
    include_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([device_launch]))

    # Launch Description
    ld = LaunchDescription()
    ld.add_action(include_launch)
    return ld
