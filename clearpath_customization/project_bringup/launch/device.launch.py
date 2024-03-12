from clearpath_config.clearpath_config import ClearpathConfig
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Node Name, Package and Executable
    name = "device"
    package = "realsense_camera"
    executable = "realsense_camera_node"

    # Namespace from Clearpath Config
    namespace = ClearpathConfig('/etc/clearpath/robot.yaml').system.namespace

    # Project Directory
    pkg_project_bringup = FindPackageShare('project_bringup')

    # Parameter File
    device_params = PathJoinSubstitution([pkg_project_bringup, 'config', 'device.yaml'])

    # Node
    device_node = Node(
        name=name,
        namespace=namespace,
        package=package,
        executable=executable,
        parameters=[device_params],
        output='screen',
    )

    # Launch Description
    ld = LaunchDescription()
    ld.add_action(device_node)
    return ld
