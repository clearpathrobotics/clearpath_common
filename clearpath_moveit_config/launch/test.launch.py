from launch import LaunchDescription 
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define the path to your demo.launch.py file
    demo_launch_file = os.path.join(get_package_share_directory('clearpath_moveit_config'), 'launch', 'demo.launch.py')

    # Wrap the demo launch in a namespace using PushRosNamespace
    return LaunchDescription([
        # Apply the namespace to all nodes within the group
        PushRosNamespace('a200_0000'),
        
        # Include the demo launch file inside the namespace
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(demo_launch_file),
        ),
    ])
