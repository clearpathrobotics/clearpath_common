# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2025, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import datetime
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs):
    # Launch Configurations
    namespace = str(
        LaunchConfiguration('namespace').perform(context))
    parameters = str(
        LaunchConfiguration('parameters').perform(context))
    enable_recorder = bool(
        LaunchConfiguration('enable_recorder').perform(context) == 'true')

    # Extract Parameters
    content = yaml.safe_load(open(parameters))
    params = {}
    for i in content:
        if 'ros__parameters' in content[i]:
            params = content[i]['ros__parameters']
    found = False
    while not found:
        if 'ros__parameters' in content:
            params = content['ros__parameters']
            found = True
        else:
            content = content.pop(list(content)[0])
        if not isinstance(content, dict):
            break
    if not found:
        if enable_recorder:
            return [
                Node(
                    package='rosbag2_transport',
                    executable='recorder',
                    name='recorder',
                    output='screen',
                    namespace=namespace,
                    parameters=[parameters],
                )
            ]
        else:
            return []

    # Create Timestamp Directory
    flat = False
    path = None
    if 'storage' in params:
        if 'uri' in params['storage']:
            path = params['storage']['uri']
    elif 'storage.uri' in params:
        path = params['storage.uri']
        flat = True
    if not path:
        path = '/etc/clearpath/bags'
    if not os.path.exists(path):
        os.makedirs(path)
    timestamp_path = os.path.join(
        path, 'rosbag2_' + datetime.datetime.now().strftime('%Y_%m_%d-%H_%M_%S'))
    if flat:
        params['storage.uri'] = timestamp_path
    else:
        params['storage']['uri'] = timestamp_path

    # Node
    node = Node(
        package='rosbag2_transport',
        executable='recorder',
        name='recorder',
        output='screen',
        namespace=namespace,
        parameters=[params],
    )

    nodes = []
    if enable_recorder:
        nodes.append(node)
    return nodes


def generate_launch_description():
    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )

    arg_parameters = DeclareLaunchArgument(
        'parameters',
        default_value=PathJoinSubstitution([
          FindPackageShare('clearpath_diagnostics'),
          'config',
          'recorder.yaml'
        ]),
        description='Recorder node parameters'
    )

    arg_enable_recorder = DeclareLaunchArgument(
        'enable_recorder',
        default_value='true',
        choices=['true', 'false'],
        description='Enable recording'
    )

    ld = LaunchDescription()
    ld.add_action(arg_namespace)
    ld.add_action(arg_parameters)
    ld.add_action(arg_enable_recorder)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
