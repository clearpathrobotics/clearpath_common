# Software License Agreement (BSD)
#
# @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
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

from typing import List

import getopt
import os
import sys

from ament_index_python.packages import get_package_share_directory

from clearpath_config.parser import ClearpathConfigParser


class Package():
    def __init__(self,
                 name: str
                 ) -> None:
        self.name = name
        self.declaration = 'pkg_' + name

    def get_name(self) -> str:
        return self.name

    def find_package_share(self) -> str:
        return '{0} = FindPackageShare(\'{1}\')'.format(self.declaration, self.name)


class LaunchFile():
    class Node():
        def __init__(self,
                     name: str,
                     package: Package,
                     executable: str,
                     namespace: str = '',
                     parameters: List[dict] | List[str] = [],
                     arguments: List[list] | List[str] = [],
                     remappings: List[tuple] = []) -> None:
            self.name = name
            self.declaration = 'node_' + self.name
            self.package = package
            self.executable = executable
            self.namespace = namespace
            self.parameters = parameters
            self.arguments = arguments
            self.remappings = remappings

    @staticmethod
    def get_static_tf_node(name: str,
                           namespace: str,
                           parent_link: str,
                           child_link: str,
                           use_sim_time: bool = False) -> Node:
        return LaunchFile.Node(
            name=name + '_static_tf',
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=namespace,
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '--frame-id', parent_link,
                '--child-frame-id', child_link
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
        )

    class Process():
        def __init__(self,
                     name: str,
                     cmd: List[list] | List[str]) -> None:
            self.name = name
            self.declaration = 'process_' + self.name
            self.cmd = cmd

    class LaunchArg():
        def __init__(self, name: str, default_value: str = '', description: str = '') -> None:
            self.name = name
            self.default_value = default_value
            self.description = description
            self.declaration = 'launch_arg_' + self.name

    class Variable():
        def __init__(self, name: str) -> None:
            self.name = name

    def __init__(self,
                 name: str,
                 path: str = 'launch',
                 package: Package = None,
                 args: List[tuple] = None
                 ) -> None:
        self.package = package
        self.path = path
        self.name = 'launch_' + name
        self.declaration = 'launch_file_{0}'.format(name)
        self.file = '{0}.launch.py'.format(name)
        self.args = args

    def get_full_path(self):
        if self.package:
            return os.path.join(
                get_package_share_directory(self.package.name),
                self.path,
                self.file)
        else:
            return os.path.join(
                self.path,
                self.file)


class ParamFile():
    class Node():
        def __init__(self,
                     name: str,
                     parameters: dict = {},
                     ) -> None:
            self.name = name
            self.parameters = parameters

        def get_name(self) -> str:
            return self.name

        def set_name(self, name) -> None:
            self.name = name

        def get_parameters(self) -> dict:
            return self.parameters

        def set_parameters(self, parameters: dict):
            self.parameters = parameters

    def __init__(self,
                 name: str,
                 namespace: str = '',
                 path: str = 'config',
                 package: Package = None
                 ) -> None:
        self.package = package
        self.path = path
        self.namespace = namespace
        self.name = 'param_file_{0}'.format(name)
        self.file = '{0}.yaml'.format(name)
        self.nodes: List[ParamFile.Node] = []

    def get_full_path(self):
        if self.package:
            return os.path.join(
                get_package_share_directory(self.package.name), self.path, self.file)
        else:
            return os.path.join(self.path, self.file)

    def add_node(self, name: str, parameters: dict) -> None:
        self.nodes.append(ParamFile.Node(name, parameters))

    def read(self) -> None:
        file_contents = ClearpathConfigParser.read_yaml(
            self.get_full_path())

        for node in file_contents:
            self.add_node(node, file_contents[node]['ros__parameters'])


class BashFile():
    def __init__(self,
                 name: str,
                 path: str,
                 package: Package = None,
                 ) -> None:
        self.package = package
        self.path = path
        self.name = name
        self.file = '{0}.bash'.format(name)

    def get_full_path(self):
        if self.package:
            return os.path.join(
                get_package_share_directory(self.package.name), self.path, self.file)
        else:
            return os.path.join(self.path, self.file)


class BaseGenerator():
    SENSORS_PATH = 'sensors/'
    PLATFORM_PATH = 'platform/'
    LAUNCH_PATH = 'launch/'
    PARAM_PATH = 'config/'

    def __init__(self,
                 setup_path: str = '/etc/clearpath/') -> None:
        # Define paths
        self.config_path = os.path.join(setup_path, 'robot.yaml')
        assert os.path.exists(self.config_path)

        self.setup_path = setup_path
        self.sensors_params_path = os.path.join(
            self.setup_path, self.SENSORS_PATH, self.PARAM_PATH)
        self.sensors_launch_path = os.path.join(
            self.setup_path, self.SENSORS_PATH, self.LAUNCH_PATH)
        self.platform_params_path = os.path.join(
            self.setup_path, self.PLATFORM_PATH, self.PARAM_PATH)
        self.platform_launch_path = os.path.join(
            self.setup_path, self.PLATFORM_PATH, self.LAUNCH_PATH)

        # Packages
        self.pkg_clearpath_platform = Package('clearpath_platform')
        self.pkg_clearpath_sensors = Package('clearpath_sensors')
        self.pkg_clearpath_platform_description = Package('clearpath_platform_description')
        self.pkg_clearpath_sensors_description = Package('clearpath_sensors_description')

        # Read YAML
        self.config = ClearpathConfigParser.read_yaml(self.config_path)
        # Parse YAML into config
        self.clearpath_config = ClearpathConfigParser(self.config)

        self.serial_number = self.clearpath_config.platform.get_serial_number()
        self.platform_model = self.clearpath_config.platform.get_model()
        self.namespace = self.clearpath_config.system.get_namespace()

    # This method should be overwritten by the child class
    def generate(self) -> None:
        raise NotImplementedError()

    @staticmethod
    def get_args():
        # Get options
        try:
            last_arg_index = sys.argv.index('--ros-args')
        except ValueError:
            last_arg_index = len(sys.argv)
        argv = sys.argv[1:last_arg_index]

        try:
            options, args = getopt.getopt(argv, 's:', ['setup_path='])
        except getopt.GetoptError as err:
            print(err)

        setup_path = '/etc/clearpath/'

        for option, value in options:
            if option in ('-s', '--setup_path'):
                setup_path = value
            else:
                pass

        return setup_path
