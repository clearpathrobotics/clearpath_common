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
import os

from typing import List

from clearpath_generator_common.common import LaunchFile, Package


class LaunchWriter():
    tab = '    '

    def __init__(self, launch_file: LaunchFile):
        self.launch_file = launch_file
        self.actions: List[str] = []
        self.included_packages: List[Package] = []
        self.included_launch_files: List[LaunchFile] = []
        self.nodes: List[LaunchFile.Node] = []
        self.declared_launch_args: List[LaunchFile.LaunchArg] = []
        self.processes: List[LaunchFile.Process] = []
        self.file = open(self.launch_file.get_full_path(), 'w+')

    def write(self, string, indent_level=1):
        self.file.write('{0}{1}\n'.format(self.tab * indent_level, string))

    def write_comment(self, comment, indent_level=1):
        self.write('# {0}'.format(comment), indent_level)

    def write_newline(self):
        self.write('', 0)

    def write_actions(self):
        self.write('ld = LaunchDescription()')
        for action in self.actions:
            self.write('ld.add_action({0})'.format(action))
        self.write('return ld')

    def write_string(self, string: str, indent_level=1):
        self.write("'{0}'".format(string), indent_level)

    def write_boolean(self, boolean: bool, indent_level=1):
        self.write(boolean, indent_level)

    def write_integer(self, integer: int, indent_level=1):
        self.write(integer, indent_level)

    def write_variable(self, variable: LaunchFile.Variable, indent_level=1):
        self.write(variable.name, indent_level)

    def write_obj(self, obj: object, indent_level=1):
        if isinstance(obj, str):
            self.write_string(obj, indent_level)
        elif isinstance(obj, bool):
            self.write_boolean(obj, indent_level)
        elif isinstance(obj, int):
            self.write_integer(obj, indent_level)
        elif isinstance(obj, LaunchFile.Variable):
            self.write_variable(obj, indent_level)
        elif isinstance(obj, dict):
            self.write_dictionary(obj, indent_level)
        elif isinstance(obj, list):
            self.write_list(obj, indent_level)
        elif isinstance(obj, tuple):
            self.write_tuple(obj, indent_level)

    def write_key_value_pair(self, key: str, value, indent_level=1):
        if isinstance(value, str):
            self.write("'{0}': '{1}'".format(key, value), indent_level)
        else:
            self.write("'{0}': {1}".format(key, value), indent_level)

    def write_dictionary(self, dictionary: dict, indent_level=1):
        self.write('{', indent_level)
        for k in dictionary.keys():
            # Write Key-Value pair
            self.write_key_value_pair(k, dictionary[k], indent_level + 1)
            self.write(',', indent_level + 1)
        self.write('}', indent_level)

    def write_list(self, _list: list, indent_level=1):
        self.write('[', indent_level)
        for i in _list:
            self.write_obj(i, indent_level + 1)
            self.write(',', indent_level + 1)
        self.write(']', indent_level)

    def write_tuple(self, _tuple: tuple, indent_level=1):
        self.write('(', indent_level)
        self.write_obj(_tuple[0], indent_level + 1)
        self.write(',', indent_level + 1)
        self.write_obj(_tuple[1], indent_level + 1)
        self.write(')', indent_level)

    def find_package(self, package: Package):
        if package not in self.included_packages:
            self.included_packages.append(package)

    def path_join_substitution(package, folder, file):
        return "PathJoinSubstitution([{0}, '{1}', '{2}'])".format(package, folder, file)

    def add_launch_arg(self, launch_arg: LaunchFile.LaunchArg):
        if launch_arg not in self.declared_launch_args:
            # Add launch arg to launch description actions
            self.actions.append(launch_arg.declaration)
            self.declared_launch_args.append(launch_arg)

    def add_launch_file(self, launch_file: LaunchFile):
        if launch_file not in self.included_launch_files:
            self.included_launch_files.append(launch_file)

    def add_node(self, node: LaunchFile.Node):
        if node not in self.nodes:
            self.nodes.append(node)

    def add_process(self, process: LaunchFile.Process):
        if process not in self.processes:
            self.processes.append(process)

    def add(self, component: LaunchFile | LaunchFile.LaunchComponent):
        if isinstance(component, LaunchFile.LaunchArg):
            self.add_launch_arg(component)
        elif isinstance(component, LaunchFile):
            self.add_launch_file(component)
        elif isinstance(component, LaunchFile.Node):
            self.add_node(component)
        elif isinstance(component, LaunchFile.Process):
            self.add_process(component)

    def initialize_file(self):
        self.write(
            'from launch import LaunchDescription', 0)
        self.write(
            'from launch.actions import {0}'.format(
              'IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess'),
            0)
        self.write(
            'from launch.launch_description_sources import PythonLaunchDescriptionSource', 0)
        self.write(
            'from launch.substitutions import {0}'.format(
              'EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration'),
            0)
        self.write(
            'from launch_ros.actions import Node', 0)
        self.write(
            'from launch_ros.substitutions import FindPackageShare', 0)
        self.write_newline()
        self.write_newline()
        self.write(
            'def generate_launch_description():', 0)
        self.write_newline()

    def close_file(self):
        self.file.close()

    def generate_file(self):
        self.initialize_file()

        if len(self.declared_launch_args) > 0:
            for arg in self.declared_launch_args:
                # Declare launch arg
                self.write('{0} = DeclareLaunchArgument('.format(arg.declaration))
                self.write("'{0}',".format(arg.name), indent_level=2)
                self.write("default_value='{0}',".format(arg.default_value), indent_level=2)
                self.write("description='{0}')".format(arg.description), indent_level=2)
                self.write_newline()

                # Launch configuration
                self.write("{0} = LaunchConfiguration('{0}')".format(arg.name))
                self.write_newline()

        if len(self.included_launch_files) > 0:
            # Include packages
            self.write_comment('Include Packages')
            for launch_file in self.included_launch_files:
                if launch_file.package:
                    self.write(launch_file.package.find_package_share())
            self.write_newline()
            # Declare launch files
            self.write_comment('Declare launch files')
            for launch_file in self.included_launch_files:
                if launch_file.package is None:
                    self.write("{0} = '{1}'".format(
                        launch_file.declaration,
                        os.path.join(launch_file.path, launch_file.file)))
                else:
                    self.write('{0} = PathJoinSubstitution(['.format(launch_file.declaration))
                    self.write("{0}, '{1}', '{2}'])".format(
                        launch_file.package.declaration,
                        launch_file.path,
                        launch_file.file), indent_level=2)
            self.write_newline()

            # Include launch files
            self.write_comment('Include launch files')
            for launch_file in self.included_launch_files:
                self.write(
                    '{0} = IncludeLaunchDescription('.format(
                      launch_file.name))
                self.write(
                    'PythonLaunchDescriptionSource([{0}]),'.format(
                      launch_file.declaration), indent_level=2)
                if launch_file.args is not None:
                    self.write('launch_arguments=', indent_level=2)
                    self.write_obj(launch_file.args, indent_level=3)
                self.write(')')
                self.write_newline()

        if len(self.nodes) > 0:
            self.write_comment('Nodes')
            for node in self.nodes:
                self.write('{0} = Node('.format(node.declaration))
                self.write("name='{0}',".format(node.name), indent_level=2)
                self.write("executable='{0}',".format(node.executable), indent_level=2)
                self.write("package='{0}',".format(node.package), indent_level=2)
                self.write("namespace='{0}',".format(node.namespace), indent_level=2)
                self.write("output='screen',", indent_level=2)
                # Arguments
                if len(node.arguments) > 0:
                    self.write('arguments=', indent_level=2)
                    self.write_obj(node.arguments, indent_level=3)
                    self.write(',', indent_level=2)
                # Remappings
                if len(node.remappings) > 0:
                    self.write('remappings=', indent_level=2)
                    self.write_obj(node.remappings, indent_level=3)
                    self.write(',', indent_level=2)
                # Parameters
                if len(node.parameters) > 0:
                    self.write('parameters=', indent_level=2)
                    self.write_obj(node.parameters, indent_level=3)
                    self.write(',', indent_level=2)
                self.write(')')
                self.write_newline()

        if len(self.processes) > 0:
            self.write_comment('Processes')
            for process in self.processes:
                self.write('{0} = ExecuteProcess('.format(process.declaration))
                self.write('shell=True,', indent_level=2)
                self.write('cmd=', indent_level=2)
                self.write_obj(process.cmd, indent_level=3)
                self.write(')')
                self.write_newline()

        # Create LaunchDescription
        self.write_comment('Create LaunchDescription')
        self.write('ld = LaunchDescription()')
        for launch_arg in self.declared_launch_args:
            self.write('ld.add_action({0})'.format(launch_arg.declaration))
        for launch_file in self.included_launch_files:
            self.write('ld.add_action({0})'.format(launch_file.name))
        for node in self.nodes:
            self.write('ld.add_action({0})'.format(node.declaration))
        for process in self.processes:
            self.write('ld.add_action({0})'.format(process.declaration))
        self.write('return ld')

        self.close_file()
        print('Generated launch file: {0}'.format(self.launch_file.get_full_path()))
