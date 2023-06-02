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
import os

from clearpath_generator_common.common import LaunchFile, Package, ParamFile


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

    def find_package(self, package: Package):
        if package not in self.included_packages:
            self.included_packages.append(package)

    def path_join_substitution(package, folder, file):
        return 'PathJoinSubstitution([{0}, \'{1}\', \'{2}\'])'.format(package, folder, file)

    def declare_launch_arg(self, launch_arg: LaunchFile.LaunchArg):
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

    def initialize_file(self):
        self.write(
            'from launch import LaunchDescription', 0)
        self.write(
            'from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess', 0)
        self.write(
            'from launch.launch_description_sources import PythonLaunchDescriptionSource', 0)
        self.write(
            'from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration', 0)
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
                self.write('\'{0}\','.format(arg.name), indent_level=2)
                self.write('default_value=\'{0}\','.format(arg.default_value), indent_level=2)
                self.write('description=\'{0}\')'.format(arg.description), indent_level=2)
                self.write_newline()

                # Launch configuration
                self.write('{0} = LaunchConfiguration(\'{0}\')'.format(arg.name))
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
                    self.write('{0} = \'{1}\''.format(
                        launch_file.declaration,
                        os.path.join(launch_file.path, launch_file.file)))
                else:
                    self.write('{0} = PathJoinSubstitution(['.format(launch_file.declaration))
                    self.write('{0}, \'{1}\', \'{2}\'])'.format(
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
                    self.write('launch_arguments=[', indent_level=2)
                    for key in launch_file.args.keys():
                        value = launch_file.args.get(key)
                        if isinstance(value, str):
                            self.write(
                              '(\'{0}\', \'{1}\'),'.format(key, value),
                              indent_level=3)
                        elif isinstance(value, dict):
                            self.write(
                              '(\'{0}\', {1}),'.format(key, value),
                              indent_level=3)
                        elif isinstance(value, ParamFile):
                            self.write(
                              '(\'{0}\', \'{1}\'),'.format(key, value.get_full_path()),
                              indent_level=3)
                        elif isinstance(value, bool):
                            self.write(
                              '(\'{0}\', {1}),'.format(key, str(value)),
                              indent_level=3)
                    self.write(']', indent_level=2)
                self.write(')')
                self.write_newline()

        if len(self.nodes) > 0:
            self.write_comment('Nodes')
            for node in self.nodes:
                self.write('{0} = Node('.format(node.declaration))
                self.write('name=\'{0}\','.format(node.name), indent_level=2)
                self.write('executable=\'{0}\','.format(node.executable), indent_level=2)
                self.write('package=\'{0}\','.format(node.package), indent_level=2)
                self.write('output=\'screen\',', indent_level=2)
                if len(node.arguments) > 0:
                    self.write('arguments=[', indent_level=2)
                    for arg in node.arguments:
                        if isinstance(arg, list):
                            self.write('[', indent_level=3)
                            for a in arg:
                                self.write('{0},'.format(a), indent_level=4)
                            self.write('],', indent_level=3)
                        elif isinstance(arg, str):
                            self.write('\'{0}\','.format(arg), indent_level=3)
                        elif isinstance(arg, bool):
                            self.write('{0},'.format(str(arg)), indent_level=3)
                    self.write('],', indent_level=2)
                if len(node.remappings) > 0:
                    self.write('remappings=[', indent_level=2)
                    for remap in node.remappings:
                        self.write('(', indent_level=3)
                        if isinstance(remap[0], list):
                            self.write('[', indent_level=4)
                            for i in remap[0]:
                                self.write('{0},'.format(i), indent_level=5)
                            self.write('],', indent_level=4)
                        elif isinstance(remap[0], str):
                            self.write('{0},'.format(remap[0]), indent_level=4)
                        self.write('{0}'.format(remap[1]), indent_level=4)
                        self.write('),', indent_level=3)
                    self.write('],', indent_level=2)
                if len(node.parameters) > 0:
                    self.write('parameters=[', indent_level=2)
                    for parameter in node.parameters:
                        if isinstance(parameter, dict):
                            self.write('{0},'.format(parameter), indent_level=3)
                        elif isinstance(parameter, str):
                            self.write('{0},'.format(parameter), indent_level=3)
                    self.write('],', indent_level=2)
                self.write(')')
                self.write_newline()

        if len(self.processes) > 0:
            self.write_comment('Processes')
            for process in self.processes:
                self.write('{0} = ExecuteProcess('.format(process.declaration))
                self.write('shell=True,', indent_level=2)
                self.write('cmd=[', indent_level=2)
                if isinstance(process.cmd[0], list):
                    for cmd in process.cmd:
                        self.write('[', indent_level=3)
                        for c in cmd:
                            self.write('{0},'.format(c), indent_level=4)
                        self.write('],', indent_level=3)
                elif isinstance(process.cmd[0], str):
                    for cmd in process.cmd:
                        self.write('\'{0}\','.format(cmd), indent_level=3)
                self.write('],', indent_level=2)
                self.write(')')

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
