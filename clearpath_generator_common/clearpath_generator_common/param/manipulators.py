# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.
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

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of Clearpath Robotics.
import os

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_generator_common.common import MoveItParamFile, Package
from clearpath_generator_common.param.writer import ParamWriter


class ManipulatorParam():

    MOVEIT = 'moveit'

    class BaseParam():
        CLEARPATH_MOVEIT = 'clearpath_moveit'
        CLEARPATH_MANIPULATORS = 'clearpath_manipulators_description'

        def __init__(self,
                     parameter: str,
                     clearpath_config: ClearpathConfig,
                     param_path: str) -> None:
            self.parameter = parameter
            self.clearpath_config = clearpath_config
            self.namespace = self.clearpath_config.system.namespace
            self.param_path = param_path
            self.param_file = None

            # Clearpath Manipulators Package
            self.clearpath_moveit_package = Package(self.CLEARPATH_MOVEIT)
            self.clearpath_manipulators = Package(self.CLEARPATH_MANIPULATORS)

    class MoveItParam(BaseParam):
        def __init__(self,
                     parameter: str,
                     clearpath_config: ClearpathConfig,
                     param_path: str) -> None:
            super().__init__(parameter, clearpath_config, param_path)
            # Default parameter directory
            self.default_parameter_directory = 'config/planning'
            self.default_parameter_package = self.clearpath_moveit_package

        def generate_parameters(self, use_sim_time: bool = False) -> None:
            parameter_file = MoveItParamFile(
                name='moveit',
                node='move_group',
                namespace=self.namespace,
                path=self.param_path,
            )
            parameter_file += self.get_trajectory_exection_parameters()
            parameter_file += self.get_planning_pipeline_parameters()
            parameter_file += self.get_planning_scene_parameters()
            parameter_file += self.get_kinematics_parameters()
            parameter_file += self.get_moveit_controller_parameters()
            parameter_file += self.get_joint_limits_parameters()

            parameter_file.add_node_header()

            # Get extra ros parameters from config
            extras = self.clearpath_config.platform.extras.ros_parameters
            for node in extras:
                if node in parameter_file.parameters:
                    parameter_file.update({node: extras.get(node)})

            if use_sim_time:
                for node in parameter_file.parameters:
                    parameter_file.update({node: {'use_sim_time': True}})

            self.param_file = parameter_file

        def generate_parameter_file(self):
            param_writer = ParamWriter(self.param_file)
            param_writer.write_file()
            print(f'Generated config: {self.param_file.full_path}')

        # Planning Pipeline
        def get_planning_pipeline_parameters(self):
            parameter_directory = 'config/planning'
            parameter_package = self.clearpath_moveit_package
            parameter_name = 'pipeline'

            parameter_file = MoveItParamFile(
                name=parameter_name,
                path=parameter_directory,
                package=parameter_package,
            )
            parameter_file.read()

            # Add All Plugins
            for subdirectory in os.listdir(parameter_file.directory()):
                subdirectory_path = os.path.join(parameter_file.directory(), subdirectory)
                if os.path.isfile(subdirectory_path):
                    continue
                plugin_file = MoveItParamFile(
                    name=subdirectory,
                    path=subdirectory_path,
                )
                for file in os.listdir(subdirectory_path):
                    plugin_parameter = MoveItParamFile(
                        name=os.path.splitext(file)[0],
                        path=subdirectory_path
                    )
                    plugin_parameter.read()
                    plugin_file += plugin_parameter
                plugin_file.add_header(subdirectory)
                parameter_file += plugin_file

            return parameter_file

        # Planning Scene
        def get_planning_scene_parameters(self):
            parameter_directory = 'config'
            parameter_package = self.clearpath_moveit_package
            parameter_name = 'planning_scene'

            parameter_file = MoveItParamFile(
                name=parameter_name,
                path=parameter_directory,
                package=parameter_package,
            )
            parameter_file.read()
            return parameter_file

        # Kinematics
        def get_kinematics_parameters(self):
            parameter_directory = 'config/kinematics'
            parameter_package = self.clearpath_moveit_package
            parameter_file = MoveItParamFile(
                name='kinematics',
                path=parameter_directory,
                package=parameter_package
            )
            for manipulator in self.clearpath_config.manipulators.get_all_manipulators():
                kinematics_file = MoveItParamFile(
                    name=manipulator.get_manipulator_type(),
                    path=parameter_directory,
                    package=parameter_package
                )
                kinematics_file.read()
                kinematics_file.replace({
                    r'${name}': manipulator.name
                })
                parameter_file += kinematics_file
            return parameter_file

        # Trajectory Execution
        def get_trajectory_exection_parameters(self):
            parameter_directory = 'config'
            parameter_package = self.clearpath_moveit_package
            parameter_name = 'trajectory_execution'

            parameter_file = MoveItParamFile(
                name=parameter_name,
                path=parameter_directory,
                package=parameter_package,
            )
            parameter_file.read()
            return parameter_file

        # MoveIt Controllers
        def get_moveit_controller_parameters(self):
            parameter_directory = 'config'
            parameter_package = self.clearpath_manipulators
            parameter_name = 'moveit_controllers'
            parameter_file = MoveItParamFile(
                name=parameter_name,
                path=parameter_directory,
                package=parameter_package,
            )
            for manipulator in self.clearpath_config.manipulators.get_all_manipulators():
                controller_file = MoveItParamFile(
                    name=parameter_name,
                    path=os.path.join(
                        parameter_directory,
                        manipulator.get_manipulator_type(),
                        manipulator.get_manipulator_model()
                    ),
                    package=parameter_package
                )
                controller_file.read()
                controller_file.replace({
                    r'${name}': manipulator.name
                })
                parameter_file += controller_file
            return parameter_file

        # Joint Limits
        def get_joint_limits_parameters(self):
            parameter_directory = 'config'
            parameter_package = self.clearpath_manipulators
            parameter_name = 'joint_limits'
            parameter_file = MoveItParamFile(
                name=parameter_name,
                path=parameter_directory,
                package=parameter_package,
            )
            for manipulator in self.clearpath_config.manipulators.get_all_manipulators():
                controller_file = MoveItParamFile(
                    name=parameter_name,
                    path=os.path.join(
                        parameter_directory,
                        manipulator.get_manipulator_type(),
                        manipulator.get_manipulator_model()
                    ),
                    package=parameter_package
                )
                controller_file.read()
                controller_file.replace({
                    r'${name}': manipulator.name
                })
                parameter_file += controller_file
            return parameter_file

    PARAMETER = {
        MOVEIT: MoveItParam
    }

    def __new__(cls,
                parameter: str,
                clearpath_config: ClearpathConfig,
                param_path: str) -> BaseParam:
        return ManipulatorParam.PARAMETER.setdefault(parameter, None)(
            parameter, clearpath_config, param_path)
