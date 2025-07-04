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
from clearpath_config.common.utils.dictionary import merge_dict, replace_dict_items
from clearpath_config.manipulators.types.arms import UniversalRobots
from clearpath_generator_common.common import MoveItParamFile, Package, ParamFile
from clearpath_generator_common.param.writer import ParamWriter


class ManipulatorParam():
    MOVEIT = 'moveit'
    CONTROL = 'control'

    class BaseParam():
        CLEARPATH_MANIPULATORS_DESCRIPTION = 'clearpath_manipulators_description'
        CLEARPATH_MANIPULATORS = 'clearpath_manipulators'

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
            self.clearpath_manipulators_description = Package(
                self.CLEARPATH_MANIPULATORS_DESCRIPTION)
            self.clearpath_manipulators = Package(
                self.CLEARPATH_MANIPULATORS)

    class Control(BaseParam):

        def __init__(self,
                     parameter: str,
                     clearpath_config: ClearpathConfig,
                     param_path: str) -> None:
            super().__init__(parameter, clearpath_config, param_path)
            self.default_parameter_name = 'control'
            self.default_parameter_directory = 'config'
            self.default_parameter_package = self.clearpath_manipulators

        def generate_parameters(self, use_sim_time: bool = False) -> None:
            default_param_file = ParamFile(
                name=self.default_parameter_name,
                package=self.default_parameter_package,
                path=self.default_parameter_directory,
            )
            if self.namespace == '/':
                namespace = ''
            else:
                namespace = self.namespace
            self.param_file = ParamFile(
                name=self.default_parameter_name,
                namespace=f'{namespace}/manipulators',
                path=self.param_path
            )

            default_param_file.read()
            self.param_file.parameters = default_param_file.parameters

            # Arms
            for arm in self.clearpath_config.manipulators.get_all_arms():
                # Arm Control Parameter File
                arm_param_file = ParamFile(
                    name='control',
                    package=Package('clearpath_manipulators_description'),
                    path='config/%s/%s' % (
                        arm.get_manipulator_type(),
                        arm.get_manipulator_model()),
                    parameters={}
                )
                arm_param_file.read()
                updated_parameters = replace_dict_items(
                    arm_param_file.parameters,
                    {r'${name}': arm.name}
                )
                # UR Arm Exception. Update Rate
                if arm.MANIPULATOR_MODEL == UniversalRobots.MANIPULATOR_MODEL:
                    try:
                        # Update Rate from Parameter File
                        update_rate_param_file = ParamFile(
                            name=f'{arm.ur_type}_update_rate',
                            package=Package('ur_robot_driver'),
                            path='config',
                            parameters={}
                        )
                        update_rate_param_file.read()
                        updated_parameters.update(update_rate_param_file.parameters)
                    except Exception as e:
                        print(f'Unable to get UniversalRobots {arm.ur_type}_'
                              f'update_rate.yaml parameter file: {e.args[0]}')
                updated_parameters = replace_dict_items(
                    updated_parameters,
                    {r'${controller_name}': arm.name}
                )
                self.param_file.parameters = merge_dict(
                    updated_parameters, self.param_file.parameters)
            # Grippers
            for arm in self.clearpath_config.manipulators.get_all_arms():
                if not arm.gripper:
                    continue
                gripper = arm.gripper
                # Gripper Control Parameter File
                gripper_param_file = ParamFile(
                    name='control',
                    package=Package('clearpath_manipulators_description'),
                    path='config/%s/%s' % (
                        gripper.get_manipulator_type(),
                        gripper.get_manipulator_model()),
                    parameters={}
                )
                gripper_param_file.read()
                updated_parameters = replace_dict_items(
                    gripper_param_file.parameters,
                    {r'${name}': gripper.name}
                )
                self.param_file.parameters = merge_dict(
                    self.param_file.parameters, updated_parameters)

            # Lifts
            for lift in self.clearpath_config.manipulators.get_all_lifts():
                # Lift Control Parameter File
                lift_param_file = ParamFile(
                    name='control',
                    package=Package('clearpath_manipulators_description'),
                    path='config/%s/%s' % (
                        lift.get_manipulator_type(),
                        lift.get_manipulator_model()),
                    parameters={}
                )
                lift_param_file.read()
                updated_parameters = replace_dict_items(
                    lift_param_file.parameters,
                    {r'${name}': lift.name}
                )
                self.param_file.parameters = merge_dict(
                    self.param_file.parameters, updated_parameters)

        def generate_parameter_file(self):
            param_writer = ParamWriter(self.param_file)
            param_writer.write_file()
            print(f'Generated config: {self.param_file.full_path}')

    class MoveItParam(BaseParam):
        def __init__(self,
                     parameter: str,
                     clearpath_config: ClearpathConfig,
                     param_path: str) -> None:
            super().__init__(parameter, clearpath_config, param_path)
            # Default parameter directory
            self.default_parameter_directory = 'config/planning'
            self.default_parameter_package = self.clearpath_manipulators

        def generate_parameters(self, use_sim_time: bool = False) -> None:
            self.param_file = MoveItParamFile(
                name='moveit',
                node='move_group',
                namespace=self.namespace,
                path=self.param_path,
            )
            self.param_file += self.get_trajectory_exection_parameters()
            self.param_file += self.get_planning_pipeline_parameters()
            self.param_file += self.get_planning_scene_parameters()
            self.param_file += self.get_kinematics_parameters()
            self.param_file += self.get_moveit_controller_parameters(use_sim_time)
            self.param_file += self.get_joint_limits_parameters()
            self.param_file += self.get_cartesian_limits_parameters()

            self.param_file.add_node_header()

            # Get MoveIt ros parameters from config
            moveit_params = self.clearpath_config.manipulators.moveit.ros_parameters
            for node in moveit_params:
                if node in self.param_file.parameters:
                    self.param_file.update({node: moveit_params.get(node)})

            if use_sim_time:
                for node in self.param_file.parameters:
                    self.param_file.update({node: {'use_sim_time': True}})

        def generate_parameter_file(self):
            param_writer = ParamWriter(self.param_file)
            param_writer.write_file()
            print(f'Generated config: {self.param_file.full_path}')

        # Planning Pipeline
        def get_planning_pipeline_parameters(self):
            parameter_directory = 'config/planning'
            parameter_package = self.clearpath_manipulators
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
            parameter_package = self.clearpath_manipulators
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
            parameter_package = self.clearpath_manipulators
            parameter_file = MoveItParamFile(
                name='kinematics',
                path=parameter_directory,
                package=parameter_package
            )
            # Arms
            for arm in self.clearpath_config.manipulators.get_all_arms():
                kinematics_file = MoveItParamFile(
                    name=arm.get_manipulator_type(),
                    path=parameter_directory,
                    package=parameter_package
                )
                kinematics_file.read()
                kinematics_file.replace({
                    r'${name}': arm.name
                })
                parameter_file += kinematics_file
            # Grippers
            for arm in self.clearpath_config.manipulators.get_all_arms():
                if not arm.gripper:
                    continue
                gripper = arm.gripper
                kinematics_file = MoveItParamFile(
                    name=gripper.get_manipulator_type(),
                    path=parameter_directory,
                    package=parameter_package
                )
                kinematics_file.read()
                kinematics_file.replace({
                    r'${name}': gripper.name
                })
                parameter_file += kinematics_file
            # Lifts
            for lift in self.clearpath_config.manipulators.get_all_lifts():
                kinematics_file = MoveItParamFile(
                    name=lift.get_manipulator_type(),
                    path=parameter_directory,
                    package=parameter_package
                )
                kinematics_file.read()
                kinematics_file.replace({
                    r'${name}': lift.name
                })
                parameter_file += kinematics_file
            parameter_file.add_header('robot_description_kinematics')
            return parameter_file

        # Trajectory Execution
        def get_trajectory_exection_parameters(self):
            parameter_directory = 'config'
            parameter_package = self.clearpath_manipulators
            parameter_name = 'trajectory_execution'

            parameter_file = MoveItParamFile(
                name=parameter_name,
                path=parameter_directory,
                package=parameter_package,
            )
            parameter_file.read()
            return parameter_file

        # MoveIt Controllers
        def get_moveit_controller_parameters(self, use_sim_time: bool = False) -> None:
            parameter_directory = 'config'
            parameter_package = self.clearpath_manipulators_description
            parameter_name = 'moveit_controllers'
            parameter_file = MoveItParamFile(
                name=parameter_name,
                path=parameter_directory,
                package=parameter_package,
            )
            # Arms
            for arm in self.clearpath_config.manipulators.get_all_arms():
                controller_file = MoveItParamFile(
                    name=parameter_name,
                    path=os.path.join(
                        parameter_directory,
                        arm.get_manipulator_type(),
                        arm.get_manipulator_model()
                    ),
                    package=parameter_package
                )
                controller_name = arm.name
                if not use_sim_time:
                    controller_name = 'manipulators/' + controller_name
                controller_file.read()
                controller_file.replace({
                    r'${controller_name}': controller_name,
                    r'${name}': arm.name
                })
                parameter_file += controller_file
            # Grippers
            for arm in self.clearpath_config.manipulators.get_all_arms():
                if not arm.gripper:
                    continue
                gripper = arm.gripper
                controller_file = MoveItParamFile(
                    name=parameter_name,
                    path=os.path.join(
                        parameter_directory,
                        gripper.get_manipulator_type(),
                        gripper.get_manipulator_model()
                    ),
                    package=parameter_package
                )
                controller_name = gripper.name
                if not use_sim_time:
                    controller_name = 'manipulators/' + controller_name
                controller_file.read()
                controller_file.replace({
                    r'${controller_name}': controller_name,
                    r'${name}': gripper.name
                })
                parameter_file += controller_file
            # Lifts
            for lift in self.clearpath_config.manipulators.get_all_lifts():
                controller_file = MoveItParamFile(
                    name=parameter_name,
                    path=os.path.join(
                        parameter_directory,
                        lift.get_manipulator_type(),
                        lift.get_manipulator_model()
                    ),
                    package=parameter_package
                )
                controller_name = lift.name
                if not use_sim_time:
                    controller_name = 'manipulators/' + controller_name
                controller_file.read()
                controller_file.replace({
                    r'${controller_name}': controller_name,
                    r'${name}': lift.name
                })
                parameter_file += controller_file
            return parameter_file

        # Joint Limits
        def get_joint_limits_parameters(self):
            parameter_directory = 'config'
            parameter_package = self.clearpath_manipulators_description
            parameter_name = 'joint_limits'
            parameter_file = MoveItParamFile(
                name=parameter_name,
                path=parameter_directory,
                package=parameter_package,
            )
            # Arms
            for arm in self.clearpath_config.manipulators.get_all_arms():
                controller_file = MoveItParamFile(
                    name=parameter_name,
                    path=os.path.join(
                        parameter_directory,
                        arm.get_manipulator_type(),
                        arm.get_manipulator_model()
                    ),
                    package=parameter_package
                )
                controller_file.read()
                controller_file.replace({
                    r'${name}': arm.name
                })
                parameter_file += controller_file
            # Grippers
            for arm in self.clearpath_config.manipulators.get_all_arms():
                if not arm.gripper:
                    continue
                gripper = arm.gripper
                controller_file = MoveItParamFile(
                    name=parameter_name,
                    path=os.path.join(
                        parameter_directory,
                        gripper.get_manipulator_type(),
                        gripper.get_manipulator_model()
                    ),
                    package=parameter_package
                )
                controller_file.read()
                controller_file.replace({
                    r'${name}': gripper.name
                })
                parameter_file += controller_file
            # Lifts
            for lift in self.clearpath_config.manipulators.get_all_lifts():
                controller_file = MoveItParamFile(
                    name=parameter_name,
                    path=os.path.join(
                        parameter_directory,
                        lift.get_manipulator_type(),
                        lift.get_manipulator_model()
                    ),
                    package=parameter_package
                )
                controller_file.read()
                controller_file.replace({
                    r'${name}': lift.name
                })
                parameter_file += controller_file
            parameter_file.add_header('robot_description_planning')
            return parameter_file

        # Cartesian Limits
        def get_cartesian_limits_parameters(self):
            parameter_directory = 'config'
            parameter_package = self.clearpath_manipulators
            parameter_name = 'cartesian_limits'

            parameter_file = MoveItParamFile(
                name=parameter_name,
                path=parameter_directory,
                package=parameter_package,
            )
            parameter_file.read()
            parameter_file.add_header('robot_description_planning')
            return parameter_file

    PARAMETERS = {
        MOVEIT: MoveItParam,
        CONTROL: Control
    }

    def __new__(cls,
                parameter: str,
                clearpath_config: ClearpathConfig,
                param_path: str) -> BaseParam:
        return ManipulatorParam.PARAMETERS.setdefault(parameter, None)(
            parameter, clearpath_config, param_path)
