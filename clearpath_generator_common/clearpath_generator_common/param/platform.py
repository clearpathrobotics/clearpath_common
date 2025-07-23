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

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of Clearpath Robotics.
import os

from apt import Cache

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.types.platform import Platform
from clearpath_config.common.utils.dictionary import merge_dict, replace_dict_items
from clearpath_config.platform.battery import BatteryConfig
from clearpath_config.sensors.types.cameras import BaseCamera, IntelRealsense
from clearpath_config.sensors.types.gps import BaseGPS
from clearpath_config.sensors.types.imu import BaseIMU, PhidgetsSpatial
from clearpath_config.sensors.types.lidars_2d import BaseLidar2D
from clearpath_config.sensors.types.lidars_3d import BaseLidar3D
from clearpath_config.sensors.types.sensor import BaseSensor
from clearpath_generator_common.common import Package, ParamFile
from clearpath_generator_common.param.writer import ParamWriter
from clearpath_generator_common.ros import ROS_DISTRO


class PlatformParam():
    CONTROL = 'control'
    DIAGNOSTIC_AGGREGATOR = 'diagnostic_aggregator'
    DIAGNOSTIC_UPDATER = 'diagnostic_updater'
    FOXGLOVE_BRIDGE = 'foxglove_bridge'
    IMU_FILTER = 'imu_filter'
    LOCALIZATION = 'localization'
    TELEOP_INTERACTIVE_MARKERS = 'teleop_interactive_markers'
    TELEOP_JOY = 'teleop_joy'
    TWIST_MUX = 'twist_mux'

    NOT_APPLICABLE = 'not_applicable'

    PARAMETERS = [
      CONTROL,
      DIAGNOSTIC_AGGREGATOR,
      DIAGNOSTIC_UPDATER,
      FOXGLOVE_BRIDGE,
      IMU_FILTER,
      LOCALIZATION,
      TELEOP_INTERACTIVE_MARKERS,
      TELEOP_JOY,
      TWIST_MUX
    ]

    class BaseParam():
        CLEARPATH_CONTROL = 'clearpath_control'
        CLEARPATH_DIAGNOSTICS = 'clearpath_diagnostics'

        def __init__(self,
                     parameter: str,
                     clearpath_config: ClearpathConfig,
                     param_path: str) -> None:
            self.parameter = parameter
            self.clearpath_config = clearpath_config
            self.platform = self.clearpath_config.platform.get_platform_model()
            self.namespace = self.clearpath_config.system.namespace
            self.param_path = param_path

            # Clearpath Platform Package
            self.clearpath_control_package = Package(self.CLEARPATH_CONTROL)

            # Default parameter file
            self.default_parameter_file_path = f'config/{self.platform}'
            self.default_parameter_file_package = self.clearpath_control_package
            self.default_parameter = self.parameter

            # Generic Control
            if self.platform == Platform.GENERIC and self.parameter == PlatformParam.CONTROL:
                control = self.clearpath_config.platform.control
                self.default_parameter = os.path.basename(control['path'])
                self.default_parameter = os.path.splitext(self.default_parameter)[0]
                self.default_parameter_file_path = os.path.dirname(control['path'])
                self.default_parameter_file_package = Package(control['package'])

            # Parameter file to generate
            self.param_file = ParamFile(
                name=self.parameter,
                namespace=self.namespace,
                path=self.param_path)

        def generate_parameters(self, use_sim_time: bool = False) -> None:
            # Default parameter file
            self.default_param_file = ParamFile(
                name=self.default_parameter,
                package=self.default_parameter_file_package,
                path=self.default_parameter_file_path,
                parameters={}
            )
            self.default_param_file.read()

            self.param_file.parameters = self.default_param_file.parameters

            # Arm Control
            if self.parameter == PlatformParam.CONTROL and use_sim_time:
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
                    self.param_file.parameters = merge_dict(
                        self.param_file.parameters, updated_parameters)

            # Gripper Control
            if self.parameter == PlatformParam.CONTROL and use_sim_time:
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

            # Lift Control
            if self.parameter == PlatformParam.CONTROL and use_sim_time:
                for lift in self.clearpath_config.manipulators.get_all_lifts():
                    # Arm Control Parameter File
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

            # Get extra ros parameters from config
            extras = self.clearpath_config.platform.extras.ros_parameters
            for node in extras:
                if node in self.param_file.parameters:
                    self.param_file.update({node: extras.get(node)})

            if use_sim_time:
                for node in self.param_file.parameters:
                    self.param_file.update({node: {'use_sim_time': True}})

        def generate_parameter_file(self):
            param_writer = ParamWriter(self.param_file)
            param_writer.write_file()
            print(f'Generated config: {self.param_file.full_path}')

    class ImuFilterParam(BaseParam):
        def __init__(self,
                     parameter: str,
                     clearpath_config: ClearpathConfig,
                     param_path: str) -> None:
            super().__init__(parameter, clearpath_config, param_path)
            self.default_parameter_file_package = self.clearpath_control_package
            self.default_parameter_file_path = 'config'

    class DiagnosticsAggregatorParam(BaseParam):
        """Parameter file that decides the aggregation of the diagnostics data for display."""

        DIAGNOSTIC_AGGREGATOR_NODE = 'diagnostic_aggregator'

        def __init__(self,
                     parameter: str,
                     clearpath_config: ClearpathConfig,
                     param_path: str) -> None:
            super().__init__(parameter, clearpath_config, param_path)
            self.default_parameter_file_package = Package(self.CLEARPATH_DIAGNOSTICS)
            self.default_parameter_file_path = 'config'

        def generate_parameters(self, use_sim_time: bool = False) -> None:
            super().generate_parameters(use_sim_time)

            # Add MCU diagnostic category for all platforms except A200
            if self.clearpath_config.get_platform_model() != Platform.A200:
                self.param_file.update({
                    self.DIAGNOSTIC_AGGREGATOR_NODE: {
                        'platform': {
                            'analyzers': {
                                'mcu': {
                                    'type': 'diagnostic_aggregator/GenericAnalyzer',
                                    'path': 'MCU',
                                    'expected': [
                                        'clearpath_diagnostic_updater: MCU Firmware Version',
                                        'clearpath_diagnostic_updater: MCU Status'
                                    ],
                                    'contains': ['MCU']
                                }
                            }
                        }
                    }
                })

            # Add Lighting for every platform except A200 and J100
            if self.clearpath_config.get_platform_model() not in (Platform.A200, Platform.J100):
                self.param_file.update({
                    self.DIAGNOSTIC_AGGREGATOR_NODE: {
                        'platform': {
                            'analyzers': {
                                'lighting': {
                                    'type': 'diagnostic_aggregator/GenericAnalyzer',
                                    'path': 'Lighting',
                                    'expected': [
                                        'lighting_node: Light Status'
                                    ],
                                    'contains': ['Light']
                                }
                            }
                        }
                    }
                })

            # Add cooling for A300 only for now
            if self.clearpath_config.get_platform_model() == Platform.A300:
                self.param_file.update({
                    self.DIAGNOSTIC_AGGREGATOR_NODE: {
                        'platform': {
                            'analyzers': {
                                'cooling': {
                                    'type': 'diagnostic_aggregator/GenericAnalyzer',
                                    'path': 'Cooling',
                                    'contains': ['Fan', 'Thermal']
                                }
                            }
                        }
                    }
                })

            if self.clearpath_config.platform.enable_ekf:
                self.param_file.update({
                    self.DIAGNOSTIC_AGGREGATOR_NODE: {
                        'platform': {
                            'analyzers': {
                                'odometry': {
                                    'type': 'diagnostic_aggregator/GenericAnalyzer',
                                    'path': 'Odometry',
                                    'contains': ['odometry', 'ekf_node'],
                                    'expected': [
                                        'ekf_node: Filter diagnostic updater',
                                        'ekf_node: odometry/filtered topic status',
                                    ]
                                }
                            }
                        }
                    }
                })

            if self.clearpath_config.platform.enable_wireless_watcher:
                self.param_file.update({
                    self.DIAGNOSTIC_AGGREGATOR_NODE: {
                        'platform': {
                            'analyzers': {
                                'networking': {
                                    'type': 'diagnostic_aggregator/GenericAnalyzer',
                                    'path': 'Networking',
                                    'contains': ['Wi-Fi'],
                                    'expected': ['wireless_watcher: Wi-Fi Monitor']
                                }
                            }
                        }
                    }
                })

            sensor_analyzers = {}

            # List all topics to be monitored from each launched sensor
            for sensor in self.clearpath_config.sensors.get_all_sensors():

                if not sensor.launch_enabled:
                    continue

                match sensor:
                    case BaseCamera():
                        sensor_analyzers['cameras'] = {
                            'type': 'diagnostic_aggregator/GenericAnalyzer',
                            'path': 'Cameras',
                            'contains': ['camera']
                        }
                    case BaseLidar2D():
                        sensor_analyzers['lidar2d'] = {
                            'type': 'diagnostic_aggregator/GenericAnalyzer',
                            'path': 'Lidar2D',
                            'contains': ['lidar2d']
                        }
                    case BaseLidar3D():
                        sensor_analyzers['lidar3d'] = {
                            'type': 'diagnostic_aggregator/GenericAnalyzer',
                            'path': 'Lidar3D',
                            'contains': ['lidar3d']
                        }
                    case BaseIMU():
                        sensor_analyzers['imu'] = {
                            'type': 'diagnostic_aggregator/GenericAnalyzer',
                            'path': 'IMU',
                            'contains': ['imu']
                        }
                    case BaseGPS():
                        sensor_analyzers['gps'] = {
                            'type': 'diagnostic_aggregator/GenericAnalyzer',
                            'path': 'GPS',
                            'contains': ['gps']
                        }

            # Update aggregator sensor sections based on the robot.yaml
            if sensor_analyzers:
                self.param_file.update({
                    self.DIAGNOSTIC_AGGREGATOR_NODE: {
                        'sensors': {
                            'type': 'diagnostic_aggregator/AnalyzerGroup',
                            'path': 'Sensors',
                            'analyzers': sensor_analyzers
                        }
                    }
                })

    class DiagnosticsUpdaterParam(BaseParam):
        """Parameter file for Clearpath Diagnostics indicating which topics to monitor."""

        DIAGNOSTIC_UPDATER_NODE = 'clearpath_diagnostic_updater'

        def __init__(self,
                     parameter: str,
                     clearpath_config: ClearpathConfig,
                     param_path: str) -> None:
            super().__init__(parameter, clearpath_config, param_path)
            self.default_parameter_file_package = Package(self.CLEARPATH_DIAGNOSTICS)
            self.default_parameter_file_path = 'config'
            self.diag_dict = {}

        def generate_parameters(self, use_sim_time: bool = False) -> None:
            super().generate_parameters(use_sim_time)

            # Update parameters based on the robot.yaml
            platform_model = self.clearpath_config.get_platform_model()
            self.param_file.update({
                self.DIAGNOSTIC_UPDATER_NODE: {
                    'serial_number': self.clearpath_config.get_serial_number(),
                    'platform_model': platform_model
                }
            })

            if use_sim_time:
                latest_apt_firmware_version = 'simulated'
                installed_apt_firmware_version = 'simulated'
            elif platform_model == Platform.A200:
                latest_apt_firmware_version = PlatformParam.NOT_APPLICABLE
                installed_apt_firmware_version = PlatformParam.NOT_APPLICABLE
            else:
                # Check latest firmware version available and save it in the config
                cache = Cache()
                latest_apt_firmware_version = 'not_found'
                installed_apt_firmware_version = 'none'
                try:
                    pkg = cache[f'ros-{ROS_DISTRO}-clearpath-firmware']
                    latest_apt_firmware_version = pkg.versions[0].version.split('-')[0]
                    if (pkg.is_installed):
                        installed_apt_firmware_version = pkg.installed.version.split('-')[0]
                except KeyError:
                    print(f'\033[93mWarning: ros-{ROS_DISTRO}-clearpath-firmware'
                          ' package not found\033[0m')

            # Set expected BMS rate based on the platform battery model
            bms_state_rate = 10.0
            bms_state_tolerance = 0.15
            if (self.clearpath_config.platform.battery.model in [BatteryConfig.S_24V20_U1]):
                bms_state_rate = 2.5
                bms_state_tolerance = 0.2
            elif (self.clearpath_config.platform.battery.model in
                    [BatteryConfig.VALENCE_U24_12XP, BatteryConfig.VALENCE_U27_12XP]):
                bms_state_rate = 3.0
                bms_state_tolerance = 0.25
            elif (self.clearpath_config.platform.battery.model in [BatteryConfig.NEC_ALM12V35]):
                bms_state_rate = 1.0
                bms_state_tolerance = 0.25
            elif (platform_model == Platform.A200):
                bms_state_rate = 1.8
                bms_state_tolerance = 0.25

            self.param_file.update({
                self.DIAGNOSTIC_UPDATER_NODE: {
                    'ros_distro': ROS_DISTRO,
                    'latest_apt_firmware_version': latest_apt_firmware_version,
                    'installed_apt_firmware_version': installed_apt_firmware_version,
                    'bms_state_rate': bms_state_rate,
                    'bms_state_tolerance': bms_state_tolerance
                }
            })

            # Additional considerations for A200 platform
            if platform_model == Platform.A200:
                self.param_file.update({
                    self.DIAGNOSTIC_UPDATER_NODE: {
                        'stop_status_rate': 0.0,  # Disable stop status diagnostic for A200
                        'mcu_power_rate': 1.8,
                        'mcu_power_tolerance': 0.25,
                        'estop_rate': 1.8,
                        'estop_tolerance': 0.25
                    }
                })
            elif platform_model == Platform.W200:
                self.param_file.update({
                    self.DIAGNOSTIC_UPDATER_NODE: {
                        'stop_status_rate': 0.0,  # Disable stop status diagnostic for W200
                    }
                })

            # List all topics to be monitored from each launched sensor
            for sensor in self.clearpath_config.sensors.get_all_sensors():

                if not sensor.launch_enabled:
                    continue

                match sensor:
                    case IntelRealsense():
                        if sensor.color_enabled:
                            self.add_topic(sensor, sensor.TOPICS.COLOR_IMAGE)
                        if sensor.depth_enabled:
                            self.add_topic(sensor, sensor.TOPICS.DEPTH_IMAGE)
                        if sensor.pointcloud_enabled:
                            self.add_topic(sensor, sensor.TOPICS.POINTCLOUD)

                    case BaseCamera():
                        self.add_topic(sensor, sensor.TOPICS.COLOR_IMAGE)

                    case BaseLidar2D():
                        self.add_topic(sensor, sensor.TOPICS.SCAN)

                    case BaseLidar3D():
                        self.add_topic(sensor, sensor.TOPICS.SCAN)
                        self.add_topic(sensor, sensor.TOPICS.POINTS)

                    case PhidgetsSpatial():
                        self.add_topic(sensor, sensor.TOPICS.DATA),
                        self.add_topic(sensor, sensor.TOPICS.RAW_DATA),
                        self.add_topic(sensor, sensor.TOPICS.MAG),

                    case BaseIMU():
                        self.add_topic(sensor, sensor.TOPICS.DATA)
                        self.add_topic(sensor, sensor.TOPICS.MAG)

                    case BaseGPS():
                        self.add_topic(sensor, sensor.TOPICS.FIX)

            # Output the list of topics into the parameter file
            self.param_file.update({self.DIAGNOSTIC_UPDATER_NODE: {'topics': self.diag_dict}})

        def add_topic(self, sensor: BaseSensor, topic_key: str) -> None:
            """
            Add a sensor topic to the dictionary using the topic key string.

            :param sensor: The sensor object from which the topic info will be gotten
            :param topic_key: The key used to identify the topic to be monitored
            """
            self.diag_dict[sensor.get_topic_name(topic_key, local=True)] = {
                'type': sensor.get_topic_type(topic_key),
                'rate': float(sensor.get_topic_rate(topic_key))
            }

    class FoxgloveBridgeParam(BaseParam):
        def __init__(self,
                     parameter: str,
                     clearpath_config: ClearpathConfig,
                     param_path: str) -> None:
            super().__init__(parameter, clearpath_config, param_path)
            self.default_parameter_file_package = Package(self.CLEARPATH_DIAGNOSTICS)
            self.default_parameter_file_path = 'config'

    class LocalizationParam(BaseParam):
        EKF_NODE = 'ekf_node'
        imu_config = [False, False, False,
                      False, False, False,
                      False, False, False,
                      False, False, True,
                      True, False, False]

        def generate_parameters(self, use_sim_time: bool = False) -> None:
            super().generate_parameters(use_sim_time)

            extras = self.clearpath_config.platform.extras.ros_parameters.get(self.EKF_NODE)
            if extras:
                self.param_file.update({self.EKF_NODE: extras})
            else:
                # Count the IMU index individually so we can continue counting for GPS
                imu_idx = 0

                if Platform.INDEX[self.platform].imu > 0:
                    imu0_parameters = {
                        'imu0': 'sensors/imu_0/data',
                        'imu0_config': self.imu_config,
                        'imu0_differential': False,
                        'imu0_queue_size': 10,
                        # Gravitational acceleration is removed in IMU driver
                        'imu0_remove_gravitational_acceleration': True
                    }
                    self.param_file.update({self.EKF_NODE: imu0_parameters})
                    imu_idx += 1

                # Add all additional IMU's
                imus = self.clearpath_config.sensors.get_all_imu()
                for imu in imus:
                    if imu.launch_enabled:
                        imu_name = f'imu{imu_idx}'
                        imu_parameters = {
                            imu_name: f'sensors/{imu.name}/data',
                            f'{imu_name}_config': self.imu_config,
                            f'{imu_name}_differential': False,
                            f'{imu_name}_queue_size': 10,
                            f'{imu_name}_remove_gravitational_acceleration': True
                        }
                        self.param_file.update({self.EKF_NODE: imu_parameters})
                        imu_idx += 1

                # Add all GPS sensors that have IMUs
                gpss = self.clearpath_config.sensors.get_all_gps()
                for gps in gpss:
                    if gps.launch_enabled and gps.has_imu():
                        imu_idx += 1
                        gps_name = f'imu{imu_idx}'
                        gps_parameters = {
                            gps_name: f'sensors/{gps.name}/imu/data',
                            f'{gps_name}_config': self.imu_config,
                            f'{gps_name}_differential': False,
                            f'{gps_name}_queue_size': 10,
                            f'{gps_name}_remove_gravitational_acceleration': True
                        }
                        self.param_file.update({self.EKF_NODE: gps_parameters})

    class TeleopJoyParam(BaseParam):
        def __init__(self,
                     parameter: str,
                     clearpath_config: ClearpathConfig,
                     param_path: str) -> None:
            super().__init__(parameter, clearpath_config, param_path)
            self.default_parameter_file_path = f'config/{self.platform}'
            self.default_parameter = f'teleop_{self.clearpath_config.platform.controller}'

    class TwistMuxParam(BaseParam):
        def __init__(self,
                     parameter: str,
                     clearpath_config: ClearpathConfig,
                     param_path: str) -> None:
            super().__init__(parameter, clearpath_config, param_path)
            self.default_parameter_file_path = 'config'

    PARAMETER = {
        IMU_FILTER: ImuFilterParam,
        DIAGNOSTIC_AGGREGATOR: DiagnosticsAggregatorParam,
        DIAGNOSTIC_UPDATER: DiagnosticsUpdaterParam,
        FOXGLOVE_BRIDGE: FoxgloveBridgeParam,
        LOCALIZATION: LocalizationParam,
        TELEOP_JOY: TeleopJoyParam,
        TWIST_MUX: TwistMuxParam,
    }

    def __new__(cls,
                parameter: str,
                clearpath_config: ClearpathConfig,
                param_path: str) -> BaseParam:
        return PlatformParam.PARAMETER.setdefault(parameter, PlatformParam.BaseParam)(
            parameter, clearpath_config, param_path)
