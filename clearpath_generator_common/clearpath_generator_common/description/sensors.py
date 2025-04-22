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
from typing import List

from clearpath_config.sensors.types.cameras import (
    AxisCamera,
    BaseCamera,
    FlirBlackfly,
    IntelRealsense,
    LuxonisOAKD,
    StereolabsZed
)
from clearpath_config.sensors.types.imu import (
    BaseIMU,
    CHRoboticsUM6,
    Microstrain,
    RedshiftUM7
)
from clearpath_config.sensors.types.ins import (
    BaseINS,
    Fixposition,
)
from clearpath_config.sensors.types.lidars_2d import BaseLidar2D, HokuyoUST, SickLMS1XX
from clearpath_config.sensors.types.lidars_3d import (
    BaseLidar3D,
    OusterOS1,
    SeyondLidar,
    VelodyneLidar,
)
from clearpath_config.sensors.types.sensor import BaseSensor


class SensorDescription():
    class BaseDescription():
        pkg_clearpath_sensors_description = 'clearpath_sensors_description'

        NAME = 'name'
        PARENT = 'parent_link'
        XYZ = 'xyz'
        RPY = 'rpy'

        def __init__(self, sensor: BaseSensor) -> None:
            self.sensor = sensor
            self.package = self.pkg_clearpath_sensors_description
            self.path = 'urdf/'

            self.parameters = {
                self.NAME: sensor.name,
                self.PARENT: sensor.parent
            }

        @property
        def name(self) -> str:
            return self.sensor.name

        @property
        def model(self) -> str:
            return self.sensor.SENSOR_MODEL

        @property
        def xyz(self) -> List[float]:
            return self.sensor.xyz

        @property
        def rpy(self) -> List[float]:
            return self.sensor.rpy

    class Lidar2dDescription(BaseDescription):
        ANGULAR_RESOLUTION = 'ang_res'
        MINIMUM_ANGLE = 'min_ang'
        MAXIMUM_ANGLE = 'max_ang'
        MINIMUM_RANGE = 'min_range'
        MAXIMUM_RANGE = 'max_range'
        UPDATE_RATE = 'update_rate'

        def __init__(self, sensor: BaseLidar2D) -> None:
            super().__init__(sensor)

            self.parameters.update({
                self.ANGULAR_RESOLUTION: 0.5,
                self.MINIMUM_ANGLE: sensor.min_angle,
                self.MAXIMUM_ANGLE: sensor.max_angle,
                self.MINIMUM_RANGE: 0.05,
                self.MAXIMUM_RANGE: 25.0,
                self.UPDATE_RATE: 40  # TODO: link to clearpath_config property
            })

    class Lidar3dDescription(BaseDescription):
        ANGULAR_RESOLUTION_H = 'ang_res_h'
        ANGULAR_RESOLUTION_V = 'ang_res_v'
        MINIMUM_ANGLE_H = 'min_ang_h'
        MAXIMUM_ANGLE_H = 'max_ang_h'
        MINIMUM_ANGLE_V = 'min_ang_v'
        MAXIMUM_ANGLE_V = 'max_ang_v'
        MINIMUM_RANGE = 'min_range'
        MAXIMUM_RANGE = 'max_range'
        UPDATE_RATE = 'update_rate'

        def __init__(self, sensor: BaseLidar3D) -> None:
            super().__init__(sensor)

            self.parameters.update({
                self.ANGULAR_RESOLUTION_H: 0.4,
                self.ANGULAR_RESOLUTION_V: 2.0,
                self.MINIMUM_ANGLE_H: -3.141592,
                self.MAXIMUM_ANGLE_H: 3.141592,
                self.MINIMUM_ANGLE_V: -0.261799,
                self.MAXIMUM_ANGLE_V: 0.261799,
                self.MINIMUM_RANGE: 0.9,
                self.MAXIMUM_RANGE: 130.0,
                self.UPDATE_RATE: 20  # TODO: link to clearpath_config property
            })

    class InsDescription(BaseDescription):
        NUM_ANTENNAS = 'num_antennas'

        GPS_0_TYPE = 'gps_0_type'
        GPS_0_XYZ_X = 'gps_0_xyz_x'
        GPS_0_XYZ_Y = 'gps_0_xyz_y'
        GPS_0_XYZ_Z = 'gps_0_xyz_z'
        GPS_0_RPY_R = 'gps_0_rpy_r'
        GPS_0_RPY_P = 'gps_0_rpy_p'
        GPS_0_RPY_Y = 'gps_0_rpy_y'
        GPS_0_PARENT = 'gps_0_parent'

        GPS_1_TYPE = 'gps_1_type'
        GPS_1_XYZ_X = 'gps_1_xyz_x'
        GPS_1_XYZ_Y = 'gps_1_xyz_y'
        GPS_1_XYZ_Z = 'gps_1_xyz_z'
        GPS_1_RPY_R = 'gps_1_rpy_r'
        GPS_1_RPY_P = 'gps_1_rpy_p'
        GPS_1_RPY_Y = 'gps_1_rpy_y'
        GPS_1_PARENT = 'gps_1_parent'

        def __init__(self, sensor: BaseINS) -> None:
            super().__init__(sensor)

            self.parameters.update({
                self.NUM_ANTENNAS: len(sensor.antennas),

                self.GPS_0_TYPE: sensor.antennas[0].antenna_type,
                self.GPS_0_XYZ_X: sensor.antennas[0].xyz[0],
                self.GPS_0_XYZ_Y: sensor.antennas[0].xyz[1],
                self.GPS_0_XYZ_Z: sensor.antennas[0].xyz[2],
                self.GPS_0_RPY_R: sensor.antennas[0].rpy[0],
                self.GPS_0_RPY_P: sensor.antennas[0].rpy[1],
                self.GPS_0_RPY_Y: sensor.antennas[0].rpy[2],
                self.GPS_0_PARENT: sensor.antennas[0].parent,

                # we only have 1 or 2 antennas, so use -1:
                # if there's only one antenna this is the same as 0
                # but the duplication is safely ignored because
                # we set NUM_ANTENNAS above
                self.GPS_1_TYPE: sensor.antennas[-1].antenna_type,
                self.GPS_1_XYZ_X: sensor.antennas[-1].xyz[0],
                self.GPS_1_XYZ_Y: sensor.antennas[-1].xyz[1],
                self.GPS_1_XYZ_Z: sensor.antennas[-1].xyz[2],
                self.GPS_1_RPY_R: sensor.antennas[-1].rpy[0],
                self.GPS_1_RPY_P: sensor.antennas[-1].rpy[1],
                self.GPS_1_RPY_Y: sensor.antennas[-1].rpy[2],
                self.GPS_1_PARENT: sensor.antennas[-1].parent,
            })

    class OusterOS1Description(Lidar3dDescription):
        SAMPLES_HORIZONTAL = 'samples_h'
        SAMPLES_VERTICAL = 'samples_v'
        BASE_TYPE = 'base'
        CAP_TYPE = 'cap'

        def __init__(self, sensor: OusterOS1) -> None:
            super().__init__(sensor)

            del self.parameters[self.ANGULAR_RESOLUTION_H]
            del self.parameters[self.ANGULAR_RESOLUTION_V]
            self.parameters.update({
                self.SAMPLES_HORIZONTAL: 1024,
                self.SAMPLES_VERTICAL: 64,
                self.BASE_TYPE: sensor.base_type,
                self.CAP_TYPE: sensor.cap_type,
            })

    class SeyondLidarDescription(Lidar3dDescription):

        def __init__(self, sensor: BaseLidar3D) -> None:
            super().__init__(sensor)

            self.parameters.update({
                self.ANGULAR_RESOLUTION_H: 0.01,
                self.ANGULAR_RESOLUTION_V: 0.01,
                self.MINIMUM_ANGLE_H: -1.0471975511965976,
                self.MAXIMUM_ANGLE_H: 1.0471975511965976,
                self.MINIMUM_ANGLE_V: -0.6108652381980153,
                self.MAXIMUM_ANGLE_V: 0.6108652381980153,
                self.MINIMUM_RANGE: 0.1,
                self.MAXIMUM_RANGE: 150.0,
                self.UPDATE_RATE: 20  # TODO: link to clearpath_config property
            })

    class ImuDescription(BaseDescription):
        UPDATE_RATE = 'update_rate'

        def __init__(self, sensor: BaseIMU) -> None:
            super().__init__(sensor)

            self.parameters.update({
                self.UPDATE_RATE: sensor.update_rate
            })

    class CameraDescription(BaseDescription):
        UPDATE_RATE = 'update_rate'

        def __init__(self, sensor: BaseCamera) -> None:
            super().__init__(sensor)

            self.parameters.update({
                self.UPDATE_RATE: sensor.fps
            })

    class AxisCameraDescription(CameraDescription):
        MODEL = 'model'

        def __init__(self, sensor: AxisCamera) -> None:
            super().__init__(sensor)

            self.parameters.update({
                self.MODEL: sensor.device_type,
            })

    class IntelRealsenseDescription(CameraDescription):
        IMAGE_WIDTH = 'image_width'
        IMAGE_HEIGHT = 'image_height'
        MODEL = 'model'

        def __init__(self, sensor: IntelRealsense) -> None:
            super().__init__(sensor)

            self.parameters.update({
                self.IMAGE_HEIGHT: sensor.color_height,
                self.IMAGE_WIDTH: sensor.color_width,
                self.MODEL: sensor.device_type,
            })

    class LuxonisOAKDDescription(CameraDescription):
        MODEL = 'model'

        def __init__(self, sensor: LuxonisOAKD) -> None:
            super().__init__(sensor)

            self.parameters.update({
                self.MODEL: sensor.device_type,
            })

    class StereolabsZedDescription(CameraDescription):
        MODEL = 'model'

        def __init__(self, sensor: StereolabsZed) -> None:
            super().__init__(sensor)

            self.parameters.update({
                self.MODEL: sensor.device_type
            })

    MODEL = {
        HokuyoUST.SENSOR_MODEL: Lidar2dDescription,
        SickLMS1XX.SENSOR_MODEL: Lidar2dDescription,
        IntelRealsense.SENSOR_MODEL: IntelRealsenseDescription,
        FlirBlackfly.SENSOR_MODEL: CameraDescription,
        AxisCamera.SENSOR_MODEL: AxisCameraDescription,
        Microstrain.SENSOR_MODEL: ImuDescription,
        OusterOS1.SENSOR_MODEL: OusterOS1Description,
        SeyondLidar.SENSOR_MODEL: SeyondLidarDescription,
        VelodyneLidar.SENSOR_MODEL: Lidar3dDescription,
        CHRoboticsUM6.SENSOR_MODEL: ImuDescription,
        RedshiftUM7.SENSOR_MODEL: ImuDescription,
        StereolabsZed.SENSOR_MODEL: StereolabsZedDescription,
        LuxonisOAKD.SENSOR_MODEL: LuxonisOAKDDescription,
        Fixposition.SENSOR_MODEL: InsDescription,
    }

    def __new__(cls, sensor: BaseSensor) -> BaseDescription:
        return SensorDescription.MODEL.setdefault(
            sensor.SENSOR_MODEL,
            SensorDescription.BaseDescription)(sensor)
