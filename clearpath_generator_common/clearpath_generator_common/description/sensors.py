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
    BaseCamera,
    FlirBlackfly,
    IntelRealsense,
    StereolabsZed
)
from clearpath_config.sensors.types.imu import (
    BaseIMU,
    CHRoboticsUM6,
    Microstrain,
    RedshiftUM7
)
from clearpath_config.sensors.types.lidars_2d import BaseLidar2D, HokuyoUST, SickLMS1XX
from clearpath_config.sensors.types.lidars_3d import BaseLidar3D, VelodyneLidar
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
                self.UPDATE_RATE: 50
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
                self.UPDATE_RATE: 50
            })

    class ImuDescription(BaseDescription):
        UPDATE_RATE = 'update_rate'

        def __init__(self, sensor: BaseIMU) -> None:
            super().__init__(sensor)

            self.parameters.update({
                self.UPDATE_RATE: 100
            })

    class CameraDescription(BaseDescription):
        UPDATE_RATE = 'update_rate'

        def __init__(self, sensor: BaseCamera) -> None:
            super().__init__(sensor)

            self.parameters.update({
                self.UPDATE_RATE: sensor.fps
            })

    class IntelRealsenseDescription(CameraDescription):
        IMAGE_WIDTH = 'image_width'
        IMAGE_HEIGHT = 'image_height'

        def __init__(self, sensor: IntelRealsense) -> None:
            super().__init__(sensor)

            self.parameters.update({
                self.IMAGE_HEIGHT: sensor.color_height,
                self.IMAGE_WIDTH: sensor.color_width,
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
        Microstrain.SENSOR_MODEL: ImuDescription,
        VelodyneLidar.SENSOR_MODEL: Lidar3dDescription,
        CHRoboticsUM6.SENSOR_MODEL: ImuDescription,
        RedshiftUM7.SENSOR_MODEL: ImuDescription,
        StereolabsZed.SENSOR_MODEL: StereolabsZedDescription,
    }

    def __new__(cls, sensor: BaseSensor) -> BaseDescription:
        return SensorDescription.MODEL.setdefault(
            sensor.SENSOR_MODEL,
            SensorDescription.BaseDescription)(sensor)
