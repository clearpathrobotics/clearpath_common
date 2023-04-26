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

from clearpath_config.sensors.base import BaseSensor
from clearpath_config.sensors.lidars_2d import HokuyoUST10, SickLMS1XX
from clearpath_config.sensors.cameras import IntelRealsense
from clearpath_config.sensors.imu import Microstrain

from typing import List


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
                self.NAME: sensor.get_name(),
                self.PARENT: sensor.get_parent()
            }

        def get_parameters(self) -> dict:
            return self.parameters

        def get_parameter(self, parameter: str) -> str:
            return self.parameters[parameter]

        def get_name(self) -> str:
            return self.parameters[self.NAME]

        def get_model(self) -> str:
            return self.sensor.SENSOR_MODEL

        def get_package(self) -> str:
            return self.package

        def get_path(self) -> str:
            return self.path

        def get_xyz(self) -> List[float]:
            return self.sensor.get_xyz()

        def get_rpy(self) -> List[float]:
            return self.sensor.get_rpy()

    class HokuyoUST10Description(BaseDescription):
        def __init__(self, sensor: HokuyoUST10) -> None:
            super().__init__(sensor)

    class SickLMS1XXDescription(BaseDescription):
        def __init__(self, sensor: SickLMS1XX) -> None:
            super().__init__(sensor)

    class IntelRealsenseDescription(BaseDescription):
        def __init__(self, sensor: IntelRealsense) -> None:
            super().__init__(sensor)

    class MicrostrainIMUDescription(BaseDescription):
        def __init__(self, sensor: Microstrain) -> None:
            super().__init__(sensor)

    MODEL = {
        HokuyoUST10.SENSOR_MODEL: HokuyoUST10Description,
        SickLMS1XX.SENSOR_MODEL: SickLMS1XXDescription,
        IntelRealsense.SENSOR_MODEL: IntelRealsenseDescription,
        Microstrain.SENSOR_MODEL: MicrostrainIMUDescription
    }

    def __new__(cls, sensor: BaseSensor) -> BaseDescription:
        return SensorDescription.MODEL[sensor.SENSOR_MODEL](sensor)
