# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2026, Clearpath Robotics, Inc., All rights reserved.
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

from ament_index_python.packages import get_package_share_directory
from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.sensors.types.gps import MicrostrainGQ7
from clearpath_config.sensors.types.imu import Microstrain
from clearpath_generator_common.param.platform import PlatformParam


class TestLocalizationParam:

    @staticmethod
    def _get_sample_path(platform: str, sample_name: str) -> str:
        share_dir = get_package_share_directory('clearpath_config')
        flat_path = os.path.join(share_dir, 'sample', sample_name)
        if os.path.exists(flat_path):
            return flat_path
        sub_path = os.path.join(share_dir, 'sample', platform, sample_name)
        if os.path.exists(sub_path):
            return sub_path
        return flat_path

    def test_a200_default_no_imu(self):
        config = ClearpathConfig(self._get_sample_path('a200', 'a200_default.yaml'))
        param = PlatformParam.LocalizationParam('localization', config, '/tmp')
        param.generate_parameters()

        ekf_params = param.param_file.parameters.get(param.EKF_NODE, {})
        imu_keys = [k for k in ekf_params if k.startswith('imu') and not k.endswith(
            ('_config', '_differential', '_queue_size', '_remove_gravitational_acceleration'))]
        assert len(imu_keys) == 0

    def test_a200_additional_imu_indexing(self):
        config = ClearpathConfig(self._get_sample_path('a200', 'a200_default.yaml'))
        imu = Microstrain(idx=0, name='imu_0', parent='base_link')
        config.sensors.imu.add(imu)

        param = PlatformParam.LocalizationParam('localization', config, '/tmp')
        param.generate_parameters()

        ekf_params = param.param_file.parameters.get(param.EKF_NODE, {})
        assert 'imu0' in ekf_params
        assert ekf_params['imu0'] == 'sensors/imu_0/data'
        assert 'imu1' not in ekf_params

    def test_a200_gps_imu_indexing(self):
        config = ClearpathConfig(self._get_sample_path('a200', 'a200_default.yaml'))
        gps = MicrostrainGQ7(idx=0, name='gps_0', parent='base_link')
        config.sensors.gps.add(gps)

        param = PlatformParam.LocalizationParam('localization', config, '/tmp')
        param.generate_parameters()

        ekf_params = param.param_file.parameters.get(param.EKF_NODE, {})
        assert 'imu0' in ekf_params
        assert ekf_params['imu0'] == 'sensors/gps_0/imu/data'
        assert 'imu1' not in ekf_params

    def test_a200_additional_imu_and_gps_imu_indexing(self):
        config = ClearpathConfig(self._get_sample_path('a200', 'a200_default.yaml'))
        imu = Microstrain(idx=0, name='imu_0', parent='base_link')
        gps = MicrostrainGQ7(idx=0, name='gps_0', parent='base_link')
        config.sensors.imu.add(imu)
        config.sensors.gps.add(gps)

        param = PlatformParam.LocalizationParam('localization', config, '/tmp')
        param.generate_parameters()

        ekf_params = param.param_file.parameters.get(param.EKF_NODE, {})
        assert 'imu0' in ekf_params
        assert ekf_params['imu0'] == 'sensors/imu_0/data'
        assert 'imu1' in ekf_params
        assert ekf_params['imu1'] == 'sensors/gps_0/imu/data'
        assert 'imu2' not in ekf_params

    def test_j100_gps_imu_indexing(self):
        config = ClearpathConfig(self._get_sample_path('j100', 'j100_default.yaml'))
        gps = MicrostrainGQ7(idx=0, name='gps_0', parent='base_link')
        config.sensors.gps.add(gps)

        param = PlatformParam.LocalizationParam('localization', config, '/tmp')
        param.generate_parameters()

        ekf_params = param.param_file.parameters.get(param.EKF_NODE, {})
        assert 'imu0' in ekf_params
        assert ekf_params['imu0'] == 'sensors/imu_0/data'
        assert 'imu1' in ekf_params
        assert ekf_params['imu1'] == 'sensors/gps_1/imu/data'
        assert 'imu2' not in ekf_params

    def test_j100_additional_imu_and_gps_imu_indexing(self):
        config = ClearpathConfig(self._get_sample_path('j100', 'j100_default.yaml'))
        imu = Microstrain(idx=1, name='imu_1', parent='base_link')
        gps = MicrostrainGQ7(idx=0, name='gps_0', parent='base_link')
        config.sensors.imu.add(imu)
        config.sensors.gps.add(gps)

        param = PlatformParam.LocalizationParam('localization', config, '/tmp')
        param.generate_parameters()

        ekf_params = param.param_file.parameters.get(param.EKF_NODE, {})
        assert 'imu0' in ekf_params
        assert ekf_params['imu0'] == 'sensors/imu_0/data'
        assert 'imu1' in ekf_params
        assert ekf_params['imu1'] == 'sensors/imu_1/data'
        assert 'imu2' in ekf_params
        assert ekf_params['imu2'] == 'sensors/gps_1/imu/data'
        assert 'imu3' not in ekf_params
