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
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from clearpath_generator_common.discovery_server.generator import DiscoveryServerGenerator


class TestRobotLaunchGenerator:

    def test_samples(self):
        errors = []
        share_dir = get_package_share_directory('clearpath_config')
        sample_dir = os.path.join(share_dir, 'sample')
        for sample in os.listdir(sample_dir):
            # Create Clearpath Directory
            src = os.path.join(sample_dir, sample)
            dst = os.path.join(os.environ['HOME'], '.clearpath', 'robot.yaml')
            shutil.rmtree(os.path.dirname(dst), ignore_errors=True)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copy(src, dst)
            # Generate
            try:
                rlg = DiscoveryServerGenerator(os.path.dirname(dst))
                rlg.generate()
            except Exception as e:
                errors.append("Sample '%s' failed to load: '%s'" % (
                    sample,
                    e.args[0],
                ))
        assert not errors, 'Errors: %s' % '\n'.join(errors)
