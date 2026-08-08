# Software License Agreement (BSD)
#
# @copyright (c) 2026, Daniil Mordanov, All rights reserved.
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

from clearpath_generator_common.param.manipulators import (
    merge_ur_control_parameters,
)


def test_ur_driver_update_rate_overrides_generic_default():
    defaults = {
        'controller_manager': {
            'ros__parameters': {
                'update_rate': 500,
                'joint_state_broadcaster': {
                    'type': 'joint_state_broadcaster/JointStateBroadcaster',
                },
            },
        },
    }
    driver_parameters = {
        'controller_manager': {
            'ros__parameters': {
                'update_rate': 125,
            },
        },
    }

    merged = merge_ur_control_parameters(defaults, driver_parameters)

    assert merged['controller_manager']['ros__parameters']['update_rate'] == 125
    assert merged['controller_manager']['ros__parameters'][
        'joint_state_broadcaster'
    ] == {'type': 'joint_state_broadcaster/JointStateBroadcaster'}
    assert driver_parameters == {
        'controller_manager': {
            'ros__parameters': {
                'update_rate': 125,
            },
        },
    }
