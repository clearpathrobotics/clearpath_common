#! /usr/bin/env python -m
# -*- coding: utf-8 -*-
#     _____
#    /  _  \
#   / _/ \  \
#  / / \_/   \
# /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
# \  / \_/ \  / /  _\| |  |  _| / _ \ |  _ \ |  _ \ / _ \ |_   _|| | | |
#  \ \_/ \_/ /  | |  | |  | └─┐| |_| || |/ / | |/ /| |_| |  | |  | └─┘ |
#   \  \_/  /   | |_ | |_ | ┌_┘|  _  ||  _ \ |  _/ |  _  |  | |  | ┌─┐ |
#    \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
#            ROBOTICS™
#
#  File: codes.py
#  Desc: Horizon Message Codes
#  
#  Copyright © 2010 Clearpath Robotics, Inc. 
#  All Rights Reserved
# 
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#      * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#      * Redistributions in binary form must reproduce the above copyright
#        notice, this list of conditions and the following disclaimer in the
#        documentation and/or other materials provided with the distribution.
#      * Neither the name of Clearpath Robotics, Inc. nor the
#        names of its contributors may be used to endorse or promote products
#        derived from this software without specific prior written permission.
# 
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
#  ARE DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
#  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#  Please send comments, questions, or patches to code@clearpathrobotics.com
#


from . import payloads         # Horizon Protocol Message Payload Definitions

from collections import namedtuple


__version__  = "1.0"
__revision__ = "$Revision: 916 $"

VERSION_BYTE = 0x00

SET = 0x01
REQUEST = 0x02

class Code:
    def __init__(self, base_code, capability, data_payload_cls, request_payload_cls=None):
        self.base_code = base_code
        self.capability = capability
        self.data_payload = data_payload_cls
        if request_payload_cls:
            # Has own request class
            self.request_payload = request_payload_cls
        else:
            # Use fallback subscribe request
            self.request_payload = payloads.Subscribe

    def set(self):
        if self.capability & SET: return self.base_code

    def request(self):
        if self.capability & REQUEST: return self.base_code + 0x4000

    def data(self):
        if self.capability & REQUEST: return self.base_code + 0x8000

codes = {}
names = {}

def extend(new_codes):
    global codes, names
    codes.update(new_codes)
    for name, code in new_codes.items():
        if code.set(): names[code.set()] = name
        if code.request(): names[code.request()] = name
        if code.data(): names[code.data()] = name


extend({
    'echo':
        Code( 0x0000, REQUEST, payloads.Echo ),
    'platform_info':
        Code( 0x0001, SET | REQUEST, payloads.PlatformInfo ),
    'platform_name':
        Code( 0x0002, SET | REQUEST, payloads.PlatformName ),
    'firmware_info':
        Code( 0x0003, REQUEST, payloads.FirmwareInfo ),
    'system_status':
        Code( 0x0004, REQUEST, payloads.SystemStatus ),
#    'platform_time':
#        Code( 0x0005, SET, payloads.PlatformTime ),
    'power_status':
         Code( 0x0005, REQUEST, payloads.PowerStatus ),
    'processor_status':
        Code( 0x0006, REQUEST, payloads.ProcessorStatus ),
    'safety_status':
        Code( 0x0010, REQUEST, payloads.SafetyStatus ),
    'config':
        Code( 0x0100, SET | REQUEST, payloads.Config, payloads.ConfigRequest ),
    'differential_speed':
        Code( 0x0200, SET | REQUEST, payloads.DifferentialSpeed ),
    'differential_control':
        Code( 0x0201, SET | REQUEST, payloads.DifferentialControl ),
    'differential_output':
        Code( 0x0202, SET | REQUEST, payloads.DifferentialOutput ),
    'ackermann_output':
        Code( 0x0203, SET | REQUEST, payloads.AckermannOutput ),
    'differential_current':
        Code( 0x0220, SET | REQUEST, payloads.DifferentialCurrent ),
    'differential_current_control':
        Code( 0x0221, SET | REQUEST, payloads.DifferentialCurrentControl ),
    'velocity':
        Code( 0x0204, SET | REQUEST, payloads.Velocity ),
    'turn':
        Code( 0x0205, SET | REQUEST, payloads.Turn ),
    'max_speed':
        Code( 0x0210, SET | REQUEST, payloads.MaxSpeed ),
    'max_accel':
        Code( 0x0211, SET | REQUEST, payloads.MaxAccel ),
    'gear_status':
        Code( 0x0212, SET | REQUEST, payloads.GearStatus ),
#    'gpadc_output':
#        Code( 0x0300, SET | REQUEST, payloads.GPADCOutput ),
#    'gpio':
#        Code( 0x0301, SET | REQUEST, payloads.GPIO ),
#    'gpio_output':
#        Code( 0x0302, SET, payloads.GPIOOutput ),
#    'gpadc_input':
#        Code( 0x0303, REQUEST, payloads.GPADCInput ),
#    'pan_tilt_zoom':
#        Code( 0x0400, SET | REQUEST, payloads.PanTiltZoom ),
    'distance':
        Code( 0x0500, REQUEST, payloads.Distance ),
    'distance_timing':
        Code( 0x0501, REQUEST, payloads.DistanceTiming ),
    'platform_orientation':
        Code( 0x0600, REQUEST , payloads.Orientation ),
    'platform_rotation':
        Code( 0x0601, REQUEST, payloads.Rotation ),
    'platform_acceleration':
        Code( 0x0602, REQUEST, payloads.Acceleration ),
#    'platform_6axis':
#       Code( None, 0x4603, 0x8603, payloads.Platform6Axis ),
#    'platform_6axis_orientation':
#       Code( None, 0x4604, 0x8604, payloads.Platform6AxisOrientation ),
    'platform_magnetometer':
        Code( 0x0606, REQUEST, payloads.Magnetometer ),
    'encoders':
        Code( 0x0800, REQUEST, payloads.Encoders ),
    'raw_encoders':
        Code( 0x0801, REQUEST, payloads.RawEncoders ),
    'encoders_config':
        Code( 0x0802, SET | REQUEST, payloads.EncodersConfig ),
#    'absolute_joint_position':
#        Code( 0x1010, 0x5010, 0x9010, payloads.AbsoluteJointPosition ),
#    'relative_joint_position':
#        Code( 0x1011, 0x5011, 0x9011, payloads.RelativeJointPosition ),
#    'joint_control':
#        Code( 0x1012, 0x5012, 0x9012, payloads.JointControl ),
#    'joint_homing_status':
#        Code( 0x1013, 0x5013, 0x9013, payloads.JointHomingStatus ),
#    'joint_torques':
#        Code( None, 0x5014, 0x9014, payloads.JointTorques ),
#    'end_effector_position':
#        Code( 0x1020, 0x5020, 0x9020, payloads.EndEffectorPosition ),
#    'end_effector_pose':
#        Code( 0x1021, 0x5021, 0x9021, payloads.EndEffectorPose ),
#    'end_effector_orientation':
#        Code( None, 0x5022, 0x9022, payloads.EndEffectorOrientation ),
    'reset':
        Code( 0x2000, SET, payloads.Reset ),
    'restore_system_config':
        Code( 0x2001, SET, payloads.RestoreSystemConfig ),
    'store_system_config':
        Code( 0x2002, SET, payloads.StoreSystemConfig ),
    'current_sensor_config':
        Code( 0x2100, SET | REQUEST, payloads.CurrentSensorConfig ),
    'voltage_sensor_config':
        Code( 0x2101, SET | REQUEST, payloads.VoltageSensorConfig ),
    'temperature_sensor_config':
        Code( 0x2102, SET | REQUEST, payloads.TemperatureSensorConfig ),
    'orientation_sensor_config':
        Code( 0x2103, SET | REQUEST, payloads.OrientationSensorConfig ),
    'gyro_config':
        Code( 0x2104, SET | REQUEST, payloads.GyroConfig ),
    'accelerometer_config':
        Code( 0x2105, SET | REQUEST, payloads.AccelerometerConfig ),
    'magnetometer_config':
        Code( 0x2106, SET | REQUEST, payloads.MagnetometerConfig ),
    'battery_estimation_config':
        Code( 0x2107, SET | REQUEST, payloads.BatteryEstimationConfig ),
    'platform_kinematics':
        Code( 0x2108, SET | REQUEST, payloads.PlatformKinematics ),
    'raw_current_sensor':
        Code( 0x2110, REQUEST, payloads.RawCurrentSensor ),
    'raw_voltage_sensor':
        Code( 0x2111, REQUEST, payloads.RawVoltageSensor ),
    'raw_temperature_sensor':
        Code( 0x2112, REQUEST, payloads.RawTemperatureSensor ),
    'raw_orientation_sensor':
        Code( 0x2113, REQUEST, payloads.RawOrientationSensor ),
    'raw_gyro':
        Code( 0x2114, REQUEST, payloads.RawGyro ),
    'raw_accelerometer':
        Code( 0x2115, REQUEST, payloads.RawAccelerometer ),
    'raw_magnetometer':
        Code( 0x2116, REQUEST, payloads.RawMagnetometer ),
    'control_flags':
        Code( 0x2130, SET | REQUEST, payloads.ControlFlags )
})
