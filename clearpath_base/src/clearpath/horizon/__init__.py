#! /usr/bin/env python -m
# -*- coding: utf-8 -*-
#     _____
#    /  _  \
#   / _/ \  \
#  / / \_/   \
# /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
# \  / \_/ \  / /  _\| |  | __| / _ \ | ┌┐ \ | ┌┐ \ / _ \ |_   _|| | | |
#  \ \_/ \_/ /  | |  | |  | └─┐| |_| || └┘ / | └┘_/| |_| |  | |  | └─┘ |
#   \  \_/  /   | |_ | |_ | ┌─┘|  _  || |\ \ | |   |  _  |  | |  | ┌─┐ |
#    \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
#            ROBOTICS™
#
#  File: horizon/__init__.py
#  Desc: Horizon Python Module
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

if __name__ == "__main__":
    print ("ERROR: clearpath.horizon is a module and can NOT be run"\
           " as a script!\nFor a command-line interface demo of Horizon, run:"\
           "\n  python -m clearpath.horizon.demo\n")

    import sys
    sys.exit(1)


## @package clearpath.horizon
#  Horizon Python Module
# 
#  Horizon Interface.                                                         
#  Supported Horizon version(s): 0.1 - 1.0
#
#  @author     Ryan Gariepy
#  @author     Malcolm Robert
#  @author     Michael Purvis
#  @date       14/01/10
#  @req        clearpath.utils                                                
#              clearpath.horizon.protocol                                     
#              clearpath.horizon.transports                                   
#  @version    1.0
#
#
#  @section HISTORY
#  Version 0.1 - 0.3 {Ryan Gariepy}
#  - Initial Creation as protocol_demo.py
#
#  Version 0.4 {Malcolm Robert}
#  - Move to horizon.py
#  - Added logging
#  - Added version support
#  - Added Doxygen documentation
#  - Changed version scheme to match Horizon doc
#  - Horizon support for v0.4 messages
#
#  Version 0.5
#  - Added TCP and UDP support
#  - Horizon support for v0.5
#
#  Version 0.6
#  - Move to horizon package __init__.py
#  - Horizon support for v0.6
#
#  Version 0.7
#  - Added Encryption Support
#  - Horizon support for v0.7
#
#  Version 0.8
#  - Horizon support for v0.8
#
#  Version 1.0
#  - Horizon support for v 0.1 - 1.0
#  - Python 2.6+ & 3.x compatible
#


# Required Clearpath Modules
from .. import utils
from . import protocol

# Required Python Modules
import datetime                 # Date & Time Manipulation
import logging                  # Logging Utilities
import time                     # System Date & Time
import inspect                  # For parameter comprehension in command decorators

__version__  = "1.0"
__revision__ = "$Revision: 916 $"

logger = logging.getLogger('clearpath.horizon')
"""Horizon Module Log"""
logger.setLevel(logging.DEBUG)
logger.addHandler(utils.NullLoggingHandler())
logger.propagate = True
logger.debug("Loading clearpath.horizon ...")    



## Horizon Interface.
#
#  Interface for interaction over a transport using the Horizon protocol.
#  Contains direct 1-1 function mappings for commands and requests.
#
class Horizon(object):    
    version = (1, 1)

    ## Create a Horizon Interface.
    #
    #  Constructor for the Horizon Interface class.                           
    #  Performs the initial creation and initialization of the underlying 
    #  protocol and transport through instantiation.                          
    #  Supports version auto-detection (will take place in open) by sending
    #  a request for the platform information message.
    #
    #  @param  retries        The number of times to retry sending a message
    #                         that received a timeout or checksum error
    #  @param  rec_timeout    The time to wait for a data message after
    #                         sending a request, 0 - wait indefinitely
    #  @param  send_timeout   The time to wait for an acknowledgment
    #                         in milliseconds, 0 - wait indefinitely
    #  @param  store_timeout  The time to store an un-handled message for the 
    #                         method get_waiting in milliseconds,
    #                         0 - store indefinitely
    #  @param  transport      The Transport class to use.
    #  @param  transport_args Dictionary of arguments to pass to the transport's
    #                         __init__ method. Do NOT include version or
    #                         store_timeout as these will be populated.
    #  @throws LookupError    If auto-detect version fails
    #  @throws TransportError Upon transport creation/initialization failure
    #  @throws ValueError     From bad arguments
    #
    #  @pydoc
    def __init__(self, transport = protocol.transports.Serial.autodetect, 
                 transport_args = {}, retries = 5, send_timeout = 50,
                 rec_timeout = 100, store_timeout = 2000):
        
        self._protocol = None        
        self._protocol = protocol.Client(transport, transport_args, retries,
                                         send_timeout, rec_timeout, store_timeout)
        
    def __str__(self):
        return str(self._protocol)

    def __del__(self):
        self.close()
           
    def open(self):
        if not self._protocol.is_open():
            self._protocol.open()
            
            # Get version from device
            firm = self.request_firmware_info()
            self.version = firm.version
        
    def close(self):
        if self._protocol != None and self._protocol.is_open():
            self._protocol.close()


    def acks(self, enabled):
        if enabled:
            self._protocol.acks = True
        else:
            self._protocol.acks = False
                
    #-------------------------------- Safeties ---------------------------------
                    
    def emergency_stop(self):        
        self._protocol.emergency_stop()

    def reset(self):
        self.set_reset()


    #-------------------------------- Commands ---------------------------------
    
    def set_platform_info(self, passcode = 0, model = '', revision = 0, serial = 0):
        self._protocol.command('platform_info', locals())

    def set_platform_name(self, name = 'Clearpath1'):
        self._protocol.command('platform_name', locals())
        
    def set_platform_time(self, time = 0):
        self._protocol.command('platform_time', locals())

    def set_platform_kinematics(self, passcode = 0, track = 0.0, wheelbase = 0.0):
        self._protocol.command('platform_kinematics', locals())

    def set_control_flags(self, passcode = 0, flags = 0x00000000):
        self._protocol.command('control_flags', locals())

    def set_safety_status(self, flags = 0x0000):
        self._protocol.command('safety_status', locals())
        
    def set_config(self, index=0, value=0.0):
        self._protocol.command('config', locals())

    def set_differential_speed(self, left_speed = 0.0, right_speed = 0.0,
                               left_accel = 0.0, right_accel = 0.0):
        self._protocol.command('differential_speed', locals())

    def set_differential_current(self, left = 0.0, right = 0.0):
        self._protocol.command('differential_current', locals())

    def set_differential_control(self, left_p = 0.0, left_i = 0.0, left_d = 0.0, 
                          left_ffwd = 0.0, left_stic = 0.0, left_sat = 0.0, 
                          right_p = 0.0, right_i = 0.0, right_d = 0.0, right_ffwd = 0.0, 
                          right_stic = 0.0, right_sat = 0.0):
        self._protocol.command('differential_control', locals())

    def set_differential_current_control(self, left_p = 0.0, left_i = 0.0, left_d = 0.0, 
                          left_ffwd = 0.0, left_stic = 0.0, left_sat = 0.0, 
                          right_p = 0.0, right_i = 0.0, right_d = 0.0, right_ffwd = 0.0, 
                          right_stic = 0.0, right_sat = 0.0):
        self._protocol.command('differential_current_control', locals())
    
    def set_differential_output(self, left = 0.0, right = 0.0):
        self._protocol.command('differential_output', locals())

    def set_ackermann_output(self, steering = 0.0, throttle = 0.0, brake = 0.0):
        self._protocol.command('ackermann_output', locals())

    def set_velocity(self, trans = 0.0, rot = 0.0, accel = 0.0):
        self._protocol.command('velocity', locals())
    
    def set_turn(self, trans = 0.0, rad = 0.0, accel = 0.0):
        self._protocol.command('turn', locals())
        
    def set_max_speed(self, forward = 0.0, reverse = 0.0):
        self._protocol.command('max_speed', locals())
        
    def set_max_accel(self, forward = 0.0, reverse = 0.0):
        self._protocol.command('max_accel', locals())

    def set_gear(self, gear = 0):
        self._protocol.command('gear', locals())
    
    def set_gpadc_output(self, values = {0:0}):
        self._protocol.command('gpadc_output', locals())
    
    def set_gpio_direction(self, mask = 0, direction = 0):
        self._protocol.command('gpio_direction', locals())
    
    def set_gpio_output(self, mask = 0, output = 0):
        self._protocol.command('gpio_output', locals())
    
    def set_pan_tilt_zoom(self, mount = 0, pan = 0.0, tilt = 0.0,  zoom = 1.0):
        self._protocol.command('pan_tilt_zoom', locals())
    
    def set_absolute_joint_position(self, angles = {0:0.0}):
        self._protocol.command('absolute_joint_position', locals())
    
    def set_relative_joint_position(self, angles = {0:0.0}):
        self._protocol.command('relative_joint_position', locals())
    
    def set_joint_control(self, joint = 0, p = 0.0, i = 0.0, d = 0.0, 
                          feed = 0.0, stiction = 0.0, limit = 0.0):
        self._protocol.command('joint_control', locals())
    
    def set_joint_homing(self, joint = 0):
        self._protocol.command('joint_homing', locals())
    
    def set_end_effector_position(self, x = 0.0, y = 0.0, z = 0.0):
        self._protocol.command('end_effector_position', locals())
    
    def set_end_effector_pose(self, x = 0.0, y = 0.0, z = 0.0, roll = 0.0,  pitch = 0.0, yaw = 0.0):
        self._protocol.command('end_effector_pose', locals())
    
    def set_reset(self):
        self._protocol.command('reset', locals())

    def restore_system_config(self, passcode = 0x3A18, flags = 0x1):
        self._protocol.command('restore_system_config', locals())

    def store_system_config(self, passcode = 0x3A18):
        self._protocol.command('store_system_config', locals())

    def set_current_sensor_config(self, passcode = 0, offsets = [ 0.0 ], scales = [ 0.0 ]):
        self._protocol.command('current_sensor_config', locals())

    def set_voltage_sensor_config(self, passcode = 0, offsets = [ 0.0 ], scales = [ 0.0 ]):
        self._protocol.command('voltage_sensor_config', locals())

    def set_temperature_sensor_config(self, passcode = 0, offsets = [ 0.0 ], scales = [ 0.0 ]):
        self._protocol.command('temperature_sensor_config', locals())

    def set_orientation_sensor_config(self, passcode = 0,
                                      roll_offset = 0.0, roll_scale = 0.0,
                                      pitch_offset = 0.0, pitch_scale = 0.0,
                                      yaw_offset = 0.0, yaw_scale = 0.0):
        self._protocol.command('orientation_sensor_config', locals())

    def set_magnetometer_config(self, passcode = 0,
                                      x_offset = 0.0, x_scale = 0.0,
                                      y_offset = 0.0, y_scale = 0.0,
                                      z_offset = 0.0, z_scale = 0.0):
        self._protocol.command('magnetometer_config', locals())

    def set_gyro_config(self, passcode = 0,
                        roll_offset = 0.0, roll_scale = 0.0,
                        pitch_offset = 0.0, pitch_scale = 0.0,
                        yaw_offset = 0.0, yaw_scale = 0.0):
        self._protocol.command('gyro_config', locals())

    def set_accelerometer_config(self, passcode = 0,
                                 x_offset = 0.0, x_scale = 0.0,
                                 y_offset = 0.0, y_scale = 0.0,
                                 z_offset = 0.0, z_scale = 0.0):
        self._protocol.command('accelerometer_config', locals())

    def set_encoders_config(self, ppr = [ 0.0 ], scales = [ 0.0 ]):
        self._protocol.command('encoders_config', locals())

    def set_battery_estimation_config(self, passcode = 0.0, offsets = [ 0.0 ], scales = [ 0.0 ]):
        self._protocol.command('battery_estimation_config', locals())

    
    #-------------------------------- Requests ---------------------------------

    def request_echo(self, subscription = 0):
        return self._protocol.request('echo', locals())
        
    def request_platform_info(self, subscription = 0):
        return self._protocol.request('platform_info', locals())    
  
    def request_platform_name(self, subscription = 0):
        return self._protocol.request('platform_name', locals())
       
    def request_platform_kinematics(self, subscription = 0):
        return self._protocol.request('platform_kinematics', locals())
   
    def request_platform_kinematics(self, subscription = 0):
        return self._protocol.request('platform_kinematics', locals())   

    def request_firmware_info(self, subscription = 0):
        return self._protocol.request('firmware_info', locals())       

    def request_control_flags(self, subscription = 0):
        return self._protocol.request('control_flags', locals())

    def request_system_status(self, subscription = 0):
        return self._protocol.request('system_status', locals())

    def request_processor_status(self, subscription = 0):
        return self._protocol.request('processor_status', locals())

    def request_power_status(self, subscription = 0):
        return self._protocol.request('power_status', locals()) 

    def request_safety_status(self, subscription = 0):
        return self._protocol.request('safety_status', locals())

    def request_config(self, index=0):
        return self._protocol.request('config', locals())

    def request_differential_speed(self, subscription = 0):
        return self._protocol.request('differential_speed', locals())

    def request_differential_control(self, subscription = 0):
        return self._protocol.request('differential_control', locals())

    def request_differential_current(self, subscription = 0):
        return self._protocol.request('differential_current', locals())

    def request_differential_current_control(self, subscription = 0):
        return self._protocol.request('differential_current_control', locals())

    def request_differential_output(self, subscription = 0):
        return self._protocol.request('differential_output', locals())
        
    def request_ackermann_output(self, subscription = 0):
        return self._protocol.request('ackermann_output', locals())
        
    def request_velocity(self, subscription = 0):
        return self._protocol.request('velocity', locals())
        
    def request_turn(self, subscription = 0):
        return self._protocol.request('turn', locals())
        
    def request_max_speed(self, subscription = 0):
        return self._protocol.request('max_speed', locals())
        
    def request_max_accel(self, subscription = 0):
        return self._protocol.request('max_accel', locals())
        
    def request_gear_status(self, subscription = 0):
        return self._protocol.request('gear_status', locals())
    
    def request_gpadc_output(self, subscription = 0, channel=0):
        return self._protocol.request('gpadc_output', locals())
        
    def request_gpio(self, subscription = 0):
        return self._protocol.request('gpio', locals())
    
    def request_gpadc_input(self, subscription = 0):
        return self._protocol.request('gpadc_input', locals())
        
    def request_pan_tilt_zoom(self, mount = 0, subscription = 0):
        return self._protocol.request('pan_tilt_zoom', locals())
        
    def request_distance(self, subscription = 0):
        return self._protocol.request('distance', locals())
        
    def request_distance_timing(self, subscription = 0):
        return self._protocol.request('distance_timing', locals())
    
    def request_platform_orientation(self, subscription = 0):
        return self._protocol.request('platform_orientation', locals())
        
    def request_platform_rotation(self, subscription = 0):
        return self._protocol.request('platform_rotation', locals())
        
    def request_platform_acceleration(self, subscription = 0):
        return self._protocol.request('platform_acceleration', locals())
        
    def request_platform_6axis(self, subscription = 0):
        return self._protocol.request('platform_6axis', locals())
        
    def request_platform_6axis_orientation(self, subscription = 0):
        return self._protocol.request('platform_6axis_orientation', locals())
    
    def request_platform_magnetometer(self, subscription = 0):
        return self._protocol.request('platform_magnetometer', locals())

    def request_encoders(self, subscription = 0):
        return self._protocol.request('encoders', locals())
    
    def request_raw_encoders(self, subscription = 0):
        return self._protocol.request('raw_encoders', locals())
        
    def request_encoders_config(self, subscription = 0):
        return self._protocol.request('encoders_config', locals())

    def request_absolute_joint_position(self, subscription = 0):
        return self._protocol.request('absolute_joint_position', locals())
    
    def request_relative_joint_position(self, subscription = 0):
        return self._protocol.request('relative_joint_position', locals())
    
    def request_joint_control(self, joint = 0, subscription = 0):
        return self._protocol.request('joint_control', locals())
    
    def request_joint_homing_status(self, subscription = 0):
        return self._protocol.request('joint_homing_status', locals())
    
    def request_joint_torques(self, subscription = 0):
        return self._protocol.request('joint_torques', locals())
    
    def request_end_effector_position(self, subscription = 0):
        return self._protocol.request('end_effector_position', locals())
    
    def request_end_effector_pose(self, subscription = 0):
        return self._protocol.request('end_effector_pose', locals())
    
    def request_end_effector_orientation(self, subscription = 0):
        return self._protocol.request('end_effector_orientation', locals())
        
    def request_current_sensor_config(self, subscription = 0):
        return self._protocol.request('current_sensor_config', locals())

    def request_voltage_sensor_config(self, subscription = 0):
        return self._protocol.request('voltage_sensor_config', locals())

    def request_temperature_sensor_config(self, subscription = 0):
        return self._protocol.request('temperature_sensor_config', locals())

    def request_orientation_sensor_config(self, subscription = 0):
        return self._protocol.request('orientation_sensor_config', locals())

    def request_magnetometer_config(self, subscription = 0):
        return self._protocol.request('magnetometer_config', locals())

    def request_gyro_config(self, subscription = 0):
        return self._protocol.request('gyro_config', locals())

    def request_battery_estimation_config(self, subscription = 0):
        return self._protocol.request('battery_estimation_config', locals())

    def request_accelerometer_config(self, subscription = 0):
        return self._protocol.request('accelerometer_config', locals())

    def request_raw_current_sensor(self, subscription = 0):
        return self._protocol.request('raw_current_sensor', locals())

    def request_raw_voltage_sensor(self, subscription = 0):
        return self._protocol.request('raw_voltage_sensor', locals())

    def request_raw_temperature_sensor(self, subscription = 0):
        return self._protocol.request('raw_temperature_sensor', locals())

    def request_raw_orientation_sensor(self, subscription = 0):
        return self._protocol.request('raw_orientation_sensor', locals())

    def request_raw_magnetometer(self, subscription = 0):
        return self._protocol.request('raw_magnetometer', locals())

    def request_raw_gyro(self, subscription = 0):
        return self._protocol.request('raw_gyro', locals())

    def request_raw_accelerometer(self, subscription = 0):
        return self._protocol.request('raw_accelerometer', locals())
            
            
    #------------------------------ Subscriptions ------------------------------
    
    
    ## Add Subscription Data Handler
    #  
    #  Adds a data subscription handler to be called when data is received.   
    #  Asynchronous method of getting subscription Data.
    #
    #  @param  backtrack  Call the new handler with any waiting data?
    #  @param  request    The name of the request data to handle, ie 'platform_info'
    #                     If None then it returns all subscription data.
    #  @param  handler    The Message Handler. 
    #                     Must have three parameters: 
    #                        str name,
    #                        Payload payload,
    #                        long timestamp
    #
    def add_handler(self, handler, backtrack = False, request = None):
        self._protocol.add_handler(handler, backtrack, request)
    
    
    ## Remove Subscription Data Handler
    #  
    #  Removes a data subscription handler.
    #
    #  @param  handler   The Handler to remove.
    #                    If None, remove all handlers. 
    #  @param  request   The name of the requested data.
    #
    def remove_handler(self, handler = None, request = None):
        self._protocol.remove_handler(handler, request)

        
    ## Get Waiting Data
    #
    #  Returns payload(s) from any waiting messages.                          
    #  Synchronous method of getting subscription Data.
    #
    #  @param  request The name of the requested data.
    #                  If None then it returns all subscription data.
    #  @return list of tuple(name,payload,timestamp), empty if no waiting data.
    #                  Invalid messages are returned as payloads.Null.
    #
    def get_waiting_data(self, request = None):
        return self._protocol.get_waiting(request)
            
            
    #------------------------------- Properties --------------------------------
        

    ## Horizon Device Open?
    #
    #  @return is the horizon device open?
    #
    def is_open(self):        
        if self._protocol.is_open() == False:
            self.close()
            return False
        return True
        

    ## Horizon Device Alive?
    #
    #  @return is the horizon device alive?
    #
    def is_alive(self):
        # Echo
        try:
            self.request_echo()
            return True
        except Exception:
            return False
                
            
    ## Get Device Time
    #
    #  Note that the time returned has a delay from transmission.
    #
    #  @return device time in milliseconds
    #
    def get_device_time(self):        
        # Get Time
        # TODO: Move this to protocol.py
        self.request_echo()
        return self._protocol._received[2]
                
            
    ## Get Program Start Time
    #
    #  @return program start time in milliseconds as stored in Transport
    #
    def get_start_time(self):
        return self._protocol.start_time


    # Class Properties
    alive = property(fget=is_alive, doc="Horizon Device Alive")
    device_time = property(fget=get_device_time, doc="Horizon Device Time")
    opened = property(fget=is_open, doc="Horizon Device Open")
    start_time = property(fget=get_start_time, doc="Horizon Start Time")
