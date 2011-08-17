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
#  File: payloads.py
#  Desc: Horizon Protocol Message Definitions
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




################################################################################
# Script



# Check if run as a script
if __name__ == "__main__":
    
    # Warn of Module ONLY status
    print ("ERROR: clearpath.horizon.payloads is a module and can NOT be run"\
           " as a script!\nFor a command-line interface demo of Horizon, run:"\
           "\n  python -m clearpath.horizon.demo\n"\
           "For Horizon message forwarding, run:\n"\
           "  python -m clearpath.horizon.forward")

    # Exit Error
    import sys
    sys.exit(1)




################################################################################
# Module



## @package clearpath.horizon.payloads
#  Horizon Protocol Message Payloads Python Module
# 
#  Horizon Protocol Message Payload Definitions                               \n
#  Abstracted from knowing message codes and message header.                  \n
#  Supported Horizon version(s): 0.1 - 1.0
#
#  @author     Ryan Gariepy
#  @author     Malcolm Robert
#  @author     Michael Purvis
#  @date       25/01/10
#  @req        clearpath.utils                                                \n
#  @version    1.0
#
#  @section USE
#
#  The intended purpose of this module is to provide the various Payload
#  definitions for the various messages within Horizon. The Payload class
#  abstracts a payload and can represent a payload within a message without
#  having any knowledge of the contained format whereas subclasses should know
#  how payload data is formatted for one or more messages that they represent.
#  These classes have no knowledge of message fields that do not fall within
#  the payload field.
#
#  @section HISTORY
#  Version 0.1 - 0.3 {Ryan Gariepy}
#  - Initial Creation as protocol.py
#
#  Version 0.4 {Malcolm Robert}
#  - Move to horizon_messages.py
#  - Added manipulator payloads
#  - Added safety system payloads
#  - Added payload abstraction
#  - Added logging
#  - Added version support
#  - Added Doxygen documentation
#  - Changed version scheme to match Horizon doc
#  - Horizon support for v0.4
#
#  Version 0.5
#  - Added GPADC payload
#  - Horizon support for v0.5
#
#  Version 0.6
#  - Added content to platform info payload
#  - Move to messages.py
#
#  Version 0.7
#  - Added reset payload
#  - Fixed number scales
#  - Horizon support for v0.7
#
#  Version 0.8
#  - Added power status payload
#  - Added raw encoders payload
#  - Horizon support for v0.8
#
#  Version 1.0
#  - Move to payloads.py
#  - Horizon support for v 0.1 - 1.0
#  - Python 2.6+ & 3.x compatible
#

"""Horizon Protocol Message Payload Definitions 

   Copyright © 2010 Clearpath Robotics, Inc.
   All rights reserved
   
   Created: 25/01/10
   Authors: Ryan Gariepy & Malcolm Robert
   Version: 1.0
   """


# Required Clearpath Modules
from .. import utils            # Clearpath Utilities

# Required Python Modules
import datetime                 # Date & Time Manipulation
import logging                  # Logging Utilities
import string
import math                     # Math Constants and Functions


# Module Support
## Module Version
__version__  = "1.0"
"""Module Version"""
## SVN Code Revision
__revision__ = "$Revision: 920 $"
""" SVN Code Revision"""


## Message Log
logger = logging.getLogger('clearpath.horizon.payloads')
"""Horizon Message Payloads Module Log"""
logger.setLevel(logging.NOTSET)
logger.addHandler(utils.NullLoggingHandler())
logger.propagate = False
logger.debug("Loading clearpath.horizon.payloads ...")  




################################################################################
# Horizon Payload Superclass
    
    
    
## Horizon Payload
#
#  Represents the basic payload of a Horizon message.                         \n
#  To be inherited for specific message payloads.
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#  
#  @pydoc
class Payload(object):
    """Horizon Protocol Message Payload"""
    
    ## Create A Horizon Message Payload
    #
    #  Constructor for the Horizon Payload class.                             \n
    #  Creates a basic message payload which is simply a byte list.           \n
    #                                                                         \n
    #  Subclass overrides should throw ValueError if invalid format and 
    #  LookupError if it has version detection problems (if supported).
    #
    #  @param  raw            Raw data buffer to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, raw = [], timestamp = 0):
        """Create A Horizon Message Payload"""
        
        # Class Variables
        self.data = []
        self.timestamp = timestamp
        
        # Doesn't actually do anything, just copies raw
        self.data = raw
        logger.debug("%s: Raw payload data: %s" % (self.__class__.__name__, 
                     str(self)))
        
        
    ## Hex String Representation
    #
    #  Return the entire payload in a string of hex characters.
    # 
    #  @return String of hex info
    #
    #  @pydoc
    def __str__(self):
        """Return the entire payload in a string of hex characters."""
        return ' '.join(map(utils.hex,self.data))
    
    
    ## Copy Instance
    #
    #  @return A deep copy of this object
    #
    #  @pydoc
    def copy(self):
        """Copy Instance"""
        
        # determine required data
        raw = None
        if self.data != None: raw = self.data[:]
        
        # create new copy
        logger.debug("%s: Creating copy." % self.__class__.__name__)
        return self.__class__(raw = raw, 
                              timestamp = self.timestamp)
    
    
    ## Human Readable Payload String
    #
    #  @return Human readable string representation of the payload
    #
    #  @pydoc
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Payload: %s\n" % str(self)
    
    
    ## Raw Bytes Payload Representation
    #
    #  Convert the payload into raw bytes (character string for Python 2.x)
    #  useful for writing to devices.
    #
    #  @return raw bytes
    #
    #  @pydoc 
    def raw_string(self):
        """Returns the data converted into raw bytes."""
        
        return utils.to_bytes(self.data)
    



################################################################################
# Horizon Payloads
    
    
   
## Horizon Message Payload - Null
#
#  Represents the null payload (payload of a message without a payload).
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @pydoc
class Null(Payload):
    """Horizon Message Payload - Null"""
    
    
    ## Create A Horizon Message Payload - Null 
    #
    #  Constructor for the Horizon Message Payload - Null  Class.             \n
    #  Version auto-detection is unsupported.
    #
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, raw = None, timestamp = 0):
        """Create A Horizon Message Payload - Null"""
        
        # Verify Length
        if raw == None: raw = []
        if len(raw) != 0:
            raise ValueError("Invalid length!")
                
        # Pass on to super-class
        Payload.__init__(self, raw = raw, 
                         timestamp = timestamp)
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Payload: NULL"


## Horizon Message Payload - Acknowledgment 
#
#  Represents the payload of a standard acknowledgment message.
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @pydoc
class Ack(Payload):
    """Horizon Message Payload - Acknowledgment"""
    
    
    
    ## Create A Horizon Message Payload - Acknowledgment 
    #
    #  Constructor for the Horizon Message Payload - Acknowledgment  Class.   \n
    #  The constructor can be called two different ways:
    #  - Ack(bad_checksum,raw=None,...)                        \n
    #    Create an acknowledgment message payload to send.                    \n 
    #  - Ack(raw,timestamp)                                    \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  bad_bandwidth  Not enough bandwidth
    #  @param  bad_checksum   Message had a bad checksum
    #  @param  bad_code       Message type is unsupported
    #  @param  bad_code_count Too many subscription types
    #  @param  bad_format     Message format is bad
    #  @param  bad_frequency  Subscription frequency is too high
    #  @param  bad_values     Message value(s) are out of range
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, bad_bandwidth = False, bad_checksum = False, 
                 bad_code = False, bad_code_count = False, bad_format = False, 
                 bad_frequency = False, bad_values = False, 
                 raw = None, timestamp = 0):
        """Create A Horizon Acknowledgment Message Payload"""
        
        # Class Variables
        self.bad_bandwidth = False
        self.bad_checksum = False
        self.bad_code = False
        self.bad_code_count = False
        self.bad_format = False
        self.bad_frequency = False
        self.bad_values = False
        
        # Create Constructor
        if raw == None:
            pass
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 2:
                raise ValueError("Invalid length!")
                
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp) 
            
            # Verify Unused Space
            if utils.to_unsigned_short(raw) & 0xFF80 != 0: 
                raise ValueError("Invalid format!")
            
            # Extract checksum
            if utils.to_unsigned_short(raw) & 0x0001 > 0:
                self.bad_checksum = True
            
            # Extract type
            if utils.to_unsigned_short(raw) & 0x0002 > 0:
                self.bad_code = True
            
            # Extract format
            if utils.to_unsigned_short(raw) & 0x0004 > 0:
                self.bad_format = True
            
            # Extract Range
            if utils.to_unsigned_short(raw) & 0x0008 > 0:
                self.bad_values = True
            
            # Extract bandwidth
            if utils.to_unsigned_short(raw) & 0x0010 > 0:
                self.bad_bandwidth = True
            
            # Extract frequency
            if utils.to_unsigned_short(raw) & 0x0020 > 0:
                self.bad_frequency = True
            
            # Extract code count
            if utils.to_unsigned_short(raw) & 0x0040 > 0:
                self.bad_code_count = True
    
   
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Bad Checksum: %d\nBad Code: %d\nBad Format: %d\n"\
               "Bad Values: %d\nBad Bandwidth: %d\nBad Frequency: %d\n"\
               "Bad Code Count: %d" % (self.bad_checksum, self.bad_code, 
                self.bad_format, self.bad_values, self.bad_bandwidth, self.bad_frequency, 
                self.bad_code_count)
                

## Horizon Message Payload - Request Subscription
#
#  Represents the payload of a common request message
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section Subscriptions
#  @copydoc subscriptions
#
#  @section data Request Data
#  @copydoc request
#
#  @pydoc
class Subscribe(Payload):
    """Horizon Message Payload - Subscribe"""
    
    
    ## Create A Horizon Message Payload - Request
    #
    #  Constructor for the Horizon Message Payload - Request Class.           \n
    #  The constructor can be called two different ways:
    #  - Request(subscription, raw=None, version, timestamp)   \n
    #    Create a request message payload to send.                            \n 
    #  - Request(raw, version, timestamp)                      \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  subscription   Subscription Frequency in Hz
    #                         0 - immediate, 0xFFFF - off
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  verify         Verify the length? (useful for subclasses)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, raw = None, subscription = 0, 
                 timestamp = 0, verify = True):
        """Create A Horizon Message Payload - Request"""
        
        # Class Variables
        ## Subscription Frequency
        self.subscription = 0

        # Create Constructor
        if raw == None:
            data = []
            
            # test subscription
            if subscription < 0 or subscription > 0x0FFFF:
                raise ValueError("Invalid subscription!")

            self.subscription = subscription
            data = utils.from_unsigned_short(subscription)
            
            # Pass on to super-class
            Payload.__init__(self, raw = data, timestamp = timestamp)
        else:
            # Not a parseable message -- outbound-only
            Payload.__init__(self, raw = raw, timestamp = timestamp)
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Subscription: %s" % self.subscription
                

class Echo(Null):
    """Horizon Message Payload - Echo"""
    
    
    ## Create A Horizon Message Payload - Echo 
    #
    #  Constructor for the Horizon Message Payload - Echo  Class.             \n
    #  Version auto-detection is unsupported.
    #
    #  @see Null.__init__
    #
    #  @pydoc
    def __init__(self, raw = None, timestamp= 0):
        """Create A Horizon Message Payload - Null"""
        
        # Pass on to super-class
        Null.__init__(self, raw = raw,   timestamp = timestamp)
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Echo"


class PlatformInfo(Payload):
    """Horizon Message Payload - Platform Information"""
    
    
    ## Create A Horizon Message Payload - Platform Information
    #
    #  Constructor for the Horizon Message Payload - Platform Information
    #  Class.                                                                 \n
    #  The constructor can be called two different ways:
    #  - PlatformInformation(model, raw=None, version, ...)    \n
    #    Create a command message payload to send.                            \n 
    #  - PlatformInformation(raw, version, timestamp)          \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  model          The Platform Model
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  revision       The Platform Model Revision Number
    #  @param  serial         The Platform Serial Number
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, passcode = 0, model = '', revision = 0, serial = 0, 
                 raw = None, timestamp = 0):
        """Create A Horizon Message Payload - Platform Information"""
        
        # Class Variables
        self.passcode = passcode
        self.model = ''
        self.revision = 0
        self.serial = 0x00000000
        
        # Create Constructor - assume this is a set_platform_info payload
        if raw == None:
            data = utils.from_unsigned_short(self.passcode)
            
            # test model
            if not all(ord(c) < 256 for c in model):
                raise ValueError("Invalid ASCII model!")

            if (len(model) > 64 or len(model) < 2):
                raise ValueError("Model must be 1-63 characters!")
            self.model = model
            data += utils.from_byte(len(model))
            data += utils.from_ascii(model)
            
            # test revision
            if revision < 0 or revision > 255:
                raise ValueError("Revision must be 0-255!")
            self.revision = revision
            data += utils.from_byte(revision)
            
            # test serial
            if serial < 0 or serial > 0xFFFFFFFF:
                raise ValueError("Serial must be 0-4294967295!")

            self.serial = serial
            data += utils.from_unsigned_int(serial)
            
            # Pass on to super-class
            Payload.__init__(self, raw = data, 
                             timestamp = timestamp)
        
        # Parse Constructor
        else:
            # Pass on to super-class
            Payload.__init__(self, raw = raw,
                                                          timestamp = timestamp)            
            # Extract Model
            self.model = utils.to_ascii(raw[1:-5])
            logger.debug("%s model: %s" % (self.__class__.__name__, self.model))
                
            # Extract Revision
            self.revision = utils.to_byte(raw[-5:-4])
            logger.debug("%s revision: %d" % (self.__class__.__name__, self.revision))
            
            # Extract Serial
            self.serial = utils.to_unsigned_int(raw[-4:])
            logger.debug("%s serial: %d" % (self.__class__.__name__, self.serial))
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Platform Model: %s\nPlatform Model Revision: %d\n"\
               "Platform Serial Number: %08X" % (self.model,
                                                 self.revision,
                                                 self.serial)
    
    
class PlatformName(Payload):
    """Horizon Message Payload - Platform Name"""
    
    
    ## Create A Horizon Message Payload - Platform Name
    #
    #  Constructor for the Horizon Message Payload - Platform Name Class.     \n
    #  The constructor can be called two different ways:
    #  - PlatformName(name, raw=None, version, timestamp)      \n
    #    Create a command message payload to send.                            \n 
    #  - PlatformName(raw, version, timestamp)                 \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  name           Platform Name
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, name = 'Clearpath1', raw = None, 
                 timestamp = 0):
        """Create A Horizon Message Payload - Platform Name"""
        
        # Class Variables
        ## Platform Name
        self.name = 'Clearpath1'
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test name
            if not all(ord(c) < 256 for c in name):
                raise ValueError("Invalid ASCII name!")

            if not (len(name) > 0 and ord(name[len(name)-1]) == 0):
                name += '\0'
            if len(name) > 64 or len(name) < 2:
                raise ValueError("Name must be 1-63 characters!")

            self.name = name
            data += utils.from_byte(len(name))
            data += utils.from_ascii(name)
            
            # Pass on to super-class
            Payload.__init__(self, raw = data,                                     timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw)-1 != raw[:1][0]:
                raise ValueError("Name must be 1-63 characters!")

                
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
            
            # Extract Name
            self.name = utils.to_ascii(raw[1:])
            logger.debug("%s name: %s" % (self.__class__.__name__, 
                     self.name))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Name: %s" % self.name
    
    
## Horizon Message Payload - Platform Time
#
#  Represents the payload of the command message 'platform time'
#  @warning Data should not be modified once created
#
#  @since 0.3
#
#  @pydoc
class PlatformTime(Payload):
    """Horizon Message Payload - Platform Time"""

    
    ## Create A Horizon Message Payload - Platform Time
    #
    #  Constructor for the Horizon Message Payload - Platform Time Class.     \n
    #  The constructor can be called two different ways:
    #  - PlatformTime(time, raw=None, version, timestamp)      \n
    #    Create a command message payload to send.                            \n 
    #  - PlatformTime(raw, version, timestamp)                 \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  time           Platform Time (0-4294967295)
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, time = 0, raw = None, 
                 timestamp = 0):
        """Create A Horizon Message Payload - Platform Time"""
        
        # Class Variables
        ## Platform Time
        self.time = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test time
            if time < 0 or time > 4294967295:
                raise ValueError("Time must be within 50 days!")

            self.time = time
            data += utils.from_unsigned_int(time)
            
            # Pass on to super-class
            Payload.__init__(self, raw = data, timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 4:
                raise ValueError("Bad length!")

                
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
            
            # Extract Time
            self.time = utils.to_unsigned_int(raw)
            logger.debug("%s time: %d" % (self.__class__.__name__, 
                     self.time))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Time: %d" % self.time
 

## Horizon Message Payload - Firmware Information
#
#  Represents the payload of the data message 'firmware information'
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @pydoc
class FirmwareInfo(Payload):
    """Horizon Message Payload - Firmare Information"""
    
    ## Create A Horizon Message Payload - Firmware Information
    #
    #  Constructor for the Horizon Message Payload - Firmware Information
    #  Class.                                                                 \n
    #  The constructor can be called two different ways:
    #  - FirmwareInformation(firmware, raw=None, version,...)  \n
    #    Create a command message payload to send.                            \n 
    #  - FirmwareInformation(raw, version, timestamp)          \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is supported.
    #
    #  @param  firmware       Firmware version (major,minor)
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @param  written        Time & Date written (year: 2000-2127)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, firmware = tuple([1,0]), raw = None, 
                 timestamp = 0, 
                 written = datetime.datetime(2000,1,1,0,0)):
        """Create A Horizon Message Payload - Firmware Information"""

        # Verify Length
        if len(raw) != 8:
            raise ValueError("Bad length!")
            
        # Extract / Verify Version
        v = tuple([utils.to_byte(raw[2:3]),
                        utils.to_byte(raw[3:4])])
        self.version = v
        logger.debug("%s version: %d.%d" % (self.__class__.__name__, 
                 self.version[0], self.version[1]))
            
        # Pass on to super-class
        Payload.__init__(self, raw = raw, 
                                                timestamp = timestamp)
        
        # Extract Firmware Version
        self.firmware = tuple([utils.to_byte(raw[0:1]),
                                utils.to_byte(raw[1:2])])
        logger.debug("%s firmware: %d.%d" % (self.__class__.__name__, 
                                             self.firmware[0], self.firmware[1]))
        
        # Extract Write Time
        time = utils.to_unsigned_int(raw[4:8])
        year = 2000 + ((time >> 21) & 0x7F)
        month = 1 + ((time >> 17) & 0x0F)
        day = 1 + ((time >> 11) & 0x3F)
        hour = ((time >> 6) & 0x1F)
        minute = ((time) & 0x3F)

        try:
            self.written = datetime.datetime(year,month,day,hour,minute)
        except ValueError as ex:
            raise ValueError(ex)
        logger.debug("%s write: %s" % (self.__class__.__name__, 
                                       self.written))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Firmware: %d.%d\nProtocol: %d.%d\nWrite: %s" % (
                self.firmware[0], self.firmware[1], self.version[0], 
                self.version[1], self.written)


## Horizon Message Payload - System Status
#
#  Represents the payload of the data message 'system status'
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @pydoc
class SystemStatus(Payload):
    """Horizon Message Payload - System Status"""
    
    
    ## Create A Horizon Message Payload - System Status
    #
    #  Constructor for the Horizon Message Payload - System Status Class.     \n
    #  The constructor can be called two different ways:
    #  - SystemStatus(uptime, voltage, raw=None, version,...)  \n
    #    Create a command message payload to send.                            \n 
    #  - SystemStatus(raw, version, timestamp)                 \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  current        A list of currents [-320A,320A]
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  temperature    A list of temperatures [-320 degC, 320 degC]
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  uptime         System uptime ([0,4294967295] milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @param  voltage        A list of voltages [-320V,320V]
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If raw is invalid
    #
    #  @pydoc
    def __init__(self, uptime = 0, voltage = [], current = [], temperature = [],
                 raw = None, timestamp = 0):
        """Create A Horizon Message Payload - System Status"""
        
        # Class Variables
        self.uptime = 0
        self.currents = []
        self.temperatures = []
        self.voltages = []
        
 
        # Parse Constructor
        if raw != None:
            # Verify Length
            length = 4;
            voltc = 0
            ampc = 0
            tempc = 0
            if len(raw) > length:
                voltc = raw[4]
                length += 1 + voltc*2;
                if len(raw) > length:
                    ampc = raw[length]
                    length += 1 + ampc*2
                    if len(raw) > length:
                        tempc = raw[length]
                        length += 1 + tempc*2
                        if length != len(raw):
                            length = -1
                    else: length = -1
                else: length = -1
            else: length = -1
            if length == -1:
                raise ValueError("Measurement counts do not match raw data length!")

                    
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
                
            # Extract Uptime
            self.uptime = utils.to_unsigned_int(raw[:4])
            logger.debug("%s uptime: %d" % (self.__class__.__name__, 
                                            self.uptime))
                
            # Extract Voltages
            voltc = raw[4]
            for i in range(0,voltc):
                self.voltages.append(
                            utils.to_short(raw[5+i*2:i*2+7])/100.0)
            logger.debug("%s voltages: %s" % (self.__class__.__name__, 
                                              ' '.join(map(str,self.voltages))))
                
            # Extract Currents
            ampc = raw[voltc*2+5]
            for i in range(0,ampc):
                self.currents.append(utils.to_short(
                                        raw[voltc*2+i*2+6:i*2+8+voltc*2])/100.0)
            logger.debug("%s currents: %s" % (self.__class__.__name__, 
                                              ' '.join(map(str,self.currents))))
                
            # Extract Temperatures
            tempc = raw[voltc*2+ampc*2+6]
            for i in range(0,tempc):
                self.temperatures.append(utils.to_short(
                          raw[voltc*2+ampc*2+i*2+7:i*2+9+voltc*2+ampc*2])/100.0)
            logger.debug("%s temperatures: %s" % (self.__class__.__name__, 
                                        ' '.join(map(str,self.temperatures))))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Uptime: %d\nVoltages: %s\nCurrents: %s\nTemperatures: %s" % (
                self.uptime, 'V '.join(map(str,self.voltages)) + 'V', 
                'A '.join(map(str,self.currents)) + 'A',
                '℃ '.join(map(str,self.temperatures)) + ' degC')


## Horizon Message Payload - Power Status
#
#  Represents the payload of the data message 'power status'
#  @warning Data should not be modified once created
#
#  @since 0.8
#
#  @pydoc
class PowerStatus(Payload):
    """Horizon Message Payload - Power Status"""
 
    ## Create A Horizon Message Payload - Power Status
    #
    #  Constructor for the Horizon Message Payload - Power Status Class.      \n
    #  The constructor can be called two different ways:
    #  - SystemStatus(charges, raw=None, version,...)          \n
    #    Create a command message payload to send.                            \n 
    #  - SystemStatus(raw, version, timestamp)                 \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  charges        List of battery percentages
    #  @param  capacities     List of battery capacities
    #  @param  descriptions   List of tuple([present,in_use,type])
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If raw is invalid
    #
    #  @pydoc
    def __init__(self, charges = [], capacities = [], descriptions = [],
                 raw = None, timestamp = 0):
        """Create A Horizon Message Payload - Power Status"""
        
        # Class Variables
        ## Charge Measurements
        self.charges = []
        ## Capacity Measurements
        self.capacities = []
        ## Battery Descriptions
        self.descriptions = []
        
        # Create Constructor
        if raw == None:
            data = []
            
            # Verify Lengths
            if len(charges) != len(capacities) or \
                    len(capacities) != len(descriptions) or \
                    len(charges) < 1 or len(charges) > 255:
                raise ValueError("Number of batteries must be [0,255]!")

            data = utils.from_byte([len(charges)])
            
            # Verify Charges
            for c in charges:
                if c < 0 or c > 100:
                    raise ValueError("Charges must be [0,100]!")
                data += utils.from_short(int(c*100))
            
            # Verify Capacities
            for c in capacities:
                if c < 0 or c > 32000:
                    raise ValueError("Capacities must be [0,32000]!")
                data += utils.from_short(int(c))
            
            # Verify Descriptions
            for d in descriptions:
                if d[2] < 0 or (d[2] > 2 and d[2] != 8):
                    raise ValueError(
                                    "Description types must be [0|1|2|8]!")
                desc = 0xCF
                if d[0] == False:
                    desc = desc & 0x7F
                if d[1] == False:
                    desc = desc & 0xBF
                desc = desc & (0xFF | (0xFF & d[2]))
                data += utils.from_byte([desc])
            
            # Pass on to super-class
            Payload.__init__(self, raw = data, timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != raw[0]*5 + 1:
                raise ValueError("Measurement counts do not match raw data length!")

                    
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
                
            # Extract Charges
            for i in range(0,raw[0]):
                self.charges.append(
                            utils.to_short(raw[1+i*2:i*2+3])/100.0)
            logger.debug("%s charges: %s" % (self.__class__.__name__, 
                                            '% '.join(map(str,self.charges))))
                
            # Extract Capacities
            for i in range(0,raw[0]):
                self.capacities.append(utils.to_short(
                                    raw[raw[0]*2+i*2+1:i*2+3+raw[0]*2]))
            logger.debug("%s capacities: %s" % (self.__class__.__name__, 
                                        ' '.join(map(str,self.capacities))))
                
            # Extract Descriptions
            for i in range(0,raw[0]):
                desc = utils.to_byte(raw[raw[0]*4+i+1:i+2+raw[0]*4])
                self.descriptions.append(tuple([desc & 0x80 > 0, 
                                                 desc & 0x40 > 0,
                                                 desc & 0x0F]))
            logger.debug("%s descriptions: %s" % (self.__class__.__name__, 
                                        ' '.join(map(str,self.descriptions))))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Charges: %s\nCapacities: %s\nDescriptions: %s" % (
                '% '.join(map(str,self.charges)) + '%', 
                'W-Hr '.join(map(str,self.capacities)) + 'W-Hr',
                ' '.join(map(str,self.descriptions)))
              


## Horizon Message Payload - Processor Status
#
class ProcessorStatus(Payload):
    """Horizon Message Payload - Power Status"""
 
    def __init__(self, raw = None, timestamp = 0):
        """Create A Horizon Message Payload - Processor Status"""
        
        # Class Variables
        ## Charge Measurements
        self.errors = []

        # Create Constructor
        if raw != None:

            # Verify Length
            if len(raw) != raw[0] * 2 + 1:
                raise ValueError("Bad length!")
                    
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
                
            # Extract Errors
            for i in range(0,raw[0]):
                self.errors.append(
                    utils.to_short(raw[1+i*2:i*2+3]))
            logger.debug("%s errors: %s" % (self.__class__.__name__, 
                                            '  '.join(map(str, self.errors))))
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Errors: %s\n" % ' '.join(map(str, self.errors))


## Horizon Message Payload - Safety System
#
#  Represents the payload of the command and data messages 'safety system'
#  @warning Data should not be modified once created
#
#  @since 0.4
#
#
#  @pydoc
class SafetyStatus(Payload):
    """Horizon Message Payload - Safety System"""
    
    # Class Constants
    ## Emergency Stop flag mask
    EMERGENCY_STOP = 0x0001
    
    
    ## Create A Horizon Message Payload - Safety System
    #
    #  Constructor for the Horizon Message Payload - Safety System Class.     \n
    #  The constructor can be called two different ways:
    #  - SafetySystem(flags, raw=None, version, timestamp)     \n
    #    Create a command message payload to send.                            \n 
    #  - SafetySystem(raw, version, timestamp)                 \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  flags          Platform Safety System Flags
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, flags = 0x0000, raw = None, 
                 timestamp = 0):
        """Create A Horizon Message Payload - Platform Name"""
        
        # Class Variables
        ## Platform Safety System Flags
        self._flags = 0x0000
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test flags
            if flags < 0 or flags > 65535:
                raise ValueError("Invalid flags!")

            self._flags = flags
            data = utils.from_unsigned_short(flags)
            
            # Pass on to super-class
            Payload.__init__(self, raw = data, timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 2:
                raise ValueError("Bad length!")
                
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
            
            # Extract Flags
            self._flags = utils.to_unsigned_short(raw)
            logger.debug("%s flags: 0x%04X" % (self.__class__.__name__, 
                     self._flags))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Flags: 0x%04X" % self._flags
                
                
    ## Has Emergency Stop Set?
    #
    #  @return is the emergency stop platform safety system flag set
    #
    #  @pydoc
    def has_emergency_stop(self):
        """Has Emergency Stop Set?"""
        
        return (self._flags & self.EMERGENCY_STOP) == self.EMERGENCY_STOP
                
                
    ## Get Flags
    #
    #  @return the platform safety system flags
    #
    #  @pydoc
    def get_flags(self):
        """Get Flags"""
        
        return self._flags
    
    
    # Class Properties
    ## Platform Safety System Flags
    flags = property(fget=get_flags, doc="Platform Safety System Flags")



## Horizon Message Payload - Differential Speed
#
#  Represents the payload of the command and data messages 'differential speed'
#  @warning Data should not be modified once created
#
#  @pydoc
class DifferentialSpeed(Payload):
    """Horizon Message Payload - Differential Speed"""
    
    
    ## Create A Horizon Message Payload - Differential Speed
    #
    #  Constructor for the Horizon Message Payload - Differential Speed.      \n
    #  The constructor can be called two different ways:
    #  - DifferentialSpeed(l_speed, r_accel, raw=None,...)     \n
    #    Create a command message payload to send.                            \n 
    #  - DifferentialSpeed(raw, version, timestamp)            \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  left_accel     Left Acceleration (m/s^2)
    #  @param  left_speed     Left Speed (m/s)
    #  @param  right_accel    Right Acceleration (m/s^2)
    #  @param  right_speed    Right Speed (m/s)
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, left_speed = 0, right_speed = 0, left_accel = 0, right_accel = 0, 
                 raw = None, timestamp = 0):
        """Create A Horizon Message Payload - Differential Speed"""
        
        # Class Variables
        self.left_accel = 0
        self.left_speed = 0
        self.right_accel = 0
        self.right_speed = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test left speed
            if left_speed < -320 or left_speed > 320:
                raise ValueError("Left Speed must be [-320,320]!")

            self.left_speed = left_speed
            data = utils.from_short(int(left_speed * 100))
            
            # test right speed
            if right_speed < -320 or right_speed > 320:
                raise ValueError("Right Speed must be [-320,320]!")

            self.right_speed = right_speed
            data += utils.from_short(int(right_speed * 100))
            
            # test left acceleration
            if left_accel < 0 or left_accel > 320:
                raise ValueError("Left Acceleration must be [0,320]!")

            self.left_accel = left_accel
            data += utils.from_short(int(left_accel * 100))
            
            # test right acceleration
            if right_accel < 0 or right_accel > 320:
                raise ValueError("Right Acceleration must be [0,320]!")

            self.right_accel = right_accel
            data += utils.from_short(int(right_accel * 100))
            
            # Pass on to super-class
            Payload.__init__(self, raw = data, timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 8:
                raise ValueError( "Bad length!")

                
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
            
            # Extract Left Speed
            self.left_speed = utils.to_short(raw[0:2]) / 100.0
            logger.debug("%s left speed: %fm/s" % \
                         (self.__class__.__name__, self.left_speed))
            
            # Extract Right Speed
            self.right_speed = utils.to_short(raw[2:4]) / 100.0
            logger.debug("%s right speed: %fm/s" % \
                         (self.__class__.__name__, self.right_speed))
            
            # Extract Left Acceleration
            self.left_accel = utils.to_short(raw[4:6]) / 100.0
            logger.debug("%s left acceleration: %fm/s^2" % \
                         (self.__class__.__name__, self.left_accel))
            
            # Extract Right Acceleration
            self.right_accel = utils.to_short(raw[6:8]) / 100.0
            logger.debug("%s right acceleration: %fm/s^2" % \
                         (self.__class__.__name__, self.right_accel))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Left Speed: %fm/s\nRight Speed: %fm/s\n"\
               "Left Acceleration: %fm/s^2\nRight Acceleration: %fm/s^2" % (
                    self.left_speed, self.right_speed, self.left_accel, self.right_accel)



    
    

## Horizon Message Payload - Differential Control
#
#  Represents the payload of the command and data messages 'differential 
#  control'
#  @warning Data should not be modified once created
#
#
#  @pydoc
class DifferentialControl(Payload):
    """Horizon Message Payload - Differential Control"""
    
    ## Create A Horizon Message Payload - Differential Control
    #
    #  Constructor for the Horizon Message Payload - Differential Control.    \n
    #  The constructor can be called two different ways:
    #  - DifferentialControl(l_p, l_d, r_i, raw=None,...)      \n
    #    Create a command message payload to send.                            \n 
    #  - DifferentialControl(raw, version, timestamp)          \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  left_d            Left derivative constant
    #  @param  left_ffwd         Left ffwd-forward constant
    #  @param  left_i            Left integral constant
    #  @param  left_sat        Left integral sat
    #  @param  left_p            Left proportional constant
    #  @param  left_stic     Left stic compenstation
    #  @param  right_d            Right derivative constant
    #  @param  right_ffwd         Right ffwd-forward constant
    #  @param  right_i            Right integral constant
    #  @param  right_sat        Right integral sat
    #  @param  right_p            Right proportional constant
    #  @param  right_stic     Right stic compenstation
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, left_p = 0.0, left_i = 0.0, left_d = 0.0, 
                 left_ffwd = 0.0, left_stic = 0.0, left_sat = 0.0, 
                 right_p = 0.0, right_i = 0.0, right_d = 0.0, right_ffwd = 0.0, 
                 right_stic = 0.0, right_sat = 0.0,
                 raw = None, timestamp = 0):
        """Create A Horizon Message Payload - Differential Control"""
        
        # Class Variables
        self.left_p = 0.0
        self.left_i = 0.0
        self.left_d = 0.0
        self.left_ffwd = 0.0
        self.left_stic = 0.0
        self.left_sat = 0.0
        self.right_p = 0.0
        self.right_i = 0.0
        self.right_d = 0.0
        self.right_ffwd = 0.0
        self.right_stic = 0.0
        self.right_sat = 0.0

        # Create Constructor
        if raw == None:
            data = []
            if left_p < -320 or left_p > 320:
                raise ValueError("Left proportional constant must be [-320,320]!")
            self.left_p = left_p
            data = utils.from_short(int(left_p*100))

            if left_i < -320 or left_i > 320:
                raise ValueError("Left integral constant must be [-320,320]!")
            self.left_i = left_i
            data += utils.from_short(int(left_i*100))
            
            if left_d < -320 or left_d > 320:
                raise ValueError("Left derivative constant must be [-320,320]!")
            self.left_d = left_d
            data += utils.from_short(int(left_d*100))
            
            if left_ffwd < -320 or left_ffwd > 320:
                raise ValueError("Left ffwd-forward constant must be [-320,320]!")
            self.left_f = left_ffwd
            data += utils.from_short(int(left_ffwd*100))
            
            if left_stic < 0 or left_stic > 100:
                raise ValueError("Left stic compensation must be [0,100]!")
            self.left_s = left_stic
            data += utils.from_short(int(left_stic*100))
            
            if left_sat < 0 or left_sat > 100:
                raise ValueError("Left integral sat must be [0,100]!")
            self.left_l = left_sat
            data += utils.from_short(int(left_sat*100))
            
            if right_p < -320 or right_p > 320:
                raise ValueError("Right proportional constant must be [-320,320]!")
            self.right_p = right_p
            data += utils.from_short(int(right_p*100))
            
            if right_i < -320 or right_i > 320:
                raise ValueError("Right integral constant must be [-320,320]!")
            self.right_i = right_i
            data += utils.from_short(int(right_i*100))
            
            if right_d < -320 or right_d > 320:
                raise ValueError("Right derivative constant must be [-320,320]!")
            self.right_d = right_d
            data += utils.from_short(int(right_d*100))
            
            if right_ffwd < -320 or right_ffwd > 320:
                raise ValueError("Right ffwd-forward constant must be [-320,320]!")
            self.right_ffwd = right_ffwd
            data += utils.from_short(int(right_ffwd*100))
            
            if right_stic < 0 or right_stic > 100:
                raise ValueError("Right stic compensation must be [0,100]!")
            self.right_stic = right_stic
            data += utils.from_short(int(right_stic*100))
            
            if right_sat < 0 or right_sat > 100:
                raise ValueError("Right integral sat must be [0,100]!")
            self.right_sat = right_sat
            data += utils.from_short(int(right_sat*100))
            
            # Pass on to super-class
            Payload.__init__(self, raw = data, timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 24:
                raise ValueError( "Bad length!")

            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
            self.left_p = utils.to_short(raw[0:2]) / 100.0
            self.left_i = utils.to_short(raw[2:4]) / 100.0
            self.left_d = utils.to_short(raw[4:6]) / 100.0
            self.left_ffwd = utils.to_short(raw[6:8]) / 100.0
            self.left_stic = utils.to_short(raw[8:10]) / 100.0
            self.left_sat = utils.to_short(raw[10:12]) / 100.0
            self.right_p = utils.to_short(raw[12:14]) / 100.0
            self.right_i = utils.to_short(raw[14:16]) / 100.0
            self.right_d = utils.to_short(raw[16:18]) / 100.0
            self.right_ffwd = utils.to_short(raw[18:20]) / 100.0
            self.right_stic = utils.to_short(raw[20:22]) / 100.0
            self.right_sat = utils.to_short(raw[22:24]) / 100.0

    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Left P: %f\nLeft I: %f\nLeft D: %f\nLeft feed-forward: %f\n"\
               "Left stic compensation: %f\nLeft integral sat: %f\n"\
               "Right P: %f\nRight I: %f\nRight D: %f\nRight feed-forward: %f"\
               "\nRight stic compensation: %f\nRight integral sat: %f\n"\
                % (
            self.left_p, self.left_i, self.left_d, self.left_ffwd, self.left_stic, self.left_sat,
            self.right_p, self.right_i, self.right_d, self.right_ffwd, self.right_stic, self.right_sat)
   

class DifferentialCurrentControl(DifferentialControl):
    pass



## Horizon Message Payload - Differential Output
#
#  Represents the payload of the command and data messages 'differential motors'
#  @warning Data should not be modified once created
#
#  @section data Differential Motors Data
#  @copydoc differential_motors
#
#  @pydoc
class DifferentialOutput(Payload):
    """Horizon Message Payload - Differential Motors"""
    
    
    ## Create A Horizon Message Payload - Differential Motors
    #
    #  Constructor for the Horizon Message Payload - Differential Speed.      \n
    #  The constructor can be called two different ways:
    #  - DifferentialMotors(left, right, raw=None,...)         \n
    #    Create a command message payload to send.                            \n 
    #  - DifferentialMotors(raw, version, timestamp)           \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  left           Left Motor Output
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  right          Right Motor Output
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, left = 0, right = 0, raw = None, 
                 timestamp = 0):
        """Create A Horizon Message Payload - Differential Motors"""
        
        # Class Variables
        self.left = 0
        self.right = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test left
            if left < -100 or left > 100:
                raise ValueError("Left Motor must be [-100,100]!")

            self.left = left
            data = utils.from_short(int(left*100))
            
            # test right
            if right < -100 or right > 100:
                raise ValueError("Right Motor must be [-100,100]!")

            self.right = right
            data += utils.from_short(int(right*100))
            
            # Pass on to super-class
            Payload.__init__(self, raw = data, timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 4:
                raise ValueError( "Bad length!")

                
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
            
            # Extract Left
            self.left = utils.to_short(raw[0:2])/100.0
            logger.debug("%s left motor: %f%%" % \
                         (self.__class__.__name__, self.left))
            
            # Extract Right
            self.right = utils.to_short(raw[2:4])/100.0
            logger.debug("%s right motor: %f%%" % \
                         (self.__class__.__name__, self.right))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Left Motor: %f%%\nRight Motor: %f%%" % (self.left, self.right)


class DifferentialCurrent(DifferentialOutput):
    pass

## Horizon Message Payload - Ackermann Output
#
#  Represents the payload of the command and data messages 'ackermann servos'
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#
#  @pydoc
class AckermannOutput(Payload):
    """Horizon Message Payload - Ackermann Servos"""
    
    
    ## Create A Horizon Message Payload - Ackermann Servos
    #
    #  Constructor for the Horizon Message Payload - Ackermann Servos.        \n
    #  The constructor can be called two different ways:
    #  - AckermannServos(steering, throttle, raw=None,...)     \n
    #    Create a command message payload to send.                            \n 
    #  - AckermannServos(raw, version, timestamp)              \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  brake          The brake position
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  steering       The steering position
    #  @param  throttle       The throttle position
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, steering = 0, throttle = 0, brake = 0, 
                 raw = None, timestamp = 0):
        """Create A Horizon Message Payload - Ackermann Servos"""
        
        # Class Variables
        ## Brake
        self.brake = 0
        ## Steering
        self.steering = 0
        ## Throttle
        self.throttle = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test steering
            if steering < -100 or steering > 100:
                raise ValueError("Steering must be [-100,100]!")

            self.steering = steering
            data += utils.from_short(int(steering*100))
            
            # test throttle
            if throttle < -100 or throttle > 100:
                raise ValueError("Throttle must be [-100,100]!")

            self.throttle = throttle
            data += utils.from_short(int(throttle*100))
            
            # test brake
            if brake < 0 or brake > 100:
                raise ValueError("Brake must be [0,100]!")

            self.brake = brake
            data += utils.from_short(int(brake*100))
            
            # Pass on to super-class
            Payload.__init__(self, raw = data, timestamp = timestamp)
        
        # Parse Constructor
        else:

            # Verify Length
            if len(raw) != 6:
                raise ValueError( "Bad length!")

            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
            
            # Extract Steering
            self.steering = utils.to_short(raw[0:2]) / 100.0
            logger.debug("%s steering: %f%%" % \
                         (self.__class__.__name__, self.steering))
            
            # Extract Throttle
            self.throttle = utils.to_short(raw[2:4]) / 100.0
            logger.debug("%s throttle: %f%%" % \
                         (self.__class__.__name__, self.throttle))
            
            # Extract Brake
            self.brake = utils.to_short(raw[4:6]) / 100.0
            logger.debug("%s brake: %f%%" % \
                         (self.__class__.__name__, self.brake))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Steering: %f%%\nThrottle: %f%%\nBrake: %f%%" % (
                                    self.steering, self.throttle, self.brake)


## Horizon Message Payload - Velocity
#
#  Represents the payload of the command and data messages 'velocity'
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#
#  @pydoc
class Velocity(Payload):
    """Horizon Message Payload - Velocity"""
    
    
    ## Create A Horizon Message Payload - Velocity
    #
    #  Constructor for the Horizon Message Payload - Velocity.                \n
    #  The constructor can be called two different ways:
    #  - Velocity(trans, rot, accel, raw=None, ...)            \n
    #    Create a command message payload to send.                            \n 
    #  - Velocity(raw, version, timestamp)                     \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  accel          The desired translational acceleration (m/s^2)
    #  @param  flags          0x02 - Automatic Transmission, 0x01 - Dynamic
    #                         Compensation, 0x03 - Both, 0x00 - None.
    #                         DEPRECATED: not in Horizon as of v0.4.
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  rot            The desired rotational speed (rad/s)
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  trans          The desired translational speed (m/s)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, trans = 0, rot = 0, accel = 0, 
                 raw = None, timestamp = 0):
        """Create A Horizon Message Payload - Velocity"""
        
        # Class Variables
        ## Translational Acceleration
        self.accel = 0
        ## Rotational speed
        self.rot = 0
        ## Translational speed
        self.trans = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test translation
            if trans < -320 or trans > 320:
                raise ValueError("Translational Speed must be [-320,320]!")

            self.trans = trans
            data += utils.from_short(int(trans*100))
            
            # test rotation
            if rot < -320 or rot > 320:
                raise ValueError("Rotational Speed must be [-320,320]!")

            self.rot = rot
            data += utils.from_short(int(rot*100))
            
            # test acceleration
            if accel < 0 or accel > 320:
                raise ValueError("Translational Acceleration must be [0,320]!")

            self.accel = accel
            data += utils.from_short(int(accel*100))
            
            # Pass on to super-class
            Payload.__init__(self, raw = data, timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if (len(raw) != 6):
                raise ValueError( "Bad length!")

                
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
            
            # Extract Translational
            self.trans = utils.to_short(raw[0:2]) / 100.0
            logger.debug("%s: translational speed: %fm/s" % \
                         (self.__class__.__name__, self.trans))
            
            # Extract Rotational
            self.rot = utils.to_short(raw[2:4]) / 100.0
            logger.debug("%s: rotational speed: %frad/s" % \
                         (self.__class__.__name__, self.rot))
            
            # Extract Acceleration
            self.accel = utils.to_short(raw[4:6]) / 100.0
            logger.debug("%s: translational acceleration: %fm/s^2" % \
                         (self.__class__.__name__, self.accel))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Translational Speed: %fm/s\nRotational Speed: %frad/s\n"\
               "Translational Acceleration: %fm/s^2" % (self.trans, 
                                                        self.rot, self.accel)


## Horizon Message Payload - Turn
#
#  Represents the payload of the command and data messages 'turn'
#  @warning Data should not be modified once created
#
#  @since 0.3
#
#
#  @pydoc
class Turn(Payload):
    """Horizon Message Payload - Turn"""
    
    
    ## Create A Horizon Message Payload - Turn
    #
    #  Constructor for the Horizon Message Payload - Turn.                    \n
    #  The constructor can be called two different ways:
    #  - Turn(trans, turn, accel, raw=None, ...)               \n
    #    Create a command message payload to send.                            \n 
    #  - Turn(raw, version, timestamp)                         \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  accel          The desired translational acceleration (m/s^2)
    #  @param  flags          0x02 - Automatic Transmission, 0x01 - Dynamic
    #                         Compensation, 0x03 - Both, 0x00 - None.
    #                         DEPRECATED: not in Horizon as of v0.4.
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  trans          The desired translational speed (m/s)
    #  @param  turn           The desired turn radius
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, trans = 0, rad = 0, accel = 0, 
                 raw = None, timestamp = 0):
        """Create A Horizon Message Payload - Turn"""
        
        # Class Variables
        self.accel = 0
        self.rad = 0
        self.trans = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test translation
            if trans < -320 or trans > 320:
                raise ValueError("Translational Speed must be [-320,320]!")

            self.trans = trans
            data += utils.from_short(int(trans*100))
            
            # test turn
            if rad < -320 or rad > 320:
                raise ValueError("Turn Radius must be [-320,320]!")

            self.rad = rad
            data += utils.from_short(int(rad*100))
            
            # test acceleration
            if accel < 0 or accel > 320:
                raise ValueError("Translational Acceleration must be [0,320]!")

            self.accel = accel
            data += utils.from_short(int(accel*100))
            
            # Pass on to super-class
            Payload.__init__(self, raw=data, timestamp=timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if (len(raw) != 6):
                raise ValueError( "Bad length!")

                
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
            
            # Extract Translational
            self.trans = utils.to_short(raw[0:2]) / 100.0
            logger.debug("%s translational speed: %fm/s" % \
                         (self.__class__.__name__, self.trans))
            
            # Extract Turn Radius
            self.rad = utils.to_short(raw[2:4]) / 100.0
            logger.debug("%s turn radius: %fm" % \
                         (self.__class__.__name__, self.rad))
            
            # Extract Acceleration
            self.accel = utils.to_short(raw[4:6]) / 100.0
            logger.debug("%s translational acceleration: %fm/s^2" % \
                         (self.__class__.__name__, self.accel))
    
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Translational Speed: %fm/s\nTurn Radius: %fm\n"\
               "Translational Acceleration: %fm/s^2" % (self.trans, 
                                                        self.rad, self.accel)


## Horizon Message Payload - Max Speed
#
#  Represents the payload of the command and data messages 'max speed'
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#
#  @pydoc
class MaxSpeed(Payload):
    """Horizon Message Payload - Max Speed"""
    
    ## Create A Horizon Message Payload - Max Speed
    #
    #  Constructor for the Horizon Message Payload - Max Speed.               \n
    #  The constructor can be called two different ways:
    #  - MaxSpeed(forward, reverse, raw=None, ...)             \n
    #    Create a command message payload to send.                            \n 
    #  - MaxSpeed(raw, version, timestamp)                     \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  forward        The maximum forward speed
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  reverse        The maximum reverse speed
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, forward = 0, reverse = 0, raw= None, 
                 timestamp = 0):
        """Create A Horizon Message Payload - Max Speed"""
        
        # Class Variables
        ## Max Forward Speed
        self.forward = 0
        ## Max Reverse Speed
        self.reverse = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test forward
            if forward < 0 or forward > 320:
                raise ValueError("Forward Speed must be [0,320]!")

            self.forward = forward
            data = utils.from_short(int(forward * 100))
            
            # test reverse
            if reverse < 0 or reverse > 320:
                raise ValueError("Reverse Speed must be [0,320]!")

            self.reverse = reverse
            data += utils.from_short(int(reverse * 100))
            
            # Pass on to super-class
            Payload.__init__(self, raw = data, timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 4:
                raise ValueError("Bad length!")

                
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
            
            # Extract Forward
            self.forward = utils.to_short(raw[0:2]) / 100.0
            logger.debug("%s forward speed: %fm/s" % \
                         (self.__class__.__name__, self.forward))
            
            # Extract Reverse
            self.reverse = utils.to_short(raw[2:4]) / 100.0
            logger.debug("%s reverse speed: %fm/s" % \
                         (self.__class__.__name__, self.reverse))
    
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Max Forward Speed: %fm/s\nMax Reverse Speed: %fm/s"\
               % (self.forward, self.reverse)
                

## Horizon Message Payload - Max Acceleration
#
#  Represents the payload of the command and data messages 'max acceleration'
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @pydoc
class MaxAccel(Payload):
    """Horizon Message Payload - Max Acceleration"""
    
    
    ## Create A Horizon Message Payload - Max Acceleration
    #
    #  Constructor for the Horizon Message Payload - Max Acceleration.        \n
    #  The constructor can be called two different ways:
    #  - MaxAcceleration(forward, reverse, raw=None, ...)      \n
    #    Create a command message payload to send.                            \n 
    #  - MaxAcceleration(raw, version, timestamp)              \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  forward        The maximum forward acceleration
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  reverse        The maximum reverse acceleration
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, forward = 0, reverse = 0, raw= None, 
                 timestamp = 0):
        """Create A Horizon Message Payload - Max Acceleration"""
        
        # Class Variables
        self.forward = 0
        self.reverse = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test forward
            if forward < 0 or forward > 320:
                raise ValueError("Forward Acceleration must be [0, 320]!")

            self.forward = forward
            data = utils.from_short(int(forward * 100))
            
            # test reverse
            if reverse < 0 or reverse > 320:
                raise ValueError("Reverse Acceleration must be [0, 320]!")

            self.reverse = reverse
            data += utils.from_short(int(reverse * 100))
            
            # Pass on to super-class
            Payload.__init__(self, raw = data, 
                                                          timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 4:
                raise ValueError( "Bad length!")

                
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
            
            # Extract Forward
            self.forward = utils.to_short(raw[0:2]) / 100.0
            logger.debug("%s forward acceleration: %fm/s^2" % \
                         (self.__class__.__name__, self.forward))
            
            # Extract Reverse
            self.reverse = utils.to_short(raw[2:4]) / 100.0
            logger.debug("%s reverse acceleration: %fm/s^2" % \
                         (self.__class__.__name__, self.reverse))
    
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Max Forward Acceleration: %fm/s^2\n"\
                "Max Reverse Acceleration: %fm/s^2"\
               % (self.forward, self.reverse)


## Horizon Message Payload - Gear
#
#  Represents the payload of the command message 'gear'
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @pydoc
class Gear(Payload):
    """Horizon Message Payload - Gear"""
    
    
    
    ## Create A Horizon Message Payload - Gear
    #
    #  Constructor for the Horizon Message Payload - Gear.                    \n
    #  The constructor can be called two different ways:
    #  - Gear(gear, raw=None,...)                              \n
    #    Create a command message payload to send.                            \n 
    #  - Gear(raw, version, timestamp)                         \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  gear           The desired gear [-128,127]
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, gear = -1,
                 raw = None, 
                 timestamp = 0):
        """Create A Horizon Message Payload - Gear"""
        
        # Class Variables
        ## Gear
        self._gear = -1
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test Gear
            if gear < -128 or gear > 127:
                raise ValueError("Gear must be [-128,127]!")

            self._gear = gear
            data += utils.from_char(gear)
            
            # Pass on to super-class
            Payload.__init__(self, raw = data, timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 1:
                raise ValueError( "Bad length!")

                
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
            
            # Extract Gear
            self._gear = utils.to_char(raw[0:1])
            logger.debug("%s gear: %d" % \
                         (self.__class__.__name__, self._gear))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Gear: %d" % (self._gear)
                
                
    ## Get Gear
    #
    #  @return the gear
    #
    #  @pydoc
    def get_gear(self):
        """Get Gear"""
        
        return self._gear
    
    
    # Class Properties
    ## Gear
    gear = property(fget=get_gear, doc="Gear")


## Horizon Message Payload - Gear Status
#
#  Represents the payload of the data message 'gear status'
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @pydoc
class GearStatus(Payload):
    """Horizon Message Payload - Gear Status"""
    
    
    ## Create A Horizon Message Payload - Gear Status
    #
    #  Constructor for the Horizon Message Payload - Gear.                    \n
    #  The constructor can be called two different ways:
    #  - GearStatus(upshifting, downshifting,gear,raw=None,...)\n
    #    Create a command message payload to send.                            \n 
    #  - GearStatus(raw, version, timestamp)                   \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  downshifting   Set the downshifting flag?
    #  @param  gear           The desired gear [-128,127]
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  upshifting     Set the upshifting flag?
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, downshifting = False, upshifting = False, gear = -1,
                 raw = None, 
                 timestamp = 0):
        """Create A Horizon Message Payload - Gear Status"""
        
        # Class Variables
        ## Downshifting
        self._down = False
        ## Gear
        self._gear = -1
        ## Upshifting
        self._up = False
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test flags
            flags = 0x0
            if downshifting and upshifting:
                raise ValueError("Cannot downshift and upshift!")

            self._down = downshifting
            self._up = upshifting
            if self._down: flags &= 0x01
            if self._up: flags &= 0x02
            data += utils.from_byte(flags)
            
            # test Gear
            if gear < -128 or gear > 127:
                raise ValueError("Gear must be [-128,127]!")

            self._gear = gear
            data += utils.from_char(gear)
            
            # Pass on to super-class
            Payload.__init__(self, raw = data, timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 2:
                raise ValueError( "Bad length!")

                
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
            
            # Extract Flags
            flags = utils.to_byte(raw[0:1])
            self._down = (flags & 0x01) == 0x01
            self._up = (flags & 0x02) == 0x02
            logger.debug("%s downshifting: %s" % \
                         (self.__class__.__name__, str(self._down)))
            logger.debug("%s upshifting: %s" % \
                         (self.__class__.__name__, str(self._up)))
            
            # Extract Gear
            self._gear = utils.to_char(raw[1:2])
            logger.debug("%s gear: %d" % \
                         (self.__class__.__name__, self._gear))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Downshifting: %s\nUpshifting: %s\nGear: %d" % (
                                    str(self._down), str(self._up), self._gear)
                
                
    ## Is Downshifting?
    #
    #  @return is the platform downshifting?
    #
    #  @pydoc
    def is_downshifting(self):
        """Is Downshifting?"""
        
        return self._down
                
                
    ## Is Upshifting?
    #
    #  @return is the platform upshifting?
    #
    #  @pydoc
    def is_upshifting(self):
        """Is Upshifting?"""
        
        return self._up
                
                
    ## Get Gear
    #
    #  @return the gear
    #
    #  @pydoc
    def get_gear(self):
        """Get Gear"""
        
        return self._gear
    
    
    # Class Properties
    ## Downshifting
    downshifting = property(fget=is_downshifting, doc="Downshifting")
    ## Upshifting
    upshifting = property(fget=is_upshifting, doc="Upshifting")
    ## Gear
    gear = property(fget=get_gear, doc="Gear")




## Horizon Message Payload - Distance
#
#  Represents the payload of the data message 'distance'
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @pydoc
class Distance(Payload):
    """Horizon Message Payload - Distance"""
    
    
    ## Create A Horizon Message Payload - Distance
    #
    #  Constructor for the Horizon Message Payload - Distance.                \n
    #  The constructor can be called two different ways:
    #  - Distance(distance, raw=None, timestamp, version, ...) \n
    #    Create a command message payload to send.                            \n 
    #  - Distance(raw, version, timestamp)                     \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  distance       List of distances scaled to two bytes
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, distance = [], 
                 raw = None, timestamp = 0):
        """Create A Horizon Message Payload - Distance"""
        
        # Class Variables
        self.distances = []
        
        # Parse Constructor
        if raw != None:
            # Verify Length
            if len(raw) < 1 or len(raw) != raw[0]*2+1:
                raise ValueError( "Bad length!")

                
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
            
            # Extract Distances
            for i in range(0, raw[0]):
                self.distances += [utils.to_short(raw[i * 2 + 1:i * 2 + 3])]
            logger.debug("%s distances: %s" % (self.__class__.__name__, 
                     ' '.join(map(str,self.distances))))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Distances: %s" % ''.join([string.rjust(str(s), 6) for s in self.distances])
        

## Horizon Message Payload - Distance & Timing
#
#  Represents the payload of the data message 'distance timing'
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#
#  @pydoc
class DistanceTiming(Payload):
    """Horizon Message Payload - Distance & Timing"""
    
    
    ## Create A Horizon Message Payload - Distance & Timing
    #
    #  Constructor for the Horizon Message Payload - Distance & Timing.       \n
    #  The constructor can be called two different ways:
    #  - DistanceTiming(distance, timing, raw=None, ...)       \n
    #    Create a command message payload to send.                            \n 
    #  - DistanceTiming(raw, version, timestamp)               \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  distance       List of distances scaled to two bytes
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  timing         List of distance aquisition times (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, distance = [], timing = [], 
                 raw = None, timestamp = 0):
        """Create A Horizon Message Payload - Distance & Timing"""
        
        # Class Variables
        self.distances = []
        self.timings = []
        
        # Parse Constructor
        if raw != None:

            # Verify Length
            if len(raw) < 1 or len(raw) != raw[0] * 6 + 1:
                raise ValueError( "Bad length!")

                
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
            
            # Extract Distances
            for i in range(0, raw[0]):
                self.distances += [utils.to_short(raw[i*2+1:i*2+3])]
            logger.debug("%s distances: %s" % (self.__class__.__name__, 
                     ' '.join(map(hex,self.distances))))
            
            # Extract Timing
            for i in range(0,raw[0]):
                self.timings += [utils.to_unsigned_int(
                                            raw[raw[0]*2+i*4+1:i*4+5+raw[0]*2])]
            logger.debug("%s times: %s" % (self.__class__.__name__, 
                     ' '.join(map(str,self.timings))))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Distances: %s\nTiming:    %s" % (''.join([string.rjust(str(s), 6) for s in self.distances]),
                                              ''.join([string.rjust(str(s), 6) for s in self.timings]))
        

## Horizon Message Payload - Orientation
#
#  Represents the payload of the data message 'orientation'
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @pydoc
class Orientation(Payload):
    """Horizon Message Payload - Orientation"""
    
    ## Create A Horizon Message Payload - Orientation
    #
    #  Constructor for the Horizon Message Payload - Orientation.             \n
    #  The constructor can be called two different ways:
    #  - Orientation(roll, pitch, yaw, raw=None, ...)          \n
    #    Create a command message payload to send.                            \n 
    #  - Orientation(raw, version, timestamp)                  \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  pitch          The vehicle's pitch angle
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  roll           The vehicle's roll angle
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @param  yaw            The vehicle's yaw (magnetic) angle
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, roll = 0, pitch = 0, yaw = 0, 
                 raw = None, timestamp = 0):
        """Create A Horizon Message Payload - Orientation"""
        
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            if roll < -math.pi or roll > math.pi:
                raise ValueError( "Roll must be [-π,π]!")
            self.roll = roll
            data += utils.from_short(int(roll*1000))
            
            if pitch < -math.pi or pitch > math.pi:
                raise ValueError( "Pitch must be [-π,π]!")
            self.pitch = pitch
            data += utils.from_short(int(pitch*1000))
            
            if yaw < -math.pi or yaw > math.pi:
                raise ValueError( "Yaw must be [-π,π]!")
            self.yaw = yaw
            data += utils.from_short(int(yaw*1000))
            
            Payload.__init__(self, raw = data, timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 6:
                raise ValueError( "Bad length!")

            Payload.__init__(self, raw = raw, timestamp = timestamp)
            self.roll = utils.to_short(raw[0:2]) / 1000.0
            self.pitch = utils.to_short(raw[2:4]) / 1000.0
            self.yaw = utils.to_short(raw[4:6]) / 1000.0

    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Roll: %f\nPitch: %f\nYaw: %f" % (
                                        self.roll, self.pitch, self.yaw)
        
        
    
    
## Horizon Message Payload - Rotation
#
#  Represents the payload of the data message 'rotational rate'
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#
#  @pydoc
class Rotation(Payload):
    """Horizon Message Payload - Rotation"""
    
    
    ## Create A Horizon Message Payload - Rotation
    #
    #  Constructor for the Horizon Message Payload - Rotation.                \n
    #  The constructor can be called two different ways:
    #  - Rotation(roll, pitch, yaw, raw=None, ...)             \n
    #    Create a command message payload to send.                            \n 
    #  - Rotation(raw, version, timestamp)                     \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  rot_pitch      The vehicle's pitch rotation rate
    #  @param  rot_roll       The vehicle's roll rotation rate
    #  @param  rot_yaw        The vehicle's yaw rotation rate
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, roll = 0, pitch = 0, yaw = 0, 
                 raw = None, timestamp = 0):
        """Create A Horizon Message Payload - Rotation"""
        
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test roll
            if roll < -10*math.pi or roll > 10*math.pi:
                raise ValueError( "Roll must be [-10π,10π]!")

            self.roll = roll
            data += utils.from_short(int(roll*1000))
            
            # test pitch
            if pitch < -10*math.pi or pitch > 10*math.pi:
                raise ValueError( "Pitch must be [-10π,10π]!")

            self.pitch = pitch
            data += utils.from_short(int(pitch*1000))
            
            # test yaw
            if yaw < -10*math.pi or yaw > 10*math.pi:
                raise ValueError( "Yaw must be [-10π,10π]!")

            self.yaw = yaw
            data += utils.from_short(int(yaw*1000))
            
            # Pass on to super-class
            Payload.__init__(self, raw = data, timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 6:
                raise ValueError( "Bad length!")
                
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
            self.roll = utils.to_short(raw[0:2])/1000.0
            self.pitch = utils.to_short(raw[2:4])/1000.0
            self.yaw = utils.to_short(raw[4:6])/1000.0
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Rotational Roll: %frad/s\nRotational Pitch: %frad/s\n"\
               "Rotational Yaw: %frad/s" % (
                          self.roll, self.pitch, self.yaw)
        
        
    

class Acceleration(Payload):
    """Horizon Message Payload - Acceleration"""
    
    ## Create A Horizon Message Payload - Acceleration
    def __init__(self, x = 0, y = 0, z = 0, 
                 raw = None, timestamp = 0):
        """Create A Horizon Message Payload - Acceleration"""
        
        # Class Variables
        self.x = 0
        self.y = 0
        self.z = 0

        # Verify Length
        if raw == None or len(raw) != 6:
            raise ValueError( "Bad length!")
                
        # Pass on to super-class
        Payload.__init__(self, raw = raw, timestamp = timestamp)
            
        # Extract X
        self.x = utils.to_short(raw[0:2]) / 1.0
        logger.debug("%s x: %f" % (self.__class__.__name__, self.x))
            
        # Extract Y
        self.y = utils.to_short(raw[2:4]) / 1.0
        logger.debug("%s y: %f" % (self.__class__.__name__, self.y))
            
        # Extract Z
        self.z = utils.to_short(raw[4:6]) / 1.0
        logger.debug("%s z: %f" % (self.__class__.__name__, self.z))
 
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "X: %f\nY: %f\nZ: %f" % (self.x, self.y, self.z)
        

 
class Magnetometer(Payload):
    """Horizon Message Payload - Magnetometer"""
    
    ## Create A Horizon Message Payload - Magnetometer
    def __init__(self, x = 0, y = 0, z = 0, 
                 raw = None, timestamp = 0):
        """Create A Horizon Message Payload - Magnetometer Data"""
        
        # Class Variables
        self.x = 0
        self.y = 0
        self.z = 0 

        # Verify Length
        if raw == None or len(raw) != 6:
            raise ValueError( "Bad length!")
                
        # Pass on to super-class
        Payload.__init__(self, raw = raw, timestamp = timestamp)
            
        # Extract X
        self.x = utils.to_short(raw[0:2]) / 1.0
        logger.debug("%s x: %f" % (self.__class__.__name__, self.x))
            
        # Extract Y
        self.y = utils.to_short(raw[2:4]) / 1.0
        logger.debug("%s y: %f" % (self.__class__.__name__, self.y))
            
        # Extract Z
        self.z = utils.to_short(raw[4:6]) / 1.0
        logger.debug("%s z: %f" % (self.__class__.__name__, self.z))
 
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "X: %f\nY: %f\nZ: %f" % (self.x, self.y, self.z)
 

## Horizon Message Payload - Encoders
#
#  Represents the payload of the data message 'encoders'
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @pydoc
class Encoders(Payload):
    """Horizon Message Payload - Encoders"""
    
    
    ## Create A Horizon Message Payload - Encoders
    #
    #  Constructor for the Horizon Message Payload - Encoders.                \n
    #  The constructor can be called two different ways:
    #  - Encoders(travel, speed, raw=None, ...)                \n
    #    Create a command message payload to send.                            \n 
    #  - Encoders(raw, version, timestamp)                     \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  speed          A list of encoder speeds
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  travel         A list of encoder distances
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, travel = [], speed = [], raw = None, 
                 timestamp = 0):
        """Create A Horizon Message Payload - Encoders"""
        
        # Class Variables
        ## Encoder Speeds
        self._speed = []
        ## Encoder Distances
        self._travel = []
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test length
            if len(travel) != len(speed):
                raise ValueError("Must have same number of distances and speeds!")

            if len(travel) > 255:
                raise ValueError("Must have 0-255 encoders!")

            data = utils.from_byte(len(speed))
            
            # test distances
            for t in travel:
                if t < -32 or t > 32:
                    raise ValueError("Distances must be [-2x10^6,2x10^6]!")

                data += utils.from_int(int(t*1000))
            self._travel = travel
            
            # test speeds
            for s in speed:
                if s < -32 or s > 32:
                    raise ValueError( "Speeds must be [-32,32]!")

                data += utils.from_short(int(s*1000))
            self._speed = speed
            
            # Pass on to super-class
            Payload.__init__(self, raw = data, timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) < 1 or len(raw) != raw[0]*6+1:
                raise ValueError( "Bad length!")

                
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
            
            # Extract Distances
            self._travel = []
            for i in range(0,raw[0]):
                self._travel.append(utils.to_int(
                                                raw[i*4+1:(i+1)*4+1])/1000.0)
            logger.debug("%s distances: %s" % (self.__class__.__name__, 
                     self._travel))
            
            # Extract Speeds
            self._speed = []
            for i in range(0,raw[0]):
                self._speed.append(utils.to_short(\
                                raw[i*2+raw[0]*4+1:(i+1)*2+raw[0]*4+1])/1000.0)
            logger.debug("%s speeds: %s" % (self.__class__.__name__, 
                     self._speed))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Distances: %s\nSpeeds: %s" % (\
                                        'm, '.join(map(str,self._travel))+'m', 
                                       'm/s, '.join(map(str,self._speed))+'m/s')
        
        
    ## Get the number of Encoders
    #
    #  @return number of encoders
    #
    #  @pydoc
    def get_count(self):
        """Get the number of Encoders."""
        
        return len(self._speed)
                
                
    ## Get Speed
    #
    #  @param  encoder  The encoder to get the speed of.
    #                   If -1 then it returns a list of all encoders
    #  @return the encoder speed (m/s), or all encoder speeds
    #
    #  @pydoc
    def get_speed(self, encoder = -1):
        """Get Encoder"""
        
        # all encoders
        if encoder < 0 or encoder >= len(self._speed):
            return self._speed
        
        # single encoder
        else:
            return self._speed[encoder]
                
                
    ## Get Distance
    #
    #  @param  encoder  The encoder to get the distance of.
    #                   If -1 then it returns a list of all encoders
    #  @return the distance of encoder (metres), or all encoder distances
    #
    #  @pydoc
    def get_travel(self, encoder = -1):
        """Get Distance"""
        
        # all encoders
        if encoder < 0 or encoder >= len(self._speed):
            return self._travel
        
        # single encoder
        else:
            return self._travel[encoder]
    
    
    # Class Properties
    ## Encoder Speed
    speed = property(fget=get_speed, doc="Encoder Speed")
    ## Encoder Distance
    travel = property(fget=get_travel, doc="Encoder Distance")


## Horizon Message Payload - Raw Encoders
#
#  Represents the payload of the data message 'raw encoders'
#
#  @warning Data should not be modified once created
#
#  @since 0.8
#
#  @pydoc
class RawEncoders(Payload):
    """Horizon Message Payload - Raw Encoders"""
    
    
    
    ## Create A Horizon Message Payload - Raw Encoders
    #
    #  Constructor for the Horizon Message Payload - Raw Encoders.            \n
    #  The constructor can be called two different ways:
    #  - Encoders(ticks, raw=None, ...)                        \n
    #    Create a command message payload to send.                            \n 
    #  - Encoders(raw, version, timestamp)                     \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  ticks          A list of encoder ticks
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, ticks = [], raw = None, 
                 timestamp = 0):
        """Create A Horizon Message Payload - Raw Encoders"""
        
        self.ticks = []
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test length
            if len(ticks) > 255:
                raise ValueError("Must have 0-255 encoders!")

            data = utils.from_byte([len(ticks)])
            
            # test ticks
            for t in ticks:
                if t < -math.pow(2, 31) or t > math.pow(2, 31)-1:
                    raise ValueError( "Ticks must be [-2^31,2^31-1]!")

                data += utils.from_int(int(t))
            self.ticks = ticks
            
            # Pass on to super-class
            Payload.__init__(self, raw = data, timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) < 1 or len(raw) != raw[0] * 4 + 1:
                raise ValueError( "Bad length!")

            Payload.__init__(self, raw = raw, timestamp = timestamp)
            
            # Extract Ticks
            self.ticks = []
            for i in range(0,raw[0]):
                self.ticks.append(utils.to_int(\
                                raw[i*4+1:(i+1)*4+1]))
            logger.debug("%s ticks: %s" % (self.__class__.__name__, 
                     self.ticks))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Ticks: %s" % (' '.join(map(str,self.ticks)))
        


## Horizon Message Payload - Reset
#
#  Represents the payload of the command message 'reset'
#
#  @warning Data should not be modified once created
#
#  @since 0.7
#
#  @pydoc
class Reset(Payload):
    """Horizon Message Payload - Reset"""
    
    ## Create A Horizon Message Payload - Reset
    #
    #  Constructor for the Horizon Message Payload - Reset Class.             \n
    #  The constructor can be called two different ways:
    #  - PlatformTime(raw=None, version, timestamp)            \n
    #    Create a command message payload to send.                            \n 
    #  - PlatformTime(raw, version, timestamp)                 \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, raw = None, 
                 timestamp = 0):
        """Create A Horizon Message Payload - Reset"""
        
        # Class Variables
        ## Passcode
        self.passcode = 0x3A18
        
        # Create Constructor
        if raw == None:
            data = []
            
            # passcode
            data += utils.from_unsigned_short(self.passcode)
            
            # Pass on to super-class
            Payload.__init__(self, raw = data, timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 2:
                raise ValueError("Bad length!")

                
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
            
            # Extract Passcode
            self.passcode = utils.to_unsigned_short(raw)
            logger.debug("%s passcode: %d" % (self.__class__.__name__, self.passcode))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Passcode: 0x%04X" % self.passcode
                

# Abstract class. This forms the basis for all the sensor config messages, as they are all
# sequences of pairs of offset/scale values.
class VariableSensorConfig(Payload):
    """Horizon Message Abstract Payload - Sensor Configuration"""

    def __init__(self, passcode = 0, offsets = [], scales = [], 
                 raw = None, timestamp = 0):
        
        # Class Variables
        self.passcode = passcode
        self.offsets = []
        self.scales = []
        
        # Create Constructor
        if raw == None:
            # Assume this is a config set going out.

            # Magic passcode
            data = utils.from_short(self.passcode)
            
            self.offsets = offsets
            self.scales = scales
            if len(self.offsets) != len(self.scales):
                raise ValueError("Offsets and scales are not the same length!")
        
            for offset, scale in zip(self.offsets, self.scales):   
                data += utils.from_short(int(offset * 1000))
                data += utils.from_short(int(scale * 1000))
            
            # Pass on to super-class
            Payload.__init__(self, raw = data, timestamp = timestamp)
        
        # Parse Constructor
        else:
            # For now, assume this is a result coming back.
            # (Depending on the payload length, we could infer whether this
            # is a config set operation or a config request result.)
                
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
            
            for i in range(0, self._num_sensors(raw)):
                base = self._base_for_value(i)
                self.offsets.append(utils.to_short(raw[base:base + 2]) / 1000.0)
                self.scales.append(utils.to_short(raw[base + 2:base + 4]) / 1000.0)

    def _num_sensors(self, raw):
        return raw[0]

    def _base_for_value(self, i):
        return i * 4 + 1


class CurrentSensorConfig(VariableSensorConfig):
    """Horizon Message Payload - Current Sensor Configuration"""

    def print_format(self):
        lines = []
        for i in range(len(self.offsets)):
            lines.append("Current Sensor %d: Offset: %f,  Scale: %f" % (i + 1, self.offsets[i], self.scales[i]))
        return "\n".join(lines)


class VoltageSensorConfig(VariableSensorConfig):
    """Horizon Message Payload - Voltage Sensor Configuration"""

    def print_format(self):
        lines = []
        for i in range(len(self.offsets)):
            lines.append("Voltage Sensor %d: Offset: %f,  Scale: %f" % (i + 1, self.offsets[i], self.scales[i]))
        return "\n".join(lines)


class TemperatureSensorConfig(VariableSensorConfig):
    """Horizon Message Payload - Temperature Sensor Configuration"""

    def print_format(self):
        lines = []
        for i in range(len(self.offsets)):
            lines.append("Temperature Sensor %d: Offset: %f,  Scale: %f" % (i + 1, self.offsets[i], self.scales[i]))
        return "\n".join(lines)


class OrientationSensorConfig(VariableSensorConfig):
    """Horizon Message Payload - Orientation Sensor Configuration"""

    def __init__(self, passcode = 0, roll_offset = 0, roll_scale = 0, 
                 pitch_offset = 0, pitch_scale = 0, yaw_offset = 0, 
                 yaw_scale = 0, raw = None, 
                 timestamp = 0):
        
        offsets = [roll_offset, pitch_offset, yaw_offset]
        scales = [roll_scale, pitch_scale, yaw_scale]
        
        VariableSensorConfig.__init__(self, passcode, offsets, scales, raw, timestamp)

        self.roll_offset, self.pitch_offset, self.yaw_offset = self.offsets
        self.roll_scale, self.pitch_scale, self.yaw_scale = self.scales

    def print_format(self):
        lines = []
        lines.append("Roll offset: %f,  Roll scale: %f" % (self.offsets[0], self.scales[0]))
        lines.append("Pitch offset: %f,  Pitch scale: %f" % (self.offsets[1], self.scales[1]))
        lines.append("Yaw offset: %f,  Yaw scale: %f" % (self.offsets[2], self.scales[2]))
        return "\n".join(lines)

    def _num_sensors(self, raw):
        return 3

    def _base_for_value(self, i):
        return i * 4


class GyroConfig(OrientationSensorConfig):
    """Horizon Message Payload - Gyro Configuration, identical to orientation sensor"""
    pass


class AccelerometerConfig(VariableSensorConfig):
    """Horizon Message Payload - Orientation Sensor Configuration"""

    def __init__(self, passcode = 0, x_offset = 0, x_scale = 0, y_offset = 0, 
                 y_scale = 0, z_offset = 0, z_scale = 0, 
                 raw = None, timestamp = 0):
        
        offsets = [x_offset, y_offset, z_offset]
        scales = [x_scale, y_scale, z_scale]
        
        VariableSensorConfig.__init__(self, passcode, offsets, scales, raw, timestamp)

        self.x_offset, self.y_offset, self.z_offset = self.offsets
        self.x_scale, self.y_scale, self.z_scale = self.scales

    def print_format(self):
        lines = []
        lines.append("x-axis offset: %f, scale: %f" % (self.offsets[0], self.scales[0]))
        lines.append("y-axis offset: %f, scale: %f" % (self.offsets[1], self.scales[1]))
        lines.append("z-axis offset: %f, scale: %f" % (self.offsets[2], self.scales[2]))
        return "\n".join(lines)

    def _num_sensors(self, raw):
        return 3

    def _base_for_value(self, i):
        return i * 4


class MagnetometerConfig(AccelerometerConfig):
    """Horizon Message Payload - Magnetometer Configuration, identical to accelerometer"""
    pass


class EncodersConfig(Payload):
    """Horizon Message Abstract Payload - Sensor Configuration"""

    def __init__(self, ppr = [], scales = [], 
                 raw = None, timestamp = 0):
        
        # Class Variables
        self.ppr = []
        self.scales = []
        
        # Create Constructor
        if raw == None:
            # Assume this is a config set going out.

            # Magic passcode
            data = []
            
            self.ppr = ppr
            self.scales = scales

            if len(self.ppr) != len(self.scales):
                raise ValueError("PPR and scales are not the same length!")
        
            for ppr, scale in zip(self.ppr, self.scales):   
                data += utils.from_short(int(ppr))
                data += utils.from_short(int(scale * 1000))
             
            # Pass on to super-class
            Payload.__init__(self, raw = data, timestamp = timestamp)
        
        # Parse Constructor
        else:
            # For now, assume this is a result coming back.
            # (Depending on the payload length, we could infer whether this
            # is a config set operation or a config request result.)
                
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
            
            for i in range(0, raw[0]):
                base = i * 4 + 1
                self.ppr.append(utils.to_short(raw[base:base + 2]))
                self.scales.append(utils.to_short(raw[base + 2:base + 4]) / 1000.0)

    def print_format(self):
        lines = []
        for i in range(len(self.ppr)):
            lines.append("Encoder %d: PPR: %f,  Scale: %f" % (i + 1, self.ppr[i], self.scales[i]))
        return "\n".join(lines)


# Abstract class. This forms the basis for all the sensor config messages, as they are all
# sequences of pairs of offset/scale values.
class RawSensor(Payload):
    """Horizon Message Abstract Payload - Raw Sensor Data"""

    def __init__(self, raw = None, timestamp = 0):
        
        # Class Variables
        self.raw_values = []
        
        # Create Constructor
        if raw != None:
            # Result coming back from firmware            
                
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)
            
            for i in range(0, self._num_sensors(raw)):
                base = self._base_for_value(i)
                self.raw_values.append(utils.to_short(raw[base:base + 2]))

    def _num_sensors(self, raw):
        return raw[0]

    def _base_for_value(self, i):
        return i * 2 + 1



class RawCurrentSensor(RawSensor):
    """Horizon Message Payload - Raw Current Data"""

    def __init__(self, raw = None, timestamp = 0):
        RawSensor.__init__(self, raw, timestamp)

        # Class Variables
        self.raw_currents = self.raw_values
        

    def print_format(self):
        lines = []
        for i in range(len(self.raw_currents)):
            lines.append("Raw Current %d: %d" % (i + 1, self.raw_currents[i]))
        return "\n".join(lines)



class RawVoltageSensor(RawSensor):
    """Horizon Message Payload - Raw Voltage Data"""

    def __init__(self, raw = None, timestamp = 0):
        RawSensor.__init__(self, raw, timestamp)

        # Class Variables
        self.raw_voltages = self.raw_values
        

    def print_format(self):
        lines = []
        for i in range(len(self.raw_voltages)):
            lines.append("Raw Voltage %d: %d" % (i + 1, self.raw_voltages[i]))
        return "\n".join(lines)


class RawTemperatureSensor(RawSensor):
    """Horizon Message Payload - Raw Temperature Data"""

    def __init__(self, raw = None, timestamp = 0):
        RawSensor.__init__(self, raw, timestamp)

        # Class Variables
        self.raw_temperatures = self.raw_values
        

    def print_format(self):
        lines = []
        for i in range(len(self.raw_temperatures)):
            lines.append("Raw Temperature %d: %d" % (i + 1, self.raw_temperatures[i]))
        return "\n".join(lines)


class RawOrientationSensor(RawSensor):
    """Horizon Message Payload - Raw Orientation Data"""

    def __init__(self, raw = None, timestamp = 0):
        RawSensor.__init__(self, raw, timestamp)

        # Class Variables
        self.raw_roll, self.raw_pitch, self.raw_yaw = self.raw_values

    def _num_sensors(self, raw):
        return 3

    def _base_for_value(self, i):
        return i * 2

    def print_format(self):
        lines = []
        lines.append("Raw Roll: %d" % self.raw_roll)
        lines.append("Raw Pitch: %d" % self.raw_pitch)
        lines.append("Raw Yaw: %d" % self.raw_yaw)
        return "\n".join(lines)


class RawGyro(RawOrientationSensor):
    """Horizon Message Payload - Raw Gyro Data, same as raw orientation data"""
    pass


class RawAccelerometer(RawSensor):
    """Horizon Message Payload - Raw Orientation Data"""

    def __init__(self, raw = None, timestamp = 0):
        RawSensor.__init__(self, raw, timestamp)

        # Class Variables
        self.raw_x, self.raw_y, self.raw_z = self.raw_values

    def _num_sensors(self, raw):
        return 3

    def _base_for_value(self, i):
        return i * 2

    def print_format(self):
        lines = []
        lines.append("Raw X: %d" % self.raw_x)
        lines.append("Raw Y: %d" % self.raw_y)
        lines.append("Raw Z: %d" % self.raw_z)
        return "\n".join(lines)


class RawMagnetometer(RawSensor):
    """Horizon Message Payload - Raw Magnetometer Data"""

    def __init__(self, raw = None, timestamp = 0):
        RawSensor.__init__(self, raw, timestamp)

        # Class Variables
        self.raw_x, self.raw_y, self.raw_z = self.raw_values

    def _num_sensors(self, raw):
        return 3

    def _base_for_value(self, i):
        return i * 2

    def print_format(self):
        lines = []
        lines.append("Raw X: %d" % self.raw_x)
        lines.append("Raw Y: %d" % self.raw_y)
        lines.append("Raw Z: %d" % self.raw_z)
        return "\n".join(lines)



class RestoreSystemConfig(Payload):
    """Horizon Message Payload - Restore System Configuration"""

    def __init__(self, passcode = 0x3A18, flags = 1, raw = None, 
                 timestamp = 0):
        """Create A Horizon Message Payload - Reset"""
        
        self.passcode = passcode
        self.flags = flags

        # Create Constructor
        if raw == None:
            data = utils.from_unsigned_short(self.passcode) + utils.from_byte(self.flags)
        
            Payload.__init__(self, raw = data, timestamp = timestamp)
        
        # Parse Constructor
        else:
            raise NotImplementedError('Payload %s cannot parse its response.' % self.__class__.__name__ )
    
    def print_format(self):
        """Return the payload as a human readable string"""
        return "Passcode: 0x%04X" % self.passcode




class StoreSystemConfig(Payload):
    """Horizon Message Payload - Store System Configuration"""

    def __init__(self, passcode = 0x3A18, raw = None, 
                 timestamp = 0):
        """Create A Horizon Message Payload - Reset"""
        
        self.passcode = passcode
        
        # Create Constructor
        if raw == None:
            data = utils.from_unsigned_short(self.passcode)
        
            Payload.__init__(self, raw = data, timestamp = timestamp)
        
        # Parse Constructor
        else:
            raise NotImplementedError('Payload %s cannot parse its response.' % self.__class__.__name__ )
     
    def print_format(self):
        """Return the payload as a human readable string"""
        return "Passcode: 0x%04X" % self.passcode


class ControlFlags(Payload):
    """Horizon Message Payload - Control Flags"""

    def __init__(self, passcode = 0, flags = 0, raw = None, timestamp = 0):
        """Create A Horizon Message Payload - Platform Information"""
        
        # Class Variables
        self.flags = 0x00000000
        self.passcode = passcode
        
        # Create Constructor - assume this is a set_platform_info payload
        if raw == None:
            data = utils.from_unsigned_short(self.passcode)
            self.flags = flags
            data += utils.from_unsigned_int(self.flags)

            # Pass on to super-class
            Payload.__init__(self, raw = data, 
                             timestamp = timestamp)
        
        # Parse Constructor
        else:
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)

            # Extract Serial
            self.flags = utils.to_unsigned_int(raw[:])
            logger.debug("%s flags: %d" % (self.__class__.__name__, self.flags))
    

    ## Human Readable Payload String
    def print_format(self):        
        return "Control Flags: %08X" % self.flags


class BatteryEstimationConfig(VariableSensorConfig):
    """Horizon Message Payload - Battery Estimation Configuration"""

    def print_format(self):
        lines = []
        for i in range(len(self.offsets)):
            lines.append("Battery %d: Offset: %f,  Scale: %f" % (i + 1, self.offsets[i], self.scales[i]))
        return "\n".join(lines)


class PlatformKinematics(Payload):

    def __init__(self, passcode = 0, track = 0, wheelbase = 0, 
                 raw = None, timestamp = 0):
        """Create A Horizon Message Payload - Platform Kinematics"""
        
        # Class Variables
        self.passcode = passcode
        self.track = track
        self.wheelbase = wheelbase
        
        # Create Constructor - assume this is a set_platform_kinematics payload
        if raw == None:
            data = utils.from_unsigned_short(self.passcode)
            data += utils.from_unsigned_short(int(self.track * 1000))
            data += utils.from_unsigned_short(int(self.wheelbase * 1000))

            # Pass on to super-class
            Payload.__init__(self, raw = data, 
                             timestamp = timestamp)
        
        # Parse Constructor
        else:
            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)

            self.track = utils.to_unsigned_short(raw[0:2]) / 1000.0
            self.wheelbase = utils.to_unsigned_short(raw[2:4]) / 1000.0

    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Platform Track: %f\nPlatform Wheelbase: %f" % (self.track, self.wheelbase)



class Config(Payload):
    """Horizon Message Indexed Config Payload"""

    def __init__(self, index=0, value=0, raw=None, timestamp=0):
        
        # Class Variables
        self.index = None
        self.value = None
        
        # Create Constructor
        if raw == None:
            # This is an SET msg going out.
            self.index = index
            self.value = value

            int_part = math.floor(self.value)
            frac_part = self.value - int_part
            data = []
            data += utils.from_unsigned_short(self.index)
            data += utils.from_unsigned_int((1 << 32) * frac_part)
            data += utils.from_int(int_part)
             
            # Pass on to super-class
            Payload.__init__(self, raw = data, timestamp = timestamp)
        
        # Parse Constructor
        else:
            # This is a DATA msg coming back.

            # Pass on to super-class
            Payload.__init__(self, raw = raw, timestamp = timestamp)

            self.index = utils.to_unsigned_short(raw[0:2])
            frac_part = float(utils.to_unsigned_int(raw[2:6])) / (1 << 32)
            int_part = utils.to_int(raw[6:10])
            self.value = int_part + frac_part

    def print_format(self):
        return "Config #%d: %f" % (self.index, self.value)


class ConfigRequest(Payload):
    """Horizon Message Indexed Config Payload"""

    def __init__(self, index=0, raw=None, timestamp=0):
        
        # Class Variables
        self.index = None
        
        # Create Constructor
        if raw == None:
            # This is an SET msg going out.
            self.index = index
            data = []
            data += utils.from_unsigned_short(self.index)
             
            # Pass on to super-class
            Payload.__init__(self, raw = data, timestamp = timestamp)
        
        # Parse Constructor
        else:
            raise NotImplementedError()

    def print_format(self):
        return "Request for config #%d" % self.index


logger.debug("... clearpath.horizon.payloads loaded.")
