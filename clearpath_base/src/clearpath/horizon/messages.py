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
#  File: messages.py
#  Desc: Horizon Protocol Message Definition
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


from .. import utils            # Clearpath Utilities
from .  import codes            # Horizon Message Codes
from .  import check

# Required Python Modules
import logging                  # Logging Utilities


# Module Support
__version__  = "1.0"
__revision__ = "$Revision: 916 $"


## Message Log
logger = logging.getLogger('clearpath.horizon.messages')
"""Horizon Messages Module Log"""
logger.setLevel(logging.DEBUG)
logger.addHandler(utils.NullLoggingHandler())
logger.propagate = False
logger.debug("Loading clearpath.horizon.messages ...")


## Horizon Message
#
#  Represents a message to be transmitted to / received from the hardware.
#
class Message(check.Fields, check.namedtuple('Message_', 'code payload no_ack timestamp')):
    """Horizon Protocol Message"""
    
    # Horizon Message Constants
    SOH = 0xAA
    STX = 0x55
 
    @classmethod
    def command(cls, name, args={}, timestamp=0, no_ack=False):
        code = codes.codes[name]
        return cls(code=code.set(), payload=code.data_payload(**args), 
                   no_ack=no_ack, timestamp=timestamp)

    @classmethod
    def request(cls, name, args={}, timestamp=0):
        code = codes.codes[name]
        return cls(code=code.request(), payload=code.request_payload(**args), 
                   no_ack=False, timestamp=timestamp)

    @classmethod
    def parse(cls, raw):
        checksum = utils.to_unsigned_short(raw[-2:])
        if utils.ccitt_checksum(raw[:-2]) != checksum:
            raise utils.ChecksumError("Bad Checksum")
            
        if raw[0] != Message.SOH:
            raise ValueError("Invalid SOH")
            
        if raw[1] != 0xFF & (~raw[2]):
            raise ValueError("Invalid length complement")
      
        timestamp = utils.to_unsigned_int(raw[4:8])
                
        if utils.to_byte(raw[8:9]) == 1:
            no_ack = True
        elif utils.to_byte(raw[8:9]) == 0:
            no_ack = False
        else:
            raise ValueError("Invalid flags")
 
        code = utils.to_unsigned_short(raw[9:11])
            
        if raw[11] != Message.STX:
            raise ValueError("Invalid STX")
 
        # If a message's stated type is a command or a request, but it's being received,
        # then its payload is actually of type acknowledgment.
        payload_cls = codes.codes[codes.names[code]].data_payload
        if code < 0x8000:
            payload_cls = codes.payloads.Ack

        # Payload
        payload = payload_cls(raw = raw[12:-2])   # payload_cls.received(raw)
        return cls(code=code, payload=payload, no_ack=no_ack, timestamp=timestamp)

   
    def __init__(self, **kwargs):
        # Assignment to fields occurs in namedtuple's __new__
        self.validate()


    def __str__(self):
        lines = []
        lines.append("Code: 0x%X" % self.code)
        lines.append("No Ack: %s" % bool(self.no_ack))
        lines.append("Timestamp: %d" % self.timestamp)
        lines.append("Payload:\n  %s" % (str(self.payload).replace("\n", "\n  ")))
        return "\n".join(lines)


    def copy(self, timestamp = None):
        """Return a copy of this message, with a new timestamp. This is for
        resending messages."""
        if timestamp == None:
            timestamp = self.timestamp
        return self._replace(timestamp=timestamp)


    def validate(self):
        self.check('timestamp').range(0, 4294967295)
        self.check('code').range(0, 65535)

        
    def data(self):
        data = []
        data += utils.from_byte(codes.VERSION_BYTE)
        data += utils.from_unsigned_int(self.timestamp)
        data += utils.from_byte(1 if self.no_ack else 0)
        data += utils.from_unsigned_short(self.code)
        data += [Message.STX]
        data += self.payload.data   # later: data()
        
        length = len(data) + 2  # +2 for checksum
        data = [Message.SOH] + utils.from_byte(length) + utils.from_byte(0xFF&(~(0xFF&length))) + data
            
        # Checksum
        checksum = utils.ccitt_checksum(data)
        data += utils.from_unsigned_short(checksum)
        return data

    
logger.debug("... clearpath.horizon.messages loaded.")
