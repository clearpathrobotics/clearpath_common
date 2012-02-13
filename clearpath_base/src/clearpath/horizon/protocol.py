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
#  File: protocol.py
#  Desc: Horizon Protocol Handlers
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
    print ("ERROR: clearpath.horizon.protocol is a module and can NOT be run"\
           " as a script!\nFor a command-line interface demo of Horizon, run:"\
           "\n  python -m clearpath.horizon.demo\n"\
           "For Horizon message forwarding, run:\n"\
           "  python -m clearpath.horizon.forward")

    # Exit Error
    import sys
    sys.exit(1)




################################################################################
# Module



## @package clearpath.horizon.protocol 
#  Horizon Protocol Python Module
# 
#  Horizon Protocol Message Handlers                                          \n
#  Abstracted from knowing messages & underlying transport.                   \n
#  Supported Horizon version(s): 0.1 - 1.0
#
#  @author     Ryan Gariepy
#  @author     Malcolm Robert
#  @date       18/01/10
#  @todo       Implement the protocol server wrapper & document in use
#  @req        clearpath.utils                                                \n
#              clearpath.horizon.codes                                        \n
#              clearpath.horizon.messages                                     \n
#              clearpath.horizon.payloads                                     \n
#              clearpath.horizon.transports                                   \n
#  @version    1.0
#
#  @section HORIZON
#  @copydoc overview1
#
#  @section USE
#
#  The intended purpose of this module is to provide a layer between the 
#  Horizon interface (or emulator) and the low-level transports that will 
#  automatically handle acknowledgments and message formatting. Only 
#  Payload classes, message codes, and errors are exposed.
#
#
#  @section HISTORY
#  Version 0.1 - 0.3 {Ryan Gariepy}
#  - Initial Creation as protocol.py
#
#  Version 0.4 {Malcolm Robert}
#  - Move to horizon_protocol.py
#  - Added protocol abstraction
#  - Added TCP/IP
#  - Added UDP
#  - Added logging
#  - Added version support
#  - Added Doxygen documentation
#  - Changed version scheme to match Horizon doc
#  - Horizon support for v0.4
#
#  Version 0.5
#  - Horizon support for v0.5
#
#  Version 0.6
#  - Moved to protocol.py
#  - Horizon support for v0.6
#  - Improved search for header in read
#  - Added TCP encryption
#
#  Version 0.7
#  - Extracted transports to transports.py
#  - Horizon support for v0.7
#
#  Version 0.8
#  - Horizon support for v 0.1 - 0.8
#
#  Version 1.0
#  - Horizon support for v 0.1 - 1.0
#  - Python 2.6+ & 3.x compatible
#
#  @section License
#  @copydoc public_license
#
#  @defgroup overview1 Overview Part I
#  @ingroup overview
#  This protocol is meant as a simple way for users of the various Clearpath 
#  Robotics research offerings to interface with the Clearpath Robotics 
#  hardware. It includes several features intended to increase communication 
#  reliability, while keeping message overhead and protocol complexity low. 
#  For the sake of rapid prototyping, it is not intended for multiple devices 
#  to be simultaneously connected to each communication line, removing the need 
#  for addressing or negotiation.
#
"""Horizon Protocol Message Handlers

   Copyright © 2010 Clearpath Robotics, Inc.
   All rights reserved
   
   Created: 18/01/10
   Authors: Ryan Gariepy & Malcolm Robert
   Version: 1.0
   """


# Required Clearpath Modules
from .. import utils            # Clearpath Utilities
from .  import codes            # Horizon Message Codes
from .  import messages         # Horizon Protocol Message Definition
from .  import transports       # Horizon Transport Definitions

# Required Python Modules
import datetime                 # Date & Time Manipulation
import logging                  # Logging Utilities
import sys                      # Python Interpreter Functionality
import time                     # System Date & Time
import math


# Module Support
__version__  = "1.0"
__revision__ = "$Revision: 916 $"


## Message Log
logger = logging.getLogger('clearpath.horizon.protocol')
"""Horizon Protocol Module Log"""
logger.setLevel(logging.NOTSET)
logger.addHandler(utils.NullLoggingHandler())
logger.propagate = False
logger.debug("Loading clearpath.horizon.protocol ...")         




class Client(object):
    """Horizon Transport Protocol Controller - Client Device"""
    
    
        
    ## Create A Horizon Protocol Client
    #  
    #  Constructor for the Horizon message Transport protocol client.         \n
    #  Performs the initial creation and initialization of the underlying 
    #  transport.                                                             \n
    #  Does NOT support version auto-detection.                               \n
    #                                                                         \n
    #  Refer to the transport's __init__ method for argument specifications.
    #                                                                         \n
    #  Override this method for subclass initialization.                      \n
    #  Overriding methods should call this method. 
    #
    #  @param  retries        The number of times to retry sending a message
    #                         that received a timeout or checksum error
    #  @param  send_timeout   The time to wait for an acknowledgment
    #                         in milliseconds, 0 - wait indefinitely
    #  @param  store_timeout  The time to store an un-handled message for the 
    #                         method get_waiting in milliseconds,
    #                         0 - store indefinitely
    #  @param  sys_time       Use system time (True) instead of time since 
    #                         instantiation (False)?
    #  @param  transport      The Transport class to use.
    #  @param  transport_args Dictionary of arguments to pass to the transport's
    #                         __init__ method. Do NOT include version or
    #                         store_timeout as these will be populated.
    #
    #  @pydoc
    def __init__(self, transport, transport_args, retries, 
                 send_timeout, rec_timeout, store_timeout):

        # Class Variables
        ## Message Handlers
        self._handlers_lock = transports.threading.Lock()
        self._handlers = { 0:[] }     # Format: { code:[handler] }

        self.start_time = time.time()
        self._transport = None
        self._transport_func = transport
        self._transport_args = transport_args
        self._retries = retries
        self._send_timeout = send_timeout
        self._rec_timeout = rec_timeout
        
        self.acks = True;
 
        # Create Transport
        transport_args['receive_callback'] = self.do_handlers
        transport_args['store_timeout'] = store_timeout


    def __del__(self):
        """Destroy A Horizon Transport"""
        # Cleanup Transport
        self.close()
        

    def __str__(self):
        """Return the transport name."""
        return str(self._transport)
    

    def open(self):
        if not self._transport:
            self._transport = self._transport_func(**self._transport_args)
            if not isinstance(self._transport, transports.Transport):
                raise ValueError ("Invalid transport!")

        if not self._transport.is_open():
            self._transport.open()
        
    
    def close(self):
        self.remove_handler()
        if self._transport != None:
            self._transport.close()


    # If no offset provided, use the program start time.
    def timestamp(self):
        return math.floor((time.time() - self.start_time) * 1000)


    def emergency_stop(self):
        code = codes.codes['safety_status']
        self.send_message(code.set, code.payload(code.payload.EMERGENCY_STOP))


    def command(self, name, args):
        if 'self' in args: del args['self']
        self.send_message(messages.Message.command(name, args, self.timestamp(), no_ack=(not self.acks)))


    def request(self, name, args):
        try:
           # Prepare 1-off handler for the first response.
            self._received = None
            self.add_handler(handler = self._receiver, request = name)
            
            # Send the Message - blocks on its ack.
            if 'self' in args: del args['self']  
            message = messages.Message.request(name, args, self.timestamp())
            self.send_message(message)

            # If this is explicitly a subscription-cancelation request,
            # then exit now...
            if 'subscription' in args and args['subscription'] == 0xFFFF:
                return;

            # Otherwise... wait on a first reply to return.
            retries = self._retries
            start = self.timestamp()
            while True:
                if self.timestamp() - start > self._rec_timeout:
                    if retries > 0:
                        message = message.copy(timestamp=self.timestamp())
                        self.send_message(message)
                        retries -= 1           
                        start = self.timestamp()
                    else:
                        raise utils.TimeoutError (
                            "Timeout Occurred waiting for response!")

                if self._received != None:
                    return self._received[1]

                time.sleep(0.001)
        finally:
            self.remove_handler(handler = self._receiver, request = name)



    def _receiver(self, name, payload, timestamp):
        self._received = (name, payload, timestamp)


    # Blocking message sender.
    def send_message(self, message): 
        """Horizon Protocol Send Message"""
        if not self._transport.is_open(): 
            raise IOError ("Transport has not been opened!")
 
        # Non-blocking message sender
        self._transport.send_message(message)
        
        # Unless instructed otherwise, block on receiving the acknowledgment
        # to this message, re-send as necessary.
        if not message.no_ack:
            tries = self._retries
            while True:
                ack = self._transport.receiver.has_ack(message.timestamp)
                if ack:
                    if ack.payload.bad_code:
                        raise utils.UnsupportedCodeError("Acknowledgment says Bad Code.")
                    elif ack.payload.bad_format:
                        raise utils.FormatError("Acknowledgment says Bad Format.")
                    elif ack.payload.bad_values:
                        raise ValueError("Acknowledgment says Bad Values.")
                    elif ack.payload.bad_frequency:
                        raise utils.SubscriptionError("Acknowledgment says Bad Frequency.")
                    elif ack.payload.bad_code_count:
                        raise utils.SubscriptionError("Acknowledgment says Too Many Subscriptions.")
                    elif ack.payload.bad_bandwidth:
                        raise utils.SubscriptionError("Acknowledgment says Not Enough Bandwidth.")
                    else:
                        # Message delivered and acknowledged successfully.
                        return True

                if self.timestamp() - message.timestamp > self._send_timeout:
                    if tries > 0:
                        # Attempt a re-send
                        tries -= 1
                        message = message.copy(timestamp = self.timestamp())
                        self._transport.send_message(message)
                    else:
                        raise utils.TimeoutError("Message Timeout Occurred!")

                time.sleep(0.001)


    def add_handler(self, handler, backtrack = False, request = None):
        """Horizon Protocol Add Data Message Handler"""
        code = 0
        if request != None:
            code = codes.codes[request].data()

        with self._handlers_lock:
            # Add Handler
            if code not in self._handlers:
                self._handlers[code] = []
            self._handlers[code].append(handler)
        
            # Backtrack
            if backtrack:
                for tup in self.get_waiting(request): 
                    handler(tup[0], tup[1], tup[2])


    def remove_handler(self, handler=None, request=None):
        """Horizon Protocol Remove Data Message Handler"""
        code = 0
        if request != None:
            code = codes.codes[request].data()

        with self._handlers_lock:
            # Remove Handler
            if code in self._handlers: 
                if handler != None and handler in self._handlers[code]:
                    self._handlers[code].remove(handler)
                elif handler == None:
                    self._handlers[code] = []


    # This function is called from the secondary thread
    # transports.Serial.Receiver. Do not manipulate global state!
    def do_handlers(self, message):
        with self._handlers_lock:
            handlers = self._handlers[0][:]
            if message.code in self._handlers:
                handlers += self._handlers[message.code]

        for handler in handlers:
            handler(codes.names[message.code], message.payload, message.timestamp)

        return len(handlers) > 0


    def get_waiting(self, request = None):
        code = 0
        if request != None:
            code = codes.codes[request].data

        waiting = []
        for message in self._transport.receiver.get_waiting(code):
            waiting.append((codes.names[message.code], message.payload, message.timestamp))

        return waiting


    def is_open(self):
        return self._transport != None and self._transport.is_open()
   

logger.debug("... clearpath.horizon.protocol loaded.")
