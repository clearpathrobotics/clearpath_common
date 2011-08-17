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
#  File: transports.py
#  Desc: Horizon Message Transport Controllers
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
    print ("ERROR: clearpath.horizon.transports is a module and can NOT be run"\
           " as a script!\nFor a command-line interface demo of Horizon, run:"\
           "\n  python -m clearpath.horizon.demo\n"\
           "For Horizon message forwarding, run:\n"\
           "  python -m clearpath.horizon.forward")

    # Exit Error
    import sys
    sys.exit(1)




################################################################################
# Module



 ## @package clearpath.horizon.transports 
#  Horizon Transports Python Module
# 
#  Horizon Message Transport Controllers                                      \n
#  Supported Horizon version(s): 0.1 - 1.0                                    \n
#                                                                             \n
#  Supported Transports:
#  - Serial (RS-232)
#
#  @author     Ryan Gariepy
#  @author     Malcolm Robert
#  @date       18/01/10
#  @todo       Would be nice to support version translation in IP Servers, 
#              however, that would require additional work to detect the
#              version that clients are using which is currently not supported
#              by the current framework.
#  @todo       FIX encryption
#  @req        clearpath.utils                                                \n
#              clearpath.horizon.messages                                     \n
#              clearpath.horizon.payloads                                     \n
#              pySerial [http://pyserial.sourceforge.net/]                    \n
#              -- for Serial
#  @version    1.0
#
#  @section USE
#
#  The intended purpose of this module is to provide functionality to send
#  and receive Horizon messages over the various transports with minimal
#  knowledge of messages and no knowledge of message payloads. Further, as
#  some transports support a one-to-many connection scheme, this module also
#  provides message routing. Note that as this module implements the transports
#  as defined in the Horizon specification, there is no support for version
#  translation. If version translation is desired, refer to the module 
#  clearpath.horizon.forward.
#
#  @section HISTORY
#
#  Versions 0.1 - 0.3 {Ryan Gariepy}
#  - Initial Creation as protocol.py
#
#  Version 0.4 {Malcolm Robert}
#  - Moved to horizon_protocol.py
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
#  - Moved to transport.py
#  - Added Server transports
#  - Added IPv6 support
#  - Horizon support for v 0.1 - 0.7
#  - Python 2.6+ & 3.x compatible
#
#  Version 0.8
#  - Horizon support for v 0.1 - 0.8
#
#  Version 1.0
#  - Added one-to-many support
#  - Horizon support for v 0.1 - 1.0
#
#  @section License
#  @copydoc public_license
#
#  @defgroup hardware Hardware
#  @ingroup doc
#  
#  @copydoc serial
#
#  If higher bandwidth communication is required, the protocol may also be 
#  packaged verbatim within TCP/IP or UDP packets if desired, with one protocol 
#  message per packet. The default Horizon port for TCP/UDP is 7284.
#
#  @copydoc tcp
#
#  @copydoc udp
#
"""Horizon Message Transport Controllers

   Copyright © 2010 Clearpath Robotics, Inc.
   All rights reserved
   
   Created: 17/03/10
   Authors: Ryan Gariepy & Malcolm Robert
   Version: 1.0
   """


# Required Clearpath Modules
from .. import utils            # Clearpath Utilities
from .  import messages         # Horizon Protocol Message Definition
from .  import codes            # Horizon Protocol Versions 

# Required Python Modules
import logging                  # Logging Utilities
import socket                   # UDP Port Control
import sys                      # Python Interpreter Functionality
import threading                # Python Thread Support
import time                     # System Date & Time
import collections              # deque
import math

# Version Dependent Modules
try:
    import serial               # Serial Port Control
except ImportError:
    pass

MessageRecord = collections.namedtuple('MessageRecord', 'message, expiry')


# Module Support
## Module Version
__version__  = "1.0"
"""Module Version"""
## SVN Code Revision
__revision__ = "$Revision: 898 $"
""" SVN Code Revision"""


## Message Log
logger = logging.getLogger('clearpath.horizon.transports')
"""Horizon Transports Module Log"""
logger.setLevel(logging.NOTSET)
logger.addHandler(utils.NullLoggingHandler())
logger.propagate = False
logger.debug("Loading clearpath.horizon.transports ...")              




################################################################################
# Horizon Transport Controller



## Transport
#
class Transport(object):
    """Horizon Transport Base Class"""
    pass


## Horizon Serial Controller
#
#  Provides a method to send and receive messages over RS-232.                \n
#  Guarantees order of arrival and arrival.                                   \n
#  Low Bandwidth use only.
#
#  @req        pySerial [http://pyserial.sourceforge.net/]
#  @since 0.1
#
#  @pydoc
class Serial(Transport):
    """Horizon Transport Controller - Serial Device"""
    
    def __init__(self, port = None, store_timeout = 0, receive_callback = None):
        """Create A Horizon Serial Transport"""

        # Dependency check
        try:
            serial
        except NameError:
            logger.error("%s: Cannot create Horizon Serial Transport without"\
                         "pySerial!" % self.__class__.__name__)
            raise utils.TransportError ("pySerial not found!")
    
        if port == None: 
            raise utils.TransportError \
                ("Serial transport creation failed: port not specified!\n")

        # Class Variables
        self.port = port
        self._serial = None    
        self._opened = False
        self.store_timeout = store_timeout
        self.receive_callback = receive_callback
        self.serial_write_lock = threading.Lock()

        # Initialization
        try:
            self._serial = serial.Serial()
            self._serial.port = port
            self._serial.baudrate = 115200
            self._serial.timeout = 0
    
        # Creation failed
        except serial.SerialException as ex:
            raise utils.TransportError \
                ("Serial Transport creation failed!\n" + str(ex))
      

    @classmethod
    def autodetect(cls, **kwargs):
        ports = utils.list_serial_ports()
        logger.info("%s: Attempting autodetect with %s." % 
                    (cls.__name__, ' '.join(ports)))

        for trynum in range(5):
            logger.info("%s: Autodetect try #%d." %  
                        (cls.__name__, trynum + 1))
            for port in ports:
                kwargs['port'] = port
                transport = cls(**kwargs)
                try:
                    transport.open()
                    return transport
                except utils.TransportError:
                    # Not this one, move on.
                    pass

        raise utils.TransportError("Unable to autodetect a serial Horizon device.")
            

    def __str__(self):
        """Return the transport device name."""
        return self.port
    

    def open(self):
        if (not self._opened):
            logger.debug("%s: Beginning transport opening for %s." % 
                         (self.__class__.__name__, self._serial.portstr))

            try:
                self._serial.open()
                if not self._serial.isOpen():
                    raise serial.SerialException
            except serial.SerialException:
                logger.debug("%s: Transport opening failed." % self.__class__.__name__)
                raise utils.TransportError("Serial Port opening failed.")

            self._opened = True
            self.receiver = self.Receiver(self._serial, self.store_timeout, self.receive_callback)
            self.receiver.start()
            time.sleep(0.1)
 
            logger.debug("%s: Sending ping request." % (self.__class__.__name__))
            message = messages.Message.request('echo') 
            try: 
                self.send_message(message)
            except utils.TransportError as ex:
                # Must catch this for the sake of closing the serial port and also
                # killing the receiver thread.
                self.close()
                raise utils.TransportError("Serial Port message send failed.\n" + str(ex))
  
            logger.debug("%s: Ping request sent." % (self.__class__.__name__))

            for sleeping in range(5):
                time.sleep(0.1)
                waiting = self.receiver.get_waiting()
                for message in waiting:
                    logger.debug("%s: Message received." % (self.__class__.__name__))
                    if codes.names[message.code] == 'echo':
                        # Success
                        logger.debug("%s: Transport opened." % self.__class__.__name__)
                        return
  
            logger.debug("%s: No response to ping request." % self.__class__.__name__)
            self.close()
            raise utils.TransportError("Could not communicate with Clearpath platform.")


    def close(self):
        logger.debug("%s: Beginning transport closing for %s." %
                (self.__class__.__name__, self._serial.portstr))
        self.receiver.stop()
        self.receiver.join()
        self._serial.close()
        self._opened = False
        logger.debug("%s: Transport closed." % self.__class__.__name__)

    
    def is_open(self):
        return self._opened;


    def send_message(self, message):
        """Serial Transport Device Send Horizon Message"""
        self.send_raw(utils.to_bytes(message.data()))


    def send_raw(self, raw):
        if not self._opened:
            raise utils.TransportError ("Cannot send while closed!")

        try:
            with self.serial_write_lock:
                try:
                    getattr(serial, "serial_for_url")
                    sent = self._serial.write(raw)
                    if sent == None:
                        raise utils.TransportError ("Write Failed!")

                except AttributeError:
                    if sys.version_info[0] > 2:
                        self._serial.write(list(map(chr, raw)))
                    else:
                        self._serial.write(raw)
                    sent = len(raw)
 
                if sent < len(raw):
                    raise utils.TransportError ("Write Incomplete!")
                
        # Send Failed
        except serial.SerialException as ex:
            raise utils.TransportError \
                    ("Serial Message send failed!\n" + str(ex))


    class Receiver(threading.Thread):

        def __init__(self, serial, store_timeout, callback):
            threading.Thread.__init__(self, name = 'clearpath.horizon.transports.Serial.Receiver')
            self._running = False
            self._buffer = []
            self._serial = serial
            self._callback = callback
            self.start_time = time.time()
            self.store_timeout = store_timeout

            # Received Messages
            self._acks_lock = threading.Lock()
            self._acks = {}  # Format: {timestamp: (MessageRecord)}
            self._received_lock = threading.Lock()
            self._received = collections.deque()  # Format: MessageRecord

            # self.daemon = True


        def stop(self):
            self._running = False


        # Only for internal use by Receiver methods. These timestamps have nothing to
        # do with platform time or API time.
        def _timestamp(self):
            return math.floor((time.time() - self.start_time) * 1000)


        def get_waiting(self, code=0x0000):
            """Horizon Protocol Get Waiting Messages"""
            msgs = []
            skip = collections.deque()
            with self._received_lock:
                try:
                    while True:
                        message_record = self._received.popleft()
                        if code == 0 or message_record.message.code == code:
                            msgs.append(message_record.message)
                        else:
                            skip.append(message_record) 
                except IndexError:
                    # No more items in the list
                    pass

                # All the ones we didn't return throw back in the received bin
                self._received = skip
            return msgs


        def run(self):
            logger.debug("%s: Entering receive loop for %s." % 
                         (self.__class__.__name__, self._serial.portstr))
            self._running = True
            
            while self._running:
                # Check for message
                try:
                    message = None
                    message = self._get_message()
                    if message != None:
                        logger.debug("%s: received message:\n %s" % \
                                 (self.__class__.__name__, str(message)))
                except IOError as ex:  # TransportError
                    # Silently swallow         
                    logger.warning(
                        "%s: IO error in attempting to retrieve message:\n%s" %\
                            (self.__class__.__name__, ex))
                except ValueError as ex:  # ChecksumError
                    # Silently swallow         
                    logger.info(
                        "%s: Value error in received message:\n%s" %\
                            (self.__class__.__name__, ex))
                    
                # If it's a good message with a payload, handle it.
                if message != None and message.payload != None:
                    if message.payload.__class__ == codes.payloads.Ack:
                        # Mark the ack as received. It will be up to the main
                        # thread to examine this for status and throw any necessary
                        # exceptions.
                        with self._acks_lock:
                            # It's keyed to the message's baked-in timestamp, because that's how
                            # acks are matched. But it's also stored beside the current time per
                            # this computer, for the purposes of expiring it.
                            self._acks[message.timestamp] = MessageRecord(message = message, 
                                                                          expiry = self._timestamp() + 
                                                                                   self.store_timeout)

                    elif self._callback != None and self._callback(message):
                        # Okay, handled by asyncronous handlers.
                        pass

                    else:
                        # Unhandled. Store it for synchronous access.
                        with self._received_lock:
                            self._received.append(MessageRecord(message = message, 
                                                                expiry = self._timestamp() + 
                                                                         self.store_timeout))

                # Check timeouts
                current = self._timestamp()
                with self._received_lock:  # stored messages
                    try:
                        while current > self._received[0].expiry:
                            self._received.popleft()
                    except IndexError:
                        # No more items in list.
                        pass
                with self._acks_lock:  # stored acks
                    for ts, message_record in list(self._acks.items()): 
                        if current > message_record.expiry:
                            del self._acks[ts]

                # Release Processor
                time.sleep(0.001)
            
            logger.debug("%s: Exiting receive loop for %s." % 
                         (self.__class__.__name__, self._serial.portstr))
 
        def has_ack(self, timestamp):
            with self._acks_lock:
                if timestamp in self._acks:
                    ack = self._acks[timestamp][0]
                    del self._acks[timestamp]
                    return ack
                else:
                    return False


        def _get_message(self):
            read = 0

            # read as much as possible without blocking (as timeout = 0)
            chars = self._serial.read(1000)

            if len(chars) > 0:
                try:
                    getattr(serial, "serial_for_url")
                    if sys.version_info[0] > 2:
                        self._buffer += chars
                    else:
                        self._buffer += list(map(ord,chars))
                except AttributeError:
                    self._buffer += list(map(ord,chars))

            # Discard bytes from the buffer until we find a 
            # SOH character followed by two characters which are
            # complements, representing the length.
            disc = []
            while(len(self._buffer) > 3 and (
                    self._buffer[0] != messages.Message.SOH or
                    self._buffer[1] != 0xFF & (~self._buffer[2]) or
                    self._buffer[1] == 0)):
                disc.append(self._buffer.pop(0))

            if len(disc) > 0:
                logger.info("%s: Discarded %d bytes:\n%s" % (
                        self.__class__.__name__, len(disc), 
                        ' '.join(map(utils.hex,disc))))

            if len(self._buffer) < 3:
                # Not enough data in the buffer read a SOH + LEN + ~LEN.
                return None
     
            length = self._buffer[1] + 3
        
            # Now know message length. Is there enough in the buffer yet to read
            # the whole message? If not, wait.
            if len(self._buffer) < length:
                return None
        
            # Create and return the Message
            raw = self._buffer[0:length]
            self._buffer = self._buffer[length:]
            logger.info("%s: Message of %d bytes found:\n%s" % (
                    self.__class__.__name__, len(raw), 
                    ' '.join(map(utils.hex,raw))))

            # This will throw errors if has a bad checksum, for example,
            # which will bubble up to the run() function.
            return messages.Message.parse(raw)

       
    logger.debug("... clearpath.horizon.transports loaded.")
    
