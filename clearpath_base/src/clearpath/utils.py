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
#  File: utils.py
#  Desc: Clearpath Robotics Inc. Utilities Python Module
#  Auth: Ryan Gariepy
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
    import sys
    
    # Warn of Module ONLY status
    if (sys.version_info[0] > 2 and sys.version_info[1] > 0) or \
            (sys.version_info[0] == 2 and sys.version_info[1] > 6):
        print ("ERROR: clearpath.utils is a module and can NOT be run as a "\
               "script!\nFor a listing of installed Clearpath Robotics Inc. "\
               "modules, run:\n  python -m clearpath")
    else:
        print ("ERROR: clearpath.utils is a module and can NOT be run as a "\
               "script!\nFor a listing of installed Clearpath Robotics Inc. "\
               "modules, run:\n  python -m clearpath.__main__")

    # Exit Error
    sys.exit(1)




################################################################################
# Module



## @package clearpath.utils 
#  Clearpath Robotics Inc. Utilities Python Module
# 
#  Utilities for use with Horizon and other Clearpath Modules                 \n
#                                                                             \n
#  Contains:
#  - Byte List Conversions
#  - Checksum Utilities: 16-bit CRC - CCITT polynomial x16+x12+x5+1 (0x1021)
#  - Common Exceptions
#  - Logging Utilities
#  - Transport Utilities
#
#  @author     Ryan Gariepy
#  @author     Malcolm Robert
#  @date       18/01/10
#  @req        pySerial [http://pyserial.sourceforge.net/]                    \n
#              -- for list_serial_ports()
#  @version    1.0
#
#  @section USE
#
#  The intended purpose of this module is to contain common classes and methods
#  that are (or could be) used in multiple Clearpath Python packages and/or
#  modules and do not provide functionality specific to one package 
#  (hence utilities).
#
#  Due to backward support for Python 2.6 and the lack of support for bytes
#  in Python 2.6, it is useful to store an array of bytes as an array of 
#  numbers. Without strict types, this presents requirements for extracting
#  and appending numbers of various types to the byte list. The methods that
#  begin with 'from_' take the number of the appropriate type (or ascii string 
#  for from_ascii) and converts it to an array of numbers all within [0,255]
#  that can be appended or inserted into other arrays. Likewise, the methods
#  that begin with 'to_' takes an array of numbers (must be of proper size)
#  and converts it to the desired type. Since most streams in 2.6 use strings
#  and streams in 3.x use bytes, the method to_bytes can be used to convert
#  the array of numbers into the appropriate format. Further, the method hex
#  is provided to display a number [0,255] in hex form (in a format preferable
#  to python's default hex method).
#
#  The method ccitt_checksum provides a 16-bit CRC table based checksum on the 
#  given array of numbers (each [0,255]) using the CCITT Polynomial (0x1021).
#  Checksums are very useful for verifying data is transmitted without error.
#
#  The Python standard built-ins do not provide many exception classes. To
#  help differentiate the types of errors encountered, the exceptions
#  ChecksumError, FormatError, SubscriptionError, TransportError, TimeoutError,
#  and UnsupportedCodeError are provided without any additional functionality
#  added beyond their parent classes.
#
#  The majority of Clearpath's Python code is in library form, intended to be
#  used by other scripts/programs/libraries. While logging debug messages is
#  useful, developers do not expect libraries to be logging messages by default
#  so the class NullLoggingHandler is provided to disable the default logging
#  action. For library loggers, set the logging level to NOTSET, set propagate
#  to False and add an instance of NullLoggingHandler as the only logging 
#  handler and all messages will be rendered disabled.
#
#  While actual communication over RS-232 (serial) is typically package
#  specific and not common, programs/scripts that invoke communication packages
#  find it useful to be able to find out what serial ports exist and are 
#  accessible on the computer. For that purpose, the method list_serial_ports
#  is provided. Note that this method requires the non-standard Python
#  library pySerial to be installed into the Python search path. 
#
#  Various logging messages (debug, warning,...) are logged with the standard
#  Python logger to the log named 'clearpath.utils'.                          \n
#  Messages are ignored by default; remove the only handler from it and turn
#  on propagation to get messages to propagate.
#
#  @section HISTORY
#  Version 0.1 - 0.3 {Ryan Gariepy}
#  - Initial Creation in protocol.py
#  
#  Version 0.5 {Malcolm Robert}
#  - Move to clearpath_utils.py
#  - Added Serial Port Listing
#  - Added TransportError
#  - Added Logging Utils
#  - Added Doxygen documentation
#  - Added independent version scheme
#
#  Version 1.0
#  - Added Exceptions
#  - Python 2.6+ & 3.x compatible
#
#  @section License
#  @copydoc public_license
#
"""Utilities for use with Horizon and other Clearpath Modules 

   Copyright © 2010 Clearpath Robotics, Inc.
   All rights reserved
   
   Created: 18/01/10
   Authors: Malcolm Robert
   Version: 1.0
   """


# Required Modules
import logging                  # Logging Utilities
import os                       # Operating System Capabilities
import re                       # Regular Expressions
import sys                      # Python Interpreter Functionality

# Non-Standard Modules
try:
    import serial               # Serial Port Control
except ImportError:
    pass


# Module Support
## Module Version
__version__  = "1.0" 
"""Module Version"""
## SVN Code Revision
__revision__ = "$Revision: 674 $"
""" SVN Code Revision"""


## Message Log
logger = logging.getLogger('clearpath.utils')
"""Clearpath Utilities Module Log"""
logger.setLevel(logging.NOTSET)
logger.propagate = False
logger.debug("Loading clearpath.utils ...")     




################################################################################
# Byte List
# @todo Consider making methods static methods within a bytes_list class



## Byte to Hex String
#
#  Display a byte in a string as HEX.
#
#  @param  byte The byte to display
#  @return string
#
#  @pydoc
def hex(byte):
    """Byte to Hex String"""
    
    return '%02X' % byte



# ----------------------------------- Signed -----------------------------------
# Signed negative numbers use two's compliment



## From Char to Byte List
#
#  Convert a signed byte to a byte list.
#  Do not pass an actual character to this function; if you need to convert
#  an actual character use from_byte(ord(character)).
#
#  @param  input The number to convert to a byte list [-128,127]
#  @return byte list
#
#  @pydoc
def from_char(input, scale = 1):
    """Char -> Byte List"""
    input = int(input * scale)
    assert input > -129 and input < 128
    
    # Convert to byte
    output = 0x00
    if input > -1: 
        
        # truncate 
        output = 0xFF & input
    else:
        
        # twos compliment
        output = 0xFF & (-input)
        output = 0xFF & (~output)
        output += 0x01
    
    return [output]



## From Byte List to Char
#
#  Convert a byte list to a signed byte.
#
#  @param  input The byte list to convert to a signed byte [byte]
#  @return signed byte
#
#  @pydoc
def to_char(input, scale = 1):
    """Byte List -> Char"""
    
    # Ensure number is correct
    assert len(input) == 1 and input[0] > -1 and input[0] < 256
    
    # convert to char
    output = 0
    if input[0] < 128: output = input[0]
    else:
        
        # invert twos compliment
        output = input[0] - 0x01
        output = 0xFF & (~output)
        output *= -1

    return output / float(scale)


## From Short to Byte List
#
#  Convert a signed short to a byte list.
#
#  @param  input The number to convert to a byte list [-32768, 32767]
#  @return byte list
#
#  @pydoc
def from_short(input, scale = 1):
    """Short -> Byte List"""
    input = int(input * scale)

    # Ensure number is correct
    assert input > -32769 and input < 32768
    
    # Convert to unsigned short
    output = 0x0000
    if input > -1: 
        
        # truncate 
        output = 0xFFFF & input
    else:
        
        # twos compliment
        output = 0xFFFF & (-input)
        output = 0xFFFF & (~output)
        output += 0x01
    
    return [output & 0xFF, (output >> 8) & 0xFF]



## From Byte List to Short
#
#  Convert a byte list to a signed short.
#
#  @param  input The byte list to convert to a signed short [byte,byte]
#  @return signed short
#
#  @pydoc
def to_short(input, scale = 1):
    """Byte List -> Short"""

    # Ensure number is correct
    assert len(input) == 2 and input[0] > -1 and input[0] < 256 and \
        input[1] > -1 and input[1] < 256
    
    # convert to short
    output = 0xFFFF & ((input[1] << 8) | input[0])
    if output > 32767:
        
        # invert twos compliment
        output -= 0x0001
        output = 0xFFFF & (~output)
        output *= -1
    
    return output / float(scale)



## From Int to Byte List
#
#  Convert a signed integer to a byte list.
#
#  @param  input The number to convert to a byte list [-2147483648, 2147483647]
#  @return byte list
#
#  @pydoc
def from_int(input, scale = 1):
    """Int -> Byte List"""
    input = int(input * scale)

    # Ensure number is correct
    if type(input) == float: input = int(input)
    assert input > -2147483649 and input < 2147483648
    
    # Convert to unsigned short
    output = 0x00000000
    if input > -1: 
        
        # truncate 
        output = 0xFFFFFFFF & input
    else:
        
        # twos compliment
        output = 0xFFFFFFFF & (-input)
        output = 0xFFFFFFFF & (~output)
        output += 0x01
    
    return [output & 0xFF, (output >> 8) & 0xFF, (output >> 16) & 0xFF, 
            (output >> 24) & 0xFF]



## From Byte List to Int
#
#  Convert a byte list to a signed integer.
#
#  @param  input The byte list to convert to a signed integer 
#                [byte,byte,byte,byte]
#  @return signed integer
#
#  @pydoc
def to_int(input, scale = 1):
    """Byte List -> Int"""
    
    # Ensure number is correct
    assert len(input) == 4 and input[0] > -1 and input[0] < 256 and \
        input[1] > -1 and input[1] < 256 and input[2] > -1 and input[2] < 256\
        and input[3] > -1 and input[3] < 256
    
    # convert to integer
    output = 0xFFFFFFFF & ((input[3] << 24) | (input[2] << 16) | 
                           (input[1] << 8) | input[0])
    if output > 2147483647:
        
        # invert twos compliment
        output -= 0x00000001
        output = 0xFFFFFFFF & (~output)
        output *= -1
    
    return output / float(scale)



# ---------------------------------- Unsigned ----------------------------------



## From Byte to Byte List
#
#  Convert an unsigned byte to a byte list.
#
#  @param  input The number to convert to a byte list [0,255]
#  @return byte list
#
#  @pydoc
def from_byte(input, scale = 1):
    """Byte -> Byte List"""
    input = int(input * scale)
    
    # Ensure number is correct
    if type(input) == float: input = int(input)
    assert input > -1 and input < 256
    
    # truncate 
    output = 0xFF & input
    
    return [output]



## From Byte List to Byte
#
#  Convert a byte list to an unsigned byte.
#
#  @param  input The byte list to convert to an unsigned byte [byte]
#  @return unsigned byte
#
#  @pydoc
def to_byte(input):
    """Byte List -> Byte"""
    
    # Ensure number is correct
    assert len(input) == 1 and input[0] > -1 and input[0] < 256
    
    return input[0]



## From Unsigned Short to Byte List
#
#  Convert an unsigned short to a byte list.
#
#  @param  input The number to convert to a byte list [0, 65535]
#  @return byte list
#
#  @pydoc
def from_unsigned_short(input, scale = 1):
    """Unsigned Short -> Byte List"""
    input = int(input * scale)
    
    # Ensure number is correct
    if type(input) == float: input = int(input)
    assert input > -1 and input < 65536
    
    # truncate 
    output = 0xFFFF & input
    
    return [output & 0xFF, (output >> 8) & 0xFF]



## From Byte List to Unsigned Short
#
#  Convert a byte list to an unsigned short.
#
#  @param  input The byte list to convert to an unsigned short [byte,byte]
#  @return unsigned short
#
#  @pydoc
def to_unsigned_short(input):
    """Byte List -> Unsigned Short"""
    
    # Ensure number is correct
    assert len(input) == 2 and input[0] > -1 and input[0] < 256 and \
        input[1] > -1 and input[1] < 256
    
    # convert to short
    output = 0xFFFF & ((input[1] << 8) | input[0])
    
    return output



## From Unsigned Int to Byte List
#
#  Convert an unsigned integer to a byte list.
#
#  @param  input The number to convert to a byte list [0, 4294967295]
#  @return byte list
#
#  @pydoc
def from_unsigned_int(input, scale = 1):
    """Unsigned Int -> Byte List"""
    input = int(input * scale)
    
    # Ensure number is correct
    if type(input) == float: input = int(input)
    assert input > -1 and input < 4294967296
    
    # truncate 
    output = 0xFFFFFFFF & input
    
    return [output & 0xFF, (output >> 8) & 0xFF, (output >> 16) & 0xFF, 
            (output >> 24) & 0xFF]



## From Byte List to Unsigned Int
#
#  Convert a byte list to an unsigned integer.
#
#  @param  input The byte list to convert to an unsigned integer 
#                [byte,byte,byte,byte]
#  @return unsigned integer
#
#  @pydoc
def to_unsigned_int(input):
    """Byte List -> Unsigned Int"""
    
    # Ensure number is correct
    assert len(input) == 4 and input[0] > -1 and input[0] < 256 and \
        input[1] > -1 and input[1] < 256 and input[2] > -1 and input[2] < 256\
        and input[3] > -1 and input[3] < 256
    
    # convert to integer
    output = 0xFFFFFFFF & ((input[3] << 24) | (input[2] << 16) | 
                           (input[1] << 8) | input[0])
    
    return output



## From ASCII to Byte List
#
#  Convert a string of ASCII to a byte list.
#
#  @param  input The string to convert to a byte list
#  @return byte list
#
#  @pydoc
def from_ascii(input):
    """ASCII -> Byte List"""
    
    # Ensure string is correct
    assert all(ord(c) < 256 for c in input)
    
    # Wrapper method for map
    def tmp(char):
        return to_byte([ord(char)])
    
    # Convert to bytes
    return list(map(tmp,input))



## From Byte List to ASCII
#
#  Convert a byte list to an ASCII string.
#
#  @param  input The byte list to convert to an ASCII string
#  @return ASCII string
#
#  @pydoc
def to_ascii(input):
    """Byte List -> ASCII"""
    
    return ''.join(map(chr,input))



## From Byte List to bytes
#
#  Convert a byte list to a bytes type.
#
#  @param  input The byte list to convert to bytes
#  @return bytes string
#
#  @pydoc
def to_bytes(input):
    """Byte List -> bytes"""
    
    if sys.version_info[0] > 2:
        return bytes(input)
    else:
        return b''.join(map(chr,input))

  
  
  
################################################################################
# Checksum
## @defgroup crc CRC Generation
#  @ingroup doc
#  
#  A 16-bit CRC using one of the CCITT polynomials is used to confirm message 
#  integrity.                                                               \n\n
#
#  <i>Polynomial:</i> x16+x12+x5+1 (0x1021)                                   \n
#
#  <i>Initial value:</i> 0xFFFF                                               \n
#
#  <i>Check constant:</i> 0x1D0F                                            \n\n
#
#  The calculated CRC of the string '123456789' should be 0x29B1              \n
#
#  To confirm CRC implementation, the following process can be used:
#  -# Calculate the CRC of any message
#  -# XOR it with 0xFFFF (bitwise inversion)
#  -# Append it to the original message
#  -# Perform the CRC calculation on the extended message
#  -# Confirm that the new CRC is equal to the check constant (0x1D0F)
#
#  \b Sample \b C \b Code \b for \b table-driven \b CRC \b computation:     \n\n
#  @code
#  /*
#   * crc.h
#   */
#
#  #ifndef __CRC16_H
#  #define __CRC16_H
#
#  /***----------Table-driven crc function----------***/
#  /* Inputs: -size of the character array,           */ 
#  /*              the CRC of which is being computed */
#  /*         - the initial value of the register to  */
#  /*              be used in the calculation         */
#  /*         - a pointer to the first element of     */
#  /*              said character array               */
#  /* Outputs: the crc as an unsigned short int       */
#  unsigned short int crc16(int size, int init_val, char *data);
#
#  #endif
#
#  /*
#   * crc.c
#   */
#
#  #include "crc.h"
#
#  //CRC lookup table for polynomial 0x1021
#  const unsigned short int table[256] =
#  {0, 4129, 8258, 12387, 16516, 20645, 24774, 28903, 33032, 37161, 41290, 
#  45419, 49548, 53677, 57806, 61935, 4657, 528, 12915, 8786, 21173, 
#  17044, 29431, 25302, 37689, 33560, 45947, 41818, 54205, 50076, 62463, 
#  58334, 9314, 13379, 1056, 5121, 25830, 29895, 17572, 21637, 42346, 
#  46411, 34088, 38153, 58862, 62927, 50604, 54669, 13907, 9842, 5649, 
#  1584, 30423, 26358, 22165, 18100, 46939, 42874, 38681, 34616, 63455, 
#  59390, 55197, 51132, 18628, 22757, 26758, 30887, 2112, 6241, 10242, 
#  14371, 51660, 55789, 59790, 63919, 35144, 39273, 43274, 47403, 23285, 
#  19156, 31415, 27286, 6769, 2640, 14899, 10770, 56317, 52188, 64447, 
#  60318, 39801, 35672, 47931, 43802, 27814, 31879, 19684, 23749, 11298, 
#  15363, 3168, 7233, 60846, 64911, 52716, 56781, 44330, 48395, 36200, 
#  40265, 32407, 28342, 24277, 20212, 15891, 11826, 7761, 3696, 65439, 
#  61374, 57309, 53244, 48923, 44858, 40793, 36728, 37256, 33193, 45514, 
#  41451, 53516, 49453, 61774, 57711, 4224, 161, 12482, 8419, 20484, 
#  16421, 28742, 24679, 33721, 37784, 41979, 46042, 49981, 54044, 58239, 
#  62302, 689, 4752, 8947, 13010, 16949, 21012, 25207, 29270, 46570, 
#  42443, 38312, 34185, 62830, 58703, 54572, 50445, 13538, 9411, 5280, 
#  1153, 29798, 25671, 21540, 17413, 42971, 47098, 34713, 38840, 59231, 
#  63358, 50973, 55100, 9939, 14066, 1681, 5808, 26199, 30326, 17941, 
#  22068, 55628, 51565, 63758, 59695, 39368, 35305, 47498, 43435, 22596, 
#  18533, 30726, 26663, 6336, 2273, 14466, 10403, 52093, 56156, 60223, 
#  64286, 35833, 39896, 43963, 48026, 19061, 23124, 27191, 31254, 2801, 
#  6864, 10931, 14994, 64814, 60687, 56684, 52557, 48554, 44427, 40424, 
#  36297, 31782, 27655, 23652, 19525, 15522, 11395, 7392, 3265, 61215, 
#  65342, 53085, 57212, 44955, 49082, 36825, 40952, 28183, 32310, 20053, 
#  24180, 11923, 16050, 3793, 7920};
#
#  /***----------Table-driven crc function----------***/
#  /* Inputs: - size of the character array, the CRC  */
#  /*               of which is being computed        */
#  /*         - the initial value of the register to  */
#  /*               be used in the calculation        */
#  /*         - a pointer to the first element of     */
#  /*               said character array              */
#  /* Outputs: the crc as an unsigned short int       */
#  unsigned short int crc16(int size, int init_val, char *data)
#  {
#        unsigned short int crc = (unsigned short int) init_val;
#        while(size--) {
#              crc = (crc << 8) ^ table[((crc >> 8) ^ *data++) & 0xFFFF];
#        }
#        return crc;
#  }
#  @endcode

  
## Precomputed checksum table. Polynomial 0x1021. 
#
#  Used for performing a 16-bit CRC with polynomial 0x1021
CCIT_CRC_TABLE = (
    0x0, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x840, 0x1861, 0x2802, 0x3823, 
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0xa50, 0x3a33, 0x2a12, 
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0xc60, 0x1c41, 
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0xe70, 
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0xa1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067, 
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 
    0x2b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 
    0x34e2, 0x24c3, 0x14a0, 0x481, 0x7466, 0x6447, 0x5424, 0x4405, 
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 
    0x26d3, 0x36f2, 0x691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x8e1, 0x3882, 0x28a3, 
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0xaf1, 0x1ad0, 0x2ab3, 0x3a92, 
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0xcc1, 
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0xed1, 0x1ef0
    )



## Perform a 16-bit CRC with CCITT Polynomial 0x1021
#
#  @param  data     A Byte List to checksum
#  @param  init_val The initial value to calculate the checksum with. 
#                   The default value of 0xffff performs a proper checksum.
#  @return          Resultant Checksum (16-bits)
#
#  @pydoc
def ccitt_checksum(data, init_val=0xFFFF):
    """Perform a 16-bit CRC with CCITT Polynomial 0x1021"""
    
    crc = init_val
    for byte in data:
        crc = ((crc << 8) & 0xff00) ^ CCIT_CRC_TABLE[((crc >> 8) ^ byte) & 0xFF]
    return crc




################################################################################
# Exceptions


## Checksum Exception
#
#  Used to indicate a bad checksum.
class ChecksumError(ValueError):
    """Used to indicate a bad checksum."""


## Format Exception
#
#  Used to indicate a bad data format.
class FormatError(ValueError):
    """Used to indicate a bad data format."""


## Subscription Exception
#
#  Used to indicate a bad subscription request.
class SubscriptionError(ValueError):
    """Used to indicate a bad subscription request."""


## Transport Exception
#
#  Used to indicate a transport problem.
class TransportError(IOError):
    """Used to indicate a transport problem."""


## Timeout Exception
#
#  Used to indicate a timeout occurred.
class TimeoutError(TransportError):
    """Used to indicate a timeout occurred."""


## Unsupported Code Exception
#
#  Used to indicate an unsupported code.
class UnsupportedCodeError(NotImplementedError):
    """Used to indicate an unsupported code."""




################################################################################
# Logging


## Null Logging Handler
#
#  Implements a logging handler that does nothing with the event.             \n
#  Useful for suppressing log messages when logging in a library.
#
#  @pydoc
class NullLoggingHandler(logging.Handler):
    """Null Logging Handler"""
    
    
    ## Logging Event
    #
    #  Does Nothing.
    #
    #  @param record The received logging record.
    #
    #  @pydoc
    def emit(self, record):
        """Logging Event"""
    
        pass # Do Nothing
   
   

# Update module logger
logger.addHandler(NullLoggingHandler())




################################################################################
# Transport



## List machine serial ports.
#
#  Utility to get a listing of OS known serial ports.
#  - Posix:   Lists devices in /dev that matches ttyS# or ttyUSB#             \n
#             Degrades to listing devices accessible by the serial module 
#  - Windows: Lists COM devices accessible by the serial module
#
#  @req       pySerial [http://pyserial.sourceforge.net/]
#
#  @pre       OS must be Posix compliant or Windows NT
#  @pre       Posix: Read access on /dev or full access of Serial ports
#  @pre       Windows: Full access of COM ports
#  @throws    OSError If Unsupported OS (not Posix or Windows)
#  @throws    IOError If can't read '/dev' on Posix
#  @return    List of serial port names ('/dev/ttyS0', '/dev/ttyUSB0', 'COM0'),
#             Empty List if none found
#
#  @pydoc
def list_serial_ports():
    """List machine serial ports."""
    
    # ensure pySerial exists
    try:
        serial
    except NameError:
        logger.error("Cannot search for serial ports without pySerial!")
        raise ImportError ("pySerial not found!")

    ports = []

    # POSIX Environment
    
    if os.name == "posix":
        logger.debug("Linux environment searching /dev...")
        
        # Test for /dev readability
        if os.access("/dev", os.R_OK):
            regex = re.compile("^tty(USB|S)[0-9]+$")
            all_ports = [s for s in os.listdir('/dev') if regex.match(s)]
            logger.debug("Found %d serial ports in /dev." % len(all_ports))

            for port in all_ports:
                port = '/dev/' + port
                if os.access(port, os.R_OK | os.W_OK):
                    ports.append(port)
                    
            logger.info("Found %d available serial ports." % len(ports))

        # Cannot read /dev
        else:
            logger.warning("Insufficient access privileges on /dev.")
            pass
    
    # Non-POSIX system; check for windows-style COMX serial ports.
    if os.name == "nt" or len(ports) == 0:
        for i in range(100):

            # Attempt to create the COM device
            try:
                tmp = serial.Serial(port=i)
                logger.info("Found serial port at COM %d." % i)
                # portstr is deprecated. Use "name" as of pyserial 2.5.
                ports.append(tmp.portstr)
                
            # Catch Non-Existent Device Exception
            except serial.SerialException:
                pass    # Port creation failed; move on.

    # Return the list
    return ports


logger.debug("... clearpath.utils loaded.")
