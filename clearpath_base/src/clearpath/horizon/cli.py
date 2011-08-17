#! /usr/bin/env python 
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
#  File: cli.py
#  Desc: Horizon Interface Demo Python Module & Script
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
# Script Run Method Check



# Check if run as a script
if __name__ == "__main__":
    
    # Check if run with '-m'
    try:
        from .. import utils
        
    # Not run with '-m'
    except:
        
        # Notify of proper run method
        print ("ERROR: clearpath.cli is a module and must be run "\
               "by:\n  python -m clearpath.cli")

        # Exit Error
        import sys
        sys.exit(1)




################################################################################
# Module



## @package clearpath.horizon.demo
#  Horizon Interface Demo Python Module & Script
# 
#  Horizon Basic Command-Line Controller.                                     \n
#  Supported Horizon version(s): 0.1 - 1.0
#
#  @author     Ryan Gariepy
#  @author     Malcolm Robert
#  @date       14/01/10
#  @req        clearpath.utils                                                \n
#              clearpath.horizon                                              \n
#              clearpath.horizon.messages                                     \n
#              clearpath.horizon.payloads                                     \n
#              clearpath.horizon.protocol                                     \n
#              clearpath.horizon.transports                                   \n
#              pyCrypto [http://www.dlitz.net/software/pycrypto/]             \n
#              -- for encryption                                              \n
#              pySerial [http://pyserial.sourceforge.net/]                    \n
#              -- for serial support
#  @version    1.0
#
#  @section USE
#  
#  The intended purpose of this module is to provide a CMD wrapper for the
#  Horizon interface to provide the user with a basic command-line demo
#  that can fully control a Horizon based platform. Further, this module 
#  provides script code to bootstrap the CMD wrapper.
#
#  While the wrapper (HorizonDemo) can be used directly, it is only useful
#  for command-line controls and its use outside of this module is discouraged.
#  To use the wrapper, instantiate and open Horizon, then instantiate the
#  wrapper with the parameters port (the prompt prefix), horizon 
#  (the opened Horizon), and output (filename to output data messages to).
#  Instantiation will automatically poll the platform for information then
#  print to standard out a splash. Then, to enter the interactive shell, call
#  cmdloop. Note that this method will block (calling Horizon methods as needed)
#  until the user exits the interactive shell.
#
#  This script is executed by running
#  'python -m clearpath.horizon.demo'
#  To list the required and optional command-line arguments run
#  'python -m clearpath.horizon.demo --help'
#
#  Various logging messages (debug, warning,...) are logged with the standard
#  Python logger to the log named 'clearpath.horizon.demo'.                   \n
#  Messages are ignored by default; remove the only handler from it and turn
#  on propagation to get messages to propagate.
#  When this module is run as a script, logging messages are turned on according
#  to command-line arguments.
#
#  @section HISTORY
#  Version 0.1 - 0.3 {Ryan Gariepy}
#  - Initial Creation as protocol_demo.py
#
#  Version 0.4 {Malcolm Robert}
#  - Move to horizon.py
#  - Added command-line options / arguments
#  - Added interactive command shell
#  - Added version support
#  - Added Doxygen documentation
#  - Changed version scheme to match Horizon doc
#  - Horizon support for v0.4
#
#  Version 0.5
#  - Move to horizon_demo.py
#  - Added TCP and UDP support
#  - Horizon support for v0.5
#
#  Version 0.6
#  - Move to demo.py
#  - Horizon support for v0.6
#
#  Version 0.7
#  - Horizon support for v0.7
#
#  Version 0.8
#  - Horizon support for v0.8
#
#  Version 1.0
#  - Added Encryption Support
#  - Horizon support for v 0.1 - 1.0
#  - Python 2.6+ & 3.x compatible
#
#  @section License
#  @copydoc public_license
#
"""Horizon Basic Command-Line Controller 

   Copyright © 2010 Clearpath Robotics, Inc.
   All rights reserved
   
   Created: 14/01/10
   Author:  Ryan Gariepy & Malcolm Robert
   Version: 1.0
   """

# Required Clearpath Modules
from .. import utils            # Clearpath Utilities
from .  import Horizon          # Horizon Interface
from .  import logger as horizon_logger
                                # Horizon Interface Logger 
from .  import messages         # Horizon Protocol Message Definition
from .  import payloads         # Horizon Protocol Message Payload Definitions
from .  import protocol         # Horizon Protocol Interface
from .  import transports       # Horizon Transport Definitions

# Required Python Modules
import cmd                      # Interactive Command-Line Sessions
import datetime                 # Date & Time Manipulation
import inspect                  # Detailed Python Manipulation
import logging                  # Logging Utilities
import optparse                 # Command-Line Options Parsing
import os                       # Operating System Capabilities
import re                       # Regular Expressions
import sys                      # Python Interpreter Functionality
import threading                # Python Thread Support
import time                     # System Date & Time
import readline


try:
    import serial               # Serial Port Control
except ImportError:
    pass


# Module Support
## Module Version
__version__  = "1.0"
"""Module Version"""
## SVN Code Revision
__revision__ = "$Revision: 800 $"
"""SVN Code Revision"""



## Message Log
logger = logging.getLogger('clearpath.horizon.demo')
"""Horizon Demo Module Log"""
logger.setLevel(logging.NOTSET)
logger.addHandler(utils.NullLoggingHandler())
logger.propagate = False
logger.debug("Loading clearpath.horizon.demo ...")




################################################################################
# Basic Command-Line Controller



## Horizon Command-Line Controller
#
#  Interactive shell for controlling a Horizon device.
#
#  @see cmd.Cmd
#  @since 0.1
#
#  @pydoc
class HorizonDemo(cmd.Cmd):
    """Horizon Command-Line Controller"""
    
    
    ## Create A Horizon Command-Line Controller
    #
    #  @param  port    The device name to use in the prompt ('horizon:port$')
    #  @param  horizon The Horizon instance to send commands to.
    #                  Must already be opened.
    #  @param  output  Where to send subscription data?
    #                  '' and '/dev/null' indicates nowhere and enables 
    #                  'get_waiting' 
    #
    #  @pydoc
    def __init__(self,port,horizon,output=''):
        """Create A Horizon Command-Line Controller"""
        logger.debug("%s: Instance creation started..." % \
                     self.__class__.__name__)
        
        cmd.Cmd.__init__(self)  # Superclass initialization
        
        # Class Variables
        ## Repetition Commands
        self._commands = []     # Format: tuple([start,freq,tuple([meth,vals])])
        # Commands are repeated from another thread, so we need to synchronize all
        # access to _commands using this lock:
        self._cmd_lock = threading.RLock()
        ## Horizon Interface
        self._horizon = horizon
        ## Subscription Output
        self._output = None
        ## Repeat Thread
        self._repeater = threading.Thread(target = self._repeat)
        ## Repeat Looping?
        self._repeating = False
        
        # Setup CMD
        self.intro = "\n------------------------------\n" \
                       "--          Horizon         --\n" \
                       "-- Clearpath Robotics, Inc. --\n" \
                       "------------------------------\n\n" \
                       "Basic Horizon Interactive Command-Line Controller "\
                       "Type 'help' to get the list of available commands.\n\n"\
                       "%s\n" % self._about()
        self.doc_leader = \
          "\nBasic Horizon Interactive Command-Line Controller \n"\
            "These shell commands are defined internally. "\
            "Type 'help' to see this list.\n"\
            "Type 'help <name>' to find out how to use the command 'name'.\n"\
            "Use 'horizon_demo --doc' to find out more about Horizon in "\
            "general.\n"
        self.prompt = 'horizon:' + port + '$ '
                
        # explore all attributes of horizon for possibilities
        for i in Horizon.__dict__:
            
            # Test doc string and version
            if i[0] != '_': #getattr(self._horizon,i) != None and \
                    #getattr(self._horizon,i).__doc__ != None and \
                    #doc.match(getattr(self._horizon,i).__doc__):
                
                # command Function Wrapper to call _parse_arguments on name
                def tmp(self,line,method=i):
                    self._parse_arguments(method=getattr(self._horizon,method),
                                          arguments=line)

                def tmp_help(self,method=i):
                    self._print_help(method)

                    
                # copy doc string
                tmp.__doc__ = getattr(self._horizon,i).__doc__
                
                # add new command
                setattr(HorizonDemo, "do_"+i, tmp)
                setattr(HorizonDemo, "help_"+i, tmp_help)
                
        # No Output -> additional command
        if output == '' or output == '/dev/null' or output == None:
            setattr(HorizonDemo, "do_get_waiting", self._waiting)
            setattr(HorizonDemo, "complete_get_waiting", self._waiting_complete)
            
        # Output
        else:
            self._output = logging.getLogger("HorizonDemo")
            self._output.setLevel(logging.DEBUG)
            logger_handler = logging.FileHandler(filename=output)
            logger_handler.setFormatter(logging.Formatter("%(message)s"))
            self._output.addHandler(logger_handler)
        
        # Setup Thread
        self._repeater.setDaemon(True)
    
        logger.debug("%s: ...instance creation complete." % \
                     self.__class__.__name__)
                
        
    ## Destroy a Horizon Demo.
    #
    #  Horizon Demo class destructor.                                    \n
    #  Stops the repeat thread if running.
    #
    #  @pydoc
    def __del__(self):
        """Destroy a Horizon Demo."""
        logger.debug("%s: Instance destruction started..." % \
                     self.__class__.__name__)
        
        # Stop Thread
        self._repeating = False
        
        logger.debug("%s: ...instance destruction complete." % 
                     self.__class__.__name__)
                
        
    ## Get Horizon About String
    #
    #  @return string of Horizon information
    #
    #  @pydoc
    def _about(self):
        """Get Horizon About String"""
        
        result = ""
        
        # Get info
        try:
            # Transport Information
            result += '-- Horizon  --\n'\
                      'Transport:     %s\n'\
                      'Device:        %s\n'\
                      'Version:       %d.%d\n' % \
                      (self._horizon._protocol._transport.__class__.__name__, 
                       str(self._horizon._protocol._transport),
                       self._horizon.version[0],self._horizon.version[1])
            
            # Platform Name
            name = self._horizon.request_platform_name()
            result += '-- Platform --\n'\
                      'Name:          %s\n' % (name.name)
                      
            # Platform Information
            info = self._horizon.request_platform_info()
            result += 'Model:         %s\n'\
                      'Revision:      %d\n'\
                      'Serial Number: %010d\n' % (info.model, info.revision, info.serial)

            # Platform Firmware
            firm = self._horizon.request_firmware_info()
            result += '-- Firmware --\n'\
                      'Horizon:       %d.%d\n'\
                      'Version:       %d.%d\n'\
                      'Last Updated:  %s\n' % (firm.version[0], firm.version[1],
                                               firm.firmware[0], firm.firmware[1],
                                               firm.written.strftime("%I:%M %p - %B %d, %Y"))
        
            # Platform Status
            syst = self._horizon.request_system_status()
            sec = syst.uptime
            mil = sec % 1000
            sec = (sec - mil) / 1000
            min = sec
            sec = min % 60
            min = (min - sec) / 60
            hou = min
            min = hou % 60
            hou = (hou - min) / 60
            day = hou
            hou = day % 24
            day = (day - hou) / 24
            result += '-- Status   --\n'\
                      'Uptime:        %02ddays %02dh %02dm %02ds %03dms\n'\
                      % (day, hou, min, sec, mil)
                          
            # Platform Power Status            i
            powe = self._horizon.request_power_status()
            result += '-- Status   --\n'
            result += 'Power:         %02.2f%%\n' % ((sum(powe.charges,0.0)/
                                                        len(powe.charges)))
                
            # Platform Safety Status
            safe = self._horizon.request_safety_status()
            emer = ''
            if safe.has_emergency_stop():
                emer = 'EMERGENCY STOP'
            elif safe.flags & 0x0002 > 0:
                emer = 'PAUSED'
            result += 'Mode:          %s\n' % (emer)
                  
        # Error(s) encountered
        except NotImplementedError as ex:
            logger.error(str(ex))
            result = "ERROR: Failed to obtain device info!"
        except utils.SubscriptionError as ex:
            logger.error(str(ex))
            result = "ERROR: Failed to obtain device info!"
        except utils.TimeoutError as ex:
            logger.error(str(ex))
            result = "ERROR: Failed to obtain device info!"
        except utils.TransportError as ex:
            logger.error(str(ex))
            result = "ERROR: Failed to obtain device info!"
        except ValueError as ex:
            logger.error(str(ex))
            result = "ERROR: Failed to obtain device info!"
            
        # Return
        return result[:-1]
                    
        
    ## Extract Arguments
    #
    #  Extract arguments from a Line using whitespace as separators.
    #
    #  @param  arguments The line to extract from
    #  @return List of string arguments, empty if none
    #
    #  @pydoc
    def _extract_arguments(self, arguments):
        """Extract arguments from a Line."""
        
        # Prepare regex(s)
        quote = '(?:(?P<quo>[\"\'])(.+?)(?P=quo))'
        escapes = '(?:(?:\\\\\\s)|(?:\\\\a)|(?:\\\\b)|(?:\\\\f)|(?:\\\\n)|' \
                    '(?:\\\\r)|(?:\\\\t)|(?:\\\\v)|(?:\\\\\\\\))'
        nonwhite = '(?:[^\\t\\n\\r\\f\\v\\\\\\ ])'
        token = '(?:(?:'+escapes+'|'+nonwhite+')+)'
        block = '('+quote+'|'+token+')'
        line = '^'+token+'(?:\\s+'+token+')*$'
        parser = re.compile(line)
        matcher = re.compile(block)
        replacer = re.compile('\\\\([^\\\\])')
        
        # Format & Parse the provided line
        arguments = arguments.strip()
        match = parser.match(arguments)
        params = []
        if match != None:
            match = matcher.findall(arguments)
            for sub in match:
                rsub = sub[0]
                
                # Extract from Quotes
                if sub[1] != '':
                    rsub = sub[2]
                
                # Replace Escapes (if not quoted in ')
                if sub[1] != "'":
                    rsub = rsub.replace('\\a','\a')
                    rsub = rsub.replace('\\b','\b')
                    rsub = rsub.replace('\\f','\f')
                    rsub = rsub.replace('\\n','\n')
                    rsub = rsub.replace('\\r','\r')
                    rsub = rsub.replace('\\t','\t')
                    rsub = rsub.replace('\\v','\v')
                    rsub = replacer.sub(r'\1',rsub)
                    rsub = rsub.replace('\\\\','\\')
                
                # Add To List
                params.append(rsub)
                
        # Return result
        return params
    

    ## Convert Arguments to their Proper Types
    # 
    #  Converts arguments based on given default values.                      \n
    #  Prints argument errors to stdout.
    #
    #  @param  params The arguments to convert
    #  @param  method The method to convert for
    #  @return Dictionary of values, Empty if none, None if error encountered
    #
    #  @pydoc
    def _arguments_to_values(self, params, method):
        """Convert arguments to their proper types."""
        
        # Get a list of the method's arguments and their default values
        args = inspect.getargspec(method)
        if 'handler' in args[0]:
            args[0].remove('handler')
        if 'subscription' in args[0]:
            args[0].remove('subscription')
        values = {}
        
        # Prepare Boolean regex
        yes = re.compile('^(?:(?:[yY][eE][sS])|(?:[tT][rR][uU][eE])|(?:1))$')
        no = re.compile('^(?:(?:[nN][oO])|(?:[fF][aA][lL][sS][eE])|(?:0))$')
        
        # Check Number of Argmuents
        if (params == None and len(args[0]) > 1) or \
                (len(params) != len(args[0]) - 1 and (len(args[0]) == 1 or 
                            type(args[3][-2]) != dict)):
            print(("ERROR: %s invalid number of arguments" % method.__name__))
            return None
        
        # Test the provided arguments
        for i,param in enumerate(params):
            
            # Dictionary
            if type(args[3][i]) == dict:
                values[args[0][i+1]] = {}
                
                # Steal rest of arguments
                for j in range(i,len(params)):
                    tokens = params[j].split(':')
                    if len(tokens) != 2:
                        print(("ERROR: %s argument %d: must be key:value" % (
                                                        method.__name__, (j+1))))
                        return None
                    key = None
                    if type(list(args[3][i].keys())[0]) == str:
                        key = tokens[0]
                    elif type(list(args[3][i].keys())[0]) == float:
                        try:
                            key = float(tokens[0])
                        except ValueError:
                            print(("ERROR: %s argument %d: key must be a float"\
                                    % (method.__name__, (j+1))))
                            return None
                    elif sys.version_info[0] < 3 and \
                            type(list(args[3][i].keys())[0]) == int:
                        try:
                            key = int(tokens[0])
                        except ValueError:
                            print(("ERROR: %s argument %d: key must be a long"\
                                    % (method.__name__,(j+1))))
                            return None
                    elif type(list(args[3][i].keys())[0]) == int:
                        try:
                            key = int(tokens[0])
                        except ValueError:
                            print(("ERROR: %s argument %d: key must be an int"\
                                    % (method.__name__,(j+1))))
                            return None
                    elif type(list(args[3][i].keys())[0]) == bool:
                        if yes.match(tokens[0]):
                            key = True
                        elif no.match(tokens[0]):
                            key = False
                        else:
                            print(("ERROR: %s argument %d: key must be a "\
                                   "boolean" % (method.__name__,(j+1))))
                            return None
                    print (key)
                    if type(args[3][i][list(args[3][i].keys())[0]]) == str:
                        values[args[0][i+1]][key] = tokens[1]
                    elif type(args[3][i][list(args[3][i].keys())[0]]) == float:
                        try:
                            values[args[0][i+1]][key] = float(tokens[1])
                        except ValueError:
                            print(("ERROR: %s argument %d: key must be a float"\
                                    % (method.__name__, (j+1))))
                            return None
                    elif sys.version_info[0] < 3 and \
                            type(args[3][i][list(args[3][i].keys())[0]]) == int:
                        try:
                            values[args[0][i+1]][key] = int(tokens[1])
                        except ValueError:
                            print(("ERROR: %s argument %d: key must be a long"\
                                    % (method.__name__,(j+1))))
                            return None
                    elif type(args[3][i][list(args[3][i].keys())[0]]) == int:
                        try:
                            values[args[0][i+1]][key] = int(tokens[1])
                        except ValueError:
                            print(("ERROR: %s argument %d: key must be an int"\
                                    % (method.__name__,(j+1))))
                            return None
                    elif type(args[3][i][list(args[3][i].keys())[0]]) == bool:
                        if yes.match(tokens[1]):
                            values[args[0][i+1]][key] = True
                        elif no.match(tokens[1]):
                            values[args[0][i+1]][key] = False
                        else:
                            print(("ERROR: %s argument %d: key must be a "\
                                   "boolean" % (method.__name__,(j+1))))
                            return None
                print((values[args[0][i+1]]))
                break
            
            # String
            elif type(args[3][i]) == str:
                values[args[0][i+1]] = param
                
            # Float
            elif type(args[3][i]) == float:
                try:
                    values[args[0][i+1]] = float(param)
                except ValueError:
                    print(("ERROR: %s argument %d: value must be a float" % (
                                                        method.__name__, (i+1))))
                    return None
            
            # Long
            elif sys.version_info[0] < 3 and type(args[3][i]) == int:
                try:
                    values[args[0][i+1]] = int(param)
                except ValueError:
                    print(("ERROR: %s argument %d: value must be a long" % (
                                                        method.__name__,(i+1))))
                    return None
            
            # Int
            elif type(args[3][i]) == int:
                try:
                    values[args[0][i+1]] = int(param)
                except ValueError:
                    print(("ERROR: %s argument %d: value must be an int" % (
                                                        method.__name__,(i+1))))
                    return None
            
            # Boolean
            elif type(args[3][i]) == bool:
                if yes.match(param):
                    values[args[0][i+1]] = True
                elif no.match(param):
                    values[args[0][i+1]] = False
                else:
                    print(("ERROR: %s argument %d: value must be a boolean" % (
                                                        method.__name__,(i+1))))
                    return None
            
        # Result(s)
        return values
        
    
    ## Parse a Line of Arguments
    #
    #  Extracts arguments from a line for the given method.                   \n
    #  Runs the method.                                                       \n
    #  Outputs anything that is returned.                                     \n
    #  ValueErrors are treated as command argument errors.
    #
    #  @param method    The method to execute
    #  @param arguments The string containing the whitespace separated arguments 
    #
    #  @pydoc 
    def _parse_arguments(self,method,arguments):
        """Parse a Line of Arguments"""
        
        # Get passed arguments
        params = self._extract_arguments(arguments)
        
        # Convert to values
        values = self._arguments_to_values(params, method)
        if values == None: return
        
        # Run the method
        try:
            result = method(**values)
            if result: print((result.print_format()))
        except NotImplementedError as ex:
            print(("ERROR: %s is not supported on the platform!" % \
                method.__name__))
            logger.debug(ex)
        except utils.TransportError as ex:
            print(("ERROR: Transport send for %s failed!" % \
                method.__name__))
            logger.debug(ex)
        except ValueError as ex:
            print(("ERROR: " + str(ex)))
            logger.debug(ex)
        except utils.TimeoutError as ex:
            print(("ERROR: Timeout occured waiting for %s completion!" % \
                method.__name__))
            logger.debug(ex)


    ## Prints a usage message for a command
    #
    def _print_help(self, method):
        helpstr = 'usage: ' + method
        argspec = inspect.getargspec(getattr(self._horizon, method))

        # Inspect the horizon function argspec to figure out args for usage
        for i in range(len(argspec.args)):
            # Arg names that are not user specified:
            if argspec.args[i] in ['self', 'subscription', 'handler'] :
                continue

            if i < (len(argspec.args) - len(argspec.defaults)) :
                # arg has no default, so just print its name
                helpstr += ' <' + argspec.args[i] + '>'
            else:
                # arg has a default, print the type
                argtype = str(type(argspec.defaults[i-len(argspec.args)]))
                # argtype has form "<type 'typename'>", we want to extract "typename"
                argtype = argtype[argtype.index('\'')+1:]
                argtype = argtype[:argtype.index('\'')]
                helpstr += ' <' + argtype + ' ' + argspec.args[i] + '>'

        print(helpstr)
            
            
    ## Output Subscription Data
    #
    #  Writes subscription data to the output file.
    #
    #  @param payload The received payload
    #
    #  @pydoc
    def _handler(self, payload):
        """Output Subscription Data"""
        
        # Format Timestamp
        sec = payload.timestamp
        mil = sec % 1000
        sec = (sec - mil) / 1000
        min = sec
        sec = min % 60
        min = (min - sec) / 60
        hou = min
        min = hou % 60
        hou = (hou - min) / 60
        day = hou
        hou = day % 24
        day = (day - hou) / 24
        
        if payload.error != None:
            self._output.debug('-- %02ddays %02dh %02dm %02ds %03dms --\n%s' % (
                day, hou, min, sec, mil, str(payload.error)))
        else:
            self._output.debug('-- %02ddays %02dh %02dm %02ds %03dms --\n%s' % (
                day, hou, min, sec, mil, payload.print_format()))

    
    ## Command Repeater Loop
    #  
    #  Repeater Thread method. Loops on _repeating waiting to send commands.
    #
    #  @pydoc
    def _repeat(self):
        """Command Repeater Loop"""
        logger.debug("%s: Entering repeat loop..." % self.__class__.__name__)
        
        # Continuous Loop
        while self._repeating:
            
            # Check for no commands -> stop thread
            if len(self._commands) == 0:
                self._repeating = False
                self._repeater = threading.Thread(target = self._repeat)
                self._repeater.setDaemon(True)
                break
            
            # Update Timestamp
            timestamp = 0
            t = datetime.datetime.today()
            timestamp = t.microsecond/1000 + t.second*1000 + \
                  t.minute*60*1000 + t.hour*60*60*1000 + t.day*24*60*60*1000
            while timestamp > 4294967295: timestamp -= 4294967295
            if sys.version_info[0] > 2:
                timestamp = int(timestamp)
            else:
                timestamp = int(timestamp)
                
            # Check Commands
            self._cmd_lock.acquire()
            try:
                for cmd in self._commands:
                    diff = 0
                    if timestamp > cmd[0]:
                        diff = timestamp - cmd[0]
                    else:
                        diff = 4294967295 - cmd[0] + timestamp
                    if diff % int(1000/cmd[1]) == 0:
                        # Run command
                        try:
                            cmd[2][0](**cmd[2][1])
                        except Exception:
                            pass
            finally:
                self._cmd_lock.release()
            
            # Release Processor
            time.sleep(0.001)
        
        logger.debug("%s: ...repeat loop exited." % self.__class__.__name__)
            
         
    ## Get Waiting Data
    #
    #  Outputs data retrieved from subscriptions.
    #
    #  @param line The rest of the line
    #
    #  @pydoc
    def _waiting(self, line):
        """syntax: get_waiting [<request>]\n""" \
        """-- Show any data received from a subscription on 'request'.\n""" \
        """   If request is not specified then list all data."""
        
        # Get passed arguments
        params = self._extract_arguments(line)
        
        # Check number of arguments
        if len(params) > 1:
            print ("ERROR: too many arguments!")
            return
        
        # Check request
        request = None
        if len(params) == 1:
            if not params[0] in self.completenames('request_'):
                print ("ERROR: Unsupported / Invalid command!")
                return
            method = None
            try:
                method = getattr(self._horizon, params[0])
            except AttributeError:
                print ("ERROR: Unsupported / Invalid command!")
                return
            args = inspect.getargspec(method)
            if not 'subscription' in args[0]:
                print(("ERROR: %s does not support subscriptions!" % params[1]))
                return
            request = params[0]
            
        # Get Data
        data = []
        try:
            data = self._horizon.get_waiting_data(request=request)
        except Exception as ex:
            print(("Error: unexpected error encountered!\n%s" % str(ex)))
        
        # Output Data
        if len(data) == 0:
            if request == None:
                print ("No data.")
            else:
                print(("No data for %s." % request))
        else:
            for payload in data:
                if isinstance(payload[1], payloads.Null):
                    continue
                
                # Format Timestamp
                sec = payload[2]
                mil = sec % 1000
                sec = (sec - mil) / 1000
                min = sec
                sec = min % 60
                min = (min - sec) / 60
                hou = min
                min = hou % 60
                hou = (hou - min) / 60
                day = hou
                hou = day % 24
                day = (day - hou) / 24
          
                print(('-- %02ddays %02dh %02dm %02ds %03dms --' % (
                            day, hou, min, sec, mil)))
                print((payload[1].print_format()))
    
    
    ## Get Waiting Command Tab Completion
    #
    #  Calculates a list of possible paramters for get_waiting based on the 
    #  entered prefix.
    #
    #  @param  text   The last parameter before tab
    #  @param  line   The entire line of text
    #  @param  begidx ???
    #  @param  endidx ???
    #  @return List of string possibilities
    #
    #  @pydoc
    def _waiting_complete(self, text, line, begidx, endidx):
        """Get Waiting Command Tab Completion"""
        
        # Get passed arguments
        params = self._extract_arguments(line)
        
        # Complete Command
        if (len(params) == 2 and len(text.strip()) > 0) or \
                (len(params) == 1 and len(text.strip()) == 0):
            cmds = self.completenames(text)
            for cmd in cmds[:]:
                if not cmd.startswith('request_'):
                    cmds.remove(cmd)
        
        # Resultant List
        return cmds
            
           
    ## Empty Line Action
    #
    #  Does nothing when an empty line is entered rather than the default
    #  'repeat last command'.
    #
    #  @pydoc
    def emptyline(self):
        """Empty Line Action"""
        
        pass    # Do Nothing
    
    
    ## List all Commands
    #
    #  Overrides Cmd default to categorize Horizon messages.
    #
    #  @see Cmd.do_help
    #
    #  @pydoc
    def do_help(self, arg):
        """syntax: help [<command>]\n""" \
        """-- Show the help text for the requested command.\n""" \
        """   If command is not specified then list all commands."""
        
        # Use default Cmd processing for command help
        if arg:
            return cmd.Cmd.do_help(self, arg)

        # List all commands (based on cmd.Cmd.do_help)
        else:
            cmds_cmd = []   # Commands
            cmds_req = []   # Requests
            cmds_hor = []   # Horizon Other
            cmds     = []   # Shell Control
            
            # Get commands
            commands = self.get_names()
            commands.sort()
            
            # Categorize Commands
            prev = ''
            for command in commands:
                if command[:3] == 'do_':
                    if command == prev:
                        continue
                    prev = command
                    command = command[3:]
                    if len(command) > 8 and command[:8] == 'request_':
                        cmds_req.append(command)
                    elif len(command) > 4 and command[:4] == 'set_':
                        cmds_cmd.append(command)
                    else:
                        try:
                            getattr(self._horizon,command)
                            cmds_hor.append(command)
                        except AttributeError:
                            if command == 'subscribe' or command == 'about' or \
                              command == 'get_waiting' or command == 'get_time':
                                cmds_hor.append(command)
                            else:
                                cmds.append(command)
            
            # Output Lists
            self.stdout.write("%s\n"%str(self.doc_leader))
            self.print_topics('Horizon', cmds_hor, 15,80)
            self.print_topics('Horizon Commands', cmds_cmd,15,80)
            self.print_topics('Horizon Requests', cmds_req, 15,80)
            self.print_topics('Interactive Shell', cmds, 15,80)
    
    
    ## About Horizon Device.
    #
    #  Get information on the Horizon device.
    #
    #  @param     arguments  Line entered after 'about'
    #
    #  @pydoc
    def do_about(self, arguments):
        """syntax: about \n""" \
        """-- Get general info about the Horizon device."""
        
        # Get passed arguments
        params = self._extract_arguments(arguments)
        
        # Check arguments
        if len(params) > 0:
            print ("ERROR: Too many arguments!")
            return
        
        # Print Information
        print((self._about()))
    
    
    ## Horizon Device Alive?
    #
    #  Check if the Horizon device is alive.
    #
    #  @param     arguments  Line entered after 'about'
    #
    #  @pydoc
    def do_is_alive(self, arguments):
        """syntax: is_alive \n""" \
        """-- Check if the Horizon device is alive."""
        
        # Get passed arguments
        params = self._extract_arguments(arguments)
        
        # Check arguments
        if len(params) > 0:
            print ("ERROR: Too many arguments!")
            return
        
        # Print Information
        print((self._horizon.alive))
    
    
    ## Horizon Device Time
    #
    #  Get the Horizon device time.
    #
    #  @param     arguments  Line entered after 'get_time'
    #
    #  @pydoc
    def do_get_time(self, arguments):
        """syntax: get_time \n""" \
        """-- Get the Horizon device time."""
        
        # Get passed arguments
        params = self._extract_arguments(arguments)
        
        # Check arguments
        if len(params) > 0:
            print ("ERROR: Too many arguments!")
            return
        
        # Get information
        try:
            sec = self._horizon.device_time
            
            # Format time
            mil = sec % 1000
            sec = (sec - mil) / 1000
            min = sec
            sec = min % 60
            min = (min - sec) / 60
            hou = min
            min = hou % 60
            hou = (hou - min) / 60
            day = hou
            hou = day % 24
            day = (day - hou) / 24
                    
            # Output time        
            print(('Time: %02ddays %02dh %02dm %02ds %03dms' \
                   % (day, hou, min, sec, mil)))
                  
        # Error(s) encountered
        except NotImplementedError:
            print ("ERROR: Failed to obtain device time!")
        except utils.SubscriptionError:
            print ("ERROR: Failed to obtain device time!")
        except utils.TimeoutError:
            print ("ERROR: Failed to obtain device time!")
        except utils.TransportError:
            print ("ERROR: Failed to obtain device time!")
        except ValueError:
            print ("ERROR: Failed to obtain device time!")
    
    
    ## Subscribe to Command.
    #
    #  Subscribe to receive request data at a specified interval.
    #
    #  @param     arguments  Line entered after 'subscribe'
    #
    #  @pydoc
    def do_subscribe(self, arguments):
        """syntax: subscribe <frequency> <command>\n""" \
        """-- Receive data at <frequency> Hz.\n""" \
        """   <frequncy> = off | (1-65534)\n""" \
        """   <command> = request command with pertinent arguments"""
        
        # Get passed arguments
        params = self._extract_arguments(arguments)
        
        # Check Number of Argmuents
        if params == None or len(params) < 2:
            print ("ERROR: subscribe invalid number of arguments")
            return None
        
        # Check frequency
        freq = 0xFFFF
        if params[0] != 'off' and ((not params[0].isdigit()) or \
                                int(params[0]) < 1 or int(params[0]) > 65534):
            print ("ERROR: Invalid frequency!")
            return
        elif params[0].isdigit():
            freq = int(params[0])
        
        # Check Command
        if not params[1] in self.completenames('request_'):
            print ("ERROR: Unsupported / Invalid command!")
            return
        method = None
        try:
            method = getattr(self._horizon, params[1])
        except AttributeError:
            print ("ERROR: Unsupported / Invalid command!")
            return
        args = inspect.getargspec(method)
        if not 'subscription' in args[0]:
            print(("ERROR: %s does not support subscriptions!" % params[1]))
            return
        
        # Check Command Argmuents
        values = self._arguments_to_values(params[2:], method)
        if values == None:
            return
        values['subscription'] = freq
            
        # Outputting?
        if self._output != None:
        
            # Manage Handlers - Add
            if not freq == 0xFFFF:
                self._horizon.add_handler(handler=self._handler, 
                                          request=params[1])
            
            # Manage Handlers - Remove
            else:
                self._horizon.remove_handler(handler=self._handler, 
                                             request=params[1])
    
        # Run the method
        try:
            res = method(**values)
            if res != None: 
                if isinstance(res, payloads.Payload):
                    print((res.print_format()))
                else:
                    print (res)
        except NotImplementedError:
            print(("ERROR: %s is not supported on the platform!" % \
                method.__name__))
        except utils.SubscriptionError:
            print(("ERROR: %s Subscription failed due to frequency!" % \
                method.__name__)) 
        except utils.TimeoutError:
            print(("ERROR: Timeout occured waiting for %s subscription!" % \
                method.__name__)) 
        except utils.TransportError:
            print(("ERROR: Transport send for %s subscription failed!" % \
                method.__name__))
        except ValueError as ex:
            print(("ERROR: " + str(ex)))
    
    
    ## Subscribe Command Tab Completion
    #
    #  Calculates a list of possible paramters for subscribe based on the 
    #  entered prefix.
    #
    #  @param  text   The last parameter before tab
    #  @param  line   The entire line of text
    #  @param  begidx ??? - Passed on to other functions
    #  @param  endidx ??? - Passed on to other functions
    #  @return List of string possibilities
    #
    #  @pydoc
    def complete_subscribe(self, text, line, begidx, endidx):
        """Subscribe Command Tab Completion"""
        
        # Get passed arguments
        params = self._extract_arguments(line)
        
        # Frequency List
        if len(params) == 1 and len(text.strip()) == 0:
            cmds = list(map(str,list(range(1,65535))))
            cmds.insert(0, 'off')
            return cmds
        
        # Complete Frequency
        elif len(params) == 2 and len(text.strip()) > 0:
            params[1] = params[1].rstrip('0')
            if params[1] == 'off' or params[1] == 'of' or params[1] == 'o':
                return ['off']
            elif params[1].isdigit() and int(params[1]) < 65535 and \
                    int(params[1]) > -1:
                cmds = [params[1]]
                last = [params[1]]
                if len('65535') - len(params[1]) > 1:
                    for multiple in range(1,len('65535') - len(params[1])):
                        tmp = []
                        for cmd in last:
                            for i in range(0,10):
                                tmp.append(cmd+str(i))
                        cmds += tmp
                        last = tmp
                if len('65535') - len(params[1]) > 0:
                    for cmd in last:
                        for i in range(0,10):
                            if int(cmd+str(i)) < 65535:
                                cmds.append(cmd+str(i))
                return cmds
            else:
                return []
        
        # Complete Command
        elif (len(params) == 3 and len(text.strip()) > 0) or \
                (len(params) == 2 and len(text.strip()) == 0):
            cmds = self.completenames(text)
            for cmd in cmds[:]:
                if not cmd.startswith('request_'):
                    cmds.remove(cmd)
        
        # Complete Arguments
        else:
            complete = None
            try:
                complete = getattr(self, 'complete_' + params[1])
            except AttributeError:
                complete = self.completedefault
            cmds = complete(text, ' '.join(params[1:]), begidx, endidx)
        
        # Resultant List
        return cmds
    
   
    ## Manipulate repeated commands.
    #
    #  @param arguments  Line entered after 'repeat'
    #
    #  @pydoc
    def do_repeat(self, arguments):
        """usage: repeat [list | off [n] | [frequency command]]\n\n""" \
        """Manipulate repeated commands\n\n""" \
        """Options: \n""" \
        """list ................. Print a numbered list of active repeat commands\n""" \
        """off [n] .............. Turn off a given repeat command.  [n] is the number provided\n""" \
        """                       by the list command, the most recent command is removed if  \n""" \
        """                       [n] is not specified.\n""" \
        """[frequency] [command]  Cause [command] to be executed at [frequency]\n""" \

        arguments = arguments.strip()

        if arguments == "list":
            self._list_repeats()
        elif arguments.startswith("off"):
            self._remove_repeat(arguments)
        else:
            self._setup_repeat(arguments)
  
    ## List active repeat commands
    def _list_repeats(self):
        self._cmd_lock.acquire()
        try:
            if len(self._commands) == 0:
                print("No repeat commands are active.")
                return

            for i in range(len(self._commands)):
                # _commands format is tuple(timestamp, freq, tuple(method, dict(args)))
                cmd_str = "{0}: [{1} Hz]".format(i, self._commands[i][1])
                cmd_str += " " + self._commands[i][2][0].func_name
                for name in self._commands[i][2][1]:
                    cmd_str += " {0}={1}".format(name, self._commands[i][2][1][name] ) 
                print(cmd_str)
        finally:
            self._cmd_lock.release()

    ## Remove an active repeat command
    #  
    #  @param arguments Command line arguments after 'repeat', assumed to be stripped.
    def _remove_repeat(self, arguments):
        self._cmd_lock.acquire()
        try:
            if len(self._commands) == 0:
                print("No repeat commands are active.")
                return
            
            # Remove last item, by default
            rem_inx = len(self._commands) - 1;

            # Parse & validate arguments
            if arguments != "off":
                # String is stripped.  It will always start with 'off', but 
                # may have an index argument we need to parse out.  Do so:
                try:
                    rem_inx = int(arguments[3:])
                except ValueError:
                    print("Index argument is not a valid integer.")
                    return
                if (rem_inx < 0) or (rem_inx >= len(self._commands)):
                    print("Index argument out of range.")
                    return

            # We have a valid index to remove.  Now, remove it.
            del self._commands[rem_inx]

        finally:
            self._cmd_lock.release()
       

    ## Subscribe to repeat a command at a specified interval.
    #
    #  @param     arguments  Line entered after 'repeat'
    #
    #  @pydoc
    def _setup_repeat(self, arguments):
        # Get passed arguments
        params = self._extract_arguments(arguments)
        
        # Check Number of Argmuents
        if params == None or len(params) < 2:
            print ("ERROR: repeat: invalid number of arguments")
            return None
        
        # Check frequency
        freq = 0xFFFF
        if params[0] != 'off' and ((not params[0].isdigit()) or \
                                int(params[0]) < 1 or int(params[0]) > 65534):
            print ("ERROR: Invalid frequency!")
            return
        elif params[0].isdigit():
            freq = int(params[0])
        
        # Check Command
        if not params[1] in self.completenames('set_'):
            print ("ERROR: Unsupported command!")
            return
        method = None
        try:
            method = getattr(self._horizon, params[1])
        except AttributeError:
            print ("ERROR: Invalid command!")
            return
        
        # Check Command Argmuents
        values = self._arguments_to_values(params[2:], method)
        if values == None:
            return

        if freq == 0xFFFF:
            print("Repeat commands may be disabled with 'repeat off [n]'")
            return
    
        # Run the method
        try:
            method(**values)
        except NotImplementedError:
            print(("ERROR: %s is not supported on the platform!" % \
                method.__name__))
        except utils.SubscriptionError:
            print(("ERROR: %s Subscription failed due to frequency!" % \
                method.__name__)) 
        except utils.TimeoutError:
            print(("ERROR: Timeout occured waiting for %s subscription!" % \
                method.__name__)) 
        except utils.TransportError:
            print(("ERROR: Transport send for %s subscription failed!" % \
                method.__name__))
        except ValueError as ex:
            print(("ERROR: " + str(ex)))
        
        # Frequency
        
        # Update Timestamp
        timestamp = 0
        t = datetime.datetime.today()
        timestamp = t.microsecond/1000 + t.second*1000 + \
              t.minute*60*1000 + t.hour*60*60*1000 + t.day*24*60*60*1000
        while timestamp > 4294967295: timestamp -= 4294967295
        timestamp = int(timestamp)
        
        # Add new
        self._cmd_lock.acquire()
        self._commands.append(tuple([timestamp,freq,
                                     tuple([method,values])]))
        self._cmd_lock.release()
        
        # Start Thread
        if self._repeating == False:
            self._repeating = True
            self._repeater.start()
            
    
    ## Repeat Command Tab Completion
    #
    #  Calculates a list of possible paramters for repeat based on the 
    #  entered prefix.
    #
    #  @param  text   The last parameter before tab
    #  @param  line   The entire line of text
    #  @param  begidx ??? - Passed on to other functions
    #  @param  endidx ??? - Passed on to other functions
    #  @return List of string possibilities
    #
    #  @pydoc
    def complete_repeat(self, text, line, begidx, endidx):
        """Repeat Command Tab Completion"""
        
        # Get passed arguments
        params = self._extract_arguments(line)
        
        # Frequency List
        if len(params) == 1 and len(text.strip()) == 0:
            cmds = list(map(str,list(range(1,65535))))
            cmds.insert(0, 'off')
            return cmds
        
        # Complete Frequency
        elif len(params) == 2 and len(text.strip()) > 0:
            params[1] = params[1].rstrip('0')
            if params[1] == 'off' or params[1] == 'of' or params[1] == 'o':
                return ['off']
            elif params[1].isdigit() and int(params[1]) < 65535 and \
                    int(params[1]) > -1:
                cmds = [params[1]]
                last = [params[1]]
                if len('65535') - len(params[1]) > 1:
                    for multiple in range(1,len('65535') - len(params[1])):
                        tmp = []
                        for cmd in last:
                            for i in range(0,10):
                                tmp.append(cmd+str(i))
                        cmds += tmp
                        last = tmp
                if len('65535') - len(params[1]) > 0:
                    for cmd in last:
                        for i in range(0,10):
                            if int(cmd+str(i)) < 65535:
                                cmds.append(cmd+str(i))
                return cmds
            else:
                return []
        
        # Complete Command
        elif (len(params) == 3 and len(text.strip()) > 0) or \
                (len(params) == 2 and len(text.strip()) == 0):
            cmds = self.completenames(text)
            for cmd in cmds[:]:
                if not cmd.startswith('set_'):
                    cmds.remove(cmd)
        
        # Complete Arguments
        else:
            complete = None
            try:
                complete = getattr(self, 'complete_' + params[1])
            except AttributeError:
                complete = self.completedefault
            cmds = complete(text, ' '.join(params[1:]), begidx, endidx)
        
        # Resultant List
        return cmds
    
    
    ## Exit the Interactive Shell.
    #
    #  Stop the command loop
    #
    #  @param     self This object
    #  @param     arg  Line entered after 'exit'
    #  @return    True Forces end of loop
    #
    #  @pydoc
    def do_quit(self, arg):
        """syntax: quit""" \
        """-- Exit the Interactive Shell."""
        
        # Exit
        return True
        
        
    ## Exit the Interactive Shell.
    #
    #  Stop the command loop
    #
    #  @param     self This object
    #  @param     arg  Line entered after 'exit'
    #  @return    True Forces end of loop
    #
    #  @pydoc
    def do_exit(self, arg):
        """syntax: exit""" \
        """-- Because quit just isn't enough."""
        
        # Exit
        return True
        
        
    ## Exit the Interactive Shell.
    #
    #  Stop the command loop
    #
    #  @param     self This object
    #  @param     arg  Line entered after 'exit'
    #  @return    True Forces end of loop
    #
    #  @pydoc
    def do_EOF(self, arg):
        """syntax: ^d""" \
        """-- Because quit and exit just aren't enough."""
        
        # Spacer
        print ("exit")
        
        # Exit
        return True
    

logger.debug("... clearpath.horizon.demo loaded.")




################################################################################
# Script Code


## Bootstrap the basic Horizon Demo.
#
#  Main Program Set-up for Basic Command-Line Interface
#  - Gets a list of serial ports
#  - Parses Command-Line Options
#  - Initializes the Horizon Interface
#  - Launches CMD control Loop
#  - Cleans-up after completion
#
#  @pre    OS must be Posix compliant or Windows NT
#  @pre    Posix: Read access on /dev or full access of Serial ports
#  @pre    Windows: Full access of COM ports
#
#  @pydoc
def __main():
    """Bootstrap the basic Horizon Demo."""
    
    # Setup Logger
    while len(horizon_logger.handlers) > 0: 
        horizon_logger.removeHandler(horizon_logger.handlers[0])
    while len(logger.handlers) > 0: 
        logger.removeHandler(logger.handlers[0])
    logger_handler = logging.StreamHandler()
    logger_handler.setFormatter(
                            logging.Formatter("%(levelname)s: %(message)s"))
    horizon_logger.addHandler(logger_handler)
    horizon_logger.setLevel(logging.WARNING)
    logger.propagate = True
    
    # Check verbosity
    ver_re = re.compile(r'^-v$')
    for arg in sys.argv[1:]:
        if ver_re.match(arg):
            horizon_logger.setLevel(logging.INFO)
            logger.info("Verbose mode entered.")
            break
    
    # Get list of available serial ports
    logger.info("Looking for serial ports...")
    ports = []
    try:
        serial
        ports = utils.list_serial_ports()
    except OSError as ex:
        logging.critical(ex)
        sys.exit(1)
    except IOError as ex:
        logging.critical(ex)
        sys.exit(1)
    except NameError:
        logger.warning("Serial not supported. Please install pySerial.")
    if len(ports) > 0:
        ports.sort()
        logger.info("...located %d serial ports: %s" %(len(ports),
                                                       ', '.join(ports)))
        
    # Get Command-Line Options
    logger.info("Parsing command-line arguments...")
    usage = 'usage: python -m clearpath.horizon.demo [<device>] [options]\n\n'
    try:
        serial
        if len(ports) == 0: raise NameError("")
        usage = usage + '<device>  = <serial> | (tcp | udp):<address>\n' + \
                        '<serial>  = ' + ' | '.join(ports) + '\n'
    except NameError:
        usage = usage + '<device>  = (tcp | udp):<address>\n'
    usage = usage + '<address> = <host>:<port>\n' + \
                    '<host>    = computer hostname or IP address\n' + \
                    '<port>    = computer port number (1-65535)'
    desc = 'Horizon Basic Command-Line Controller: Interactively control a ' + \
           'Horizon device connected at <device>.'
    vers = 'Horizon Protocol Document version to communicate with. ' + \
           'Supported version(s): '
    ver = []
    p = optparse.OptionParser(usage=usage,description=desc)
    p.add_option('--output', '-o', action='store', type='str', metavar='FILE', 
                 default='/dev/null', help='Location to send subscription '\
                 'data. Default location (/dev/null) enables "get_waiting" '\
                 'command. To watch the results in real-time, use the "tail '\
                 '-F" command on this file.')
    p.add_option('--verbose', '-v', action='count', 
                 help='Print detailed messages. Use twice for debug messages.')
    p.add_option('--debug_messages', action='store', type='choice',
                 choices=['DEBUG','INFO','WARNING','ERROR','NONE'], 
                 help='Select the debug level for messages.py. '\
                 'INFO is useful for general debug and DEBUG is useful for' \
                 ' crash location detection. This flag automatically enables'\
                 ' verbose debug messages (-v -v) regardless of LEVEL.')
    p.add_option('--debug_payloads', action='store', type='choice',
                 choices=['DEBUG','INFO','WARNING','ERROR','NONE'], 
                 help='Select the debug level for payloads.py. '\
                 'INFO is useful for general debug and DEBUG is useful for' \
                 ' crash location detection. This flag automatically enables'\
                 ' verbose debug messages (-v -v) regardless of LEVEL.')
    p.add_option('--debug_protocol', action='store', type='choice',
                 choices=['DEBUG','INFO','WARNING','ERROR','NONE'], 
                 help='Select the debug level for protocol.py. '\
                 'INFO is useful for general debug and DEBUG is useful for' \
                 ' crash location detection. This flag automatically enables'\
                 ' verbose debug messages (-v -v) regardless of LEVEL.')
    p.add_option('--debug_transports', action='store', type='choice',
                 choices=['DEBUG','INFO','WARNING','ERROR','NONE'], 
                 help='Select the debug level for transports.py. '\
                 'INFO is useful for general debug and DEBUG is useful for' \
                 ' crash location detection. This flag automatically enables'\
                 ' verbose debug messages (-v -v) regardless of LEVEL.')
    p.add_option('--doc', action='store_true', 
                 help='Access the latest Horizon specification document ')
    encr = []
    try:
        Crypto.Random
        try:
            Crypto.Cipher.AES
            encr += ['AES']
        except NameError:
            pass
        try:
            Crypto.Cipher.Blowfish
            encr += ['Blowfish']
        except NameError:
            pass
        try:
            Crypto.Cipher.CAST
            encr += ['CAST']
        except NameError:
            pass
        try:
            Crypto.Cipher.DES
            encr += ['DES']
        except NameError:
            pass
        try:
            Crypto.Cipher.DES3
            encr += ['DES3']
        except NameError:
            pass
        try:
            Crypto.Cipher.ARC2
            encr += ['RC2']
        except NameError:
            pass
        p.add_option('--encryption', action='store', type='choice',
                 choices=encr, 
                 help='Select the encryption cipher to use for TCP '\
                 'communication ['+', '.join(encr)+']. '\
                 'Not all choices may be available; refer to '\
                 '`python -m clearpath.horizon.demo --doc` for details.', 
                 metavar='CIPHER')
        p.add_option('--key', action='store',type='str', metavar='KEY', 
                 help='The encryption key to use with --encryption. '\
                 'The key length is dependent upon the chosen cipher.')
    except NameError:
        pass
    p.add_option('--version', action='store', type='str', metavar='VER', 
                 default='0.0', help=vers)
    options, arguments = p.parse_args()
    logger.info("...command-line arguments parsed.")
    
    # Check for doc
    if options.doc:
        logger.info("Documentation flag found: Launching documentation...")
        loc = __file__.rstrip('demo.py')+'doc/'
        if os.system("man -l "+loc+"horizon.3") != 0:
            if os.system("less "+loc+"horizon.txt") != 0:
                os.system("cat "+loc+"horizon.txt")
        sys.exit(0)
    
    # Check Number of Command-Line Arguments
    if len(arguments) > 1:
        p.print_usage()
        sys.stderr.write("horizon_demo.py: error: too many arguments\n")
        sys.exit(1)
        
    # Address Regular Expression
    addr_re1 = re.compile(r'^((?:[a-zA-Z0-9]+' \
                           '(?:[a-zA-Z0-9\\-]+[a-zA-Z0-9])?)' \
                           '(?:\\.(?:[a-zA-Z0-9]+(?:[a-zA-Z0-9\\-]+[a-zA-Z0-9]'\
                           ')?))*):([1-9][0-9]{0,4})$')
    addr_re2 = re.compile(r'^((?:[a-zA-Z0-9\\-]{1,63})' \
                           '(?:\\.(?:[a-zA-Z0-9\\-]' \
                           '{1,63}))*):([1-9][0-9]{0,4})$')
    addr_re3 = re.compile(r'^([a-zA-Z0-9\\-\\.]{1,255}):' \
                           '((?:6553[0-5])|(?:655' \
                           '[0-2][0-9])|(?:65[0-4][0-9]{2})|(?:6[0-4][0-9]{3})'\
                           '|(?:[1-5][0-9]{4})|(?:[1-9][0-9]{0,3}))$')
    
    # Check verbosity
    if options.debug_messages != None or options.debug_payloads != None or \
            options.debug_protocol != None or options.debug_transports != None:
        logger_handler.setFormatter(logging.Formatter(
                                        "%(name)s: %(levelname)s: %(message)s"))
        horizon_logger.setLevel(logging.DEBUG)
        logger.info("Debug mode entered.")
        if options.debug_messages != None:
            messages.logger.propagate = True
            if options.debug_messages == 'DEBUG':
                messages.logger.setLevel(logging.DEBUG)
            elif options.debug_messages == 'INFO':
                messages.logger.setLevel(logging.INFO)
            elif options.debug_messages == 'WARNING':
                messages.logger.setLevel(logging.WARNING)
            elif options.debug_messages == 'ERROR':
                messages.logger.setLevel(logging.ERROR)
            else:
                messages.logger.propagate = False
        if options.debug_payloads != None:
            payloads.logger.propagate = True
            if options.debug_payloads == 'DEBUG':
                payloads.logger.setLevel(logging.DEBUG)
            elif options.debug_payloads == 'INFO':
                payloads.logger.setLevel(logging.INFO)
            elif options.debug_payloads == 'WARNING':
                payloads.logger.setLevel(logging.WARNING)
            elif options.debug_payloads == 'ERROR':
                payloads.logger.setLevel(logging.ERROR)
            else:
                payloads.logger.propagate = False
        if options.debug_protocol != None:
            protocol.logger.propagate = True
            if options.debug_protocol == 'DEBUG':
                protocol.logger.setLevel(logging.DEBUG)
            elif options.debug_protocol == 'INFO':
                protocol.logger.setLevel(logging.INFO)
            elif options.debug_protocol == 'WARNING':
                protocol.logger.setLevel(logging.WARNING)
            elif options.debug_protocol == 'ERROR':
                protocol.logger.setLevel(logging.ERROR)
            else:
                protocol.logger.propagate = False
        if options.debug_transports != None:
            transports.logger.propagate = True
            if options.debug_transports == 'DEBUG':
                transports.logger.setLevel(logging.DEBUG)
            elif options.debug_transports == 'INFO':
                transports.logger.setLevel(logging.INFO)
            elif options.debug_transports == 'WARNING':
                transports.logger.setLevel(logging.WARNING)
            elif options.debug_transports == 'ERROR':
                transports.logger.setLevel(logging.ERROR)
            else:
                transports.logger.propagate = False
    elif options.verbose != None and options.verbose > 1:
        horizon_logger.setLevel(logging.DEBUG)
        logger.info("Debug mode entered.")

    transport = None 
    transport_args = {}

    if len(arguments) == 0:
        # Autodetect
        transport = transports.Serial.autodetect

    else:
        # Otherwise, try to treat it as a serial port
        transport = transports.Serial
        transport_args = {'port':arguments[0]}
        logger.info("Using serial port %s." % arguments[0])
            
    # Invalid Device
    if transport == None:
        p.print_usage()
        sys.stderr.write("error: invalid device specified\n")
        sys.exit(1)
   
    # Check for output
    output = ''
    if options.output != '/dev/null' and (os.path.isdir(output) or \
            (os.path.isfile(output) and not os.access(output, os.R_OK|os.W_OK))\
            or (not os.path.lexists(output) and not os.access(
                os.path.dirname(os.path.abspath(output)), os.R_OK|os.W_OK))):
        p.print_usage()
        ex = "error: option --output: inaccessible value"
        sys.stderr.write(ex + '\n')
        sys.exit(1)
    else:
        if options.output != '/dev/null': output = options.output
        if output != '':
            logger.info("Saving subscription data to '%s'." % output)
        
    # Try to initialize Horizon
    horizon = None
    command = None
    err = 0
    try:
        
        # Init Horizon
        logger.debug("Initializing Horizon Interface...")
        horizon = Horizon(transport=transport,
                          transport_args=transport_args)
        logger.debug("...Horizon Interface Initialized.")
        
        # open the communications
        logger.debug("Opening Communications...")
        horizon.open()
        logger.debug("...Communications Open.")
        
        # Start command Loop
        command = HorizonDemo(port=str(horizon), 
                              horizon=horizon, 
                              output=output)
        logger.debug("Entering shell...")
        histfile = os.path.join(os.environ["HOME"], ".cprcli_history")
        try:
            readline.read_history_file(histfile)
        except IOError:
            # No history, which is cool.
            pass

        readline.set_history_length(100)
        command.cmdloop()
        readline.write_history_file(histfile)
        logger.debug("...shell exited.")

 #   except utils.TransportError as ex:
 #       logger.error("Unable to autodetect serial Horizon device.")
 #       err = 1
   
#    except IOError as ex:
#        logger.error("Connection Failed: %s", ex)
#        logger.debug("...shell exited.")
#        err = 1
    
    except KeyboardInterrupt:
        print ("") # Only catch to prevent program closing before clean-up
        logger.debug("...shell exited.")
    
    #except Exception as ex:
    #    logging.critical(ex)
    #    logger.debug("...shell exited.")
    #    err = 1
               
    # Cleanup
    finally:
        if horizon != None:
            logger.debug("Closing Communications...")
            horizon.close()
            logger.debug("...Communications Closed.")
        
    # Finish Program
    logger.info("Program Complete!")
    sys.exit(err)




################################################################################
# Script


# Check if run as a script
if __name__ == "__main__":
    
    # Run the main method
    __main()

    # Exit Bad - Should not reach so if it does: error
    sys.exit(1)
