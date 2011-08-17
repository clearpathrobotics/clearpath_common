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
#  File: horizon/__main__.py
#  Desc: Horizon Python Module Script
#  Auth: Malcolm Robert
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
# Module



## @package clearpath.horizon.__main__
#  Horizon Python Module Script
# 
#  Python Script for the Horizon Python Module.
#
#  @author     Malcolm Robert
#  @date       07/04/10
#  @version    1.0
#
#  @section USE
#
#  This module, the main script for the clearpath.horizon package, provides 
#  absolutely no functionality; it just outputs what scripts within the
#  clearpath.horizon package can be run.
#
#  This script is executed by running
#  'python -m clearpath.horizon' for Python 2.7+ and Python 3.1+ or 
#  'python -m clearpath.horizon.__main__' for Python 2.6- and Python 3.0.
#
#  @section HISTORY
#
#  Version 1.0
#  - Initial Creation
#  - Python 2.5+ & 3.x compatible
#
#  @section License
#  @copydoc public_license
#
"""Python Script for the Horizon Python Module.

   Copyright © 2010 Clearpath Robotics, Inc.
   All rights reserved
   
   Created: 07/04/10
   Author:  Malcolm Robert
   Version: 1.0
   """


# A note on comments:
# To take advantage of the superior Doxygen documentation generator,
# Doxygen comments (first line ##, following lines #) are used.
# It is still important to provide Python __doc__ strings for use with help() 
# and pydoc, however, they (except for module) are also grabbed by Doxygen.
# To fix this, the custom Doxygen tag @pydoc has been added to separate
# __doc__ strings from being included in the last Doxygen tag.
# @pydoc must be the last Doxygen tag before """doc string"""
# Additionally, the custom Doxygen tag @req has been added to specify
# required modules that do not come with the standard Python distribution.
# Both custom tags are aliases to @par.


# Required Python Modules
import sys                      # Python Interpreter Functionality


# Module Support
## Module Version
__version__  = "1.0"
"""Module Version"""
## SVN Code Revision
__revision__ = "$Revision: 235 $"
""" SVN Code Revision"""




################################################################################
# Script Listing



## List Available Scripts
#
#  Main Program for Clearpath Robotics, Inc. Horizon package
#  - Prints the list of scripts to stdout
#
#  @pydoc
def __main():
    """List Available Scripts."""
    
    # Print the tree list
    print ("The package clearpath.horizon is a module and NOT a script.\n"
           "For a command-line interface demo of Horizon, run:"\
           "\n  python -m clearpath.horizon.demo\n"\
           "For Horizon message forwarding, run:\n"\
           "  python -m clearpath.horizon.forward")
    
    # Exit the program
    sys.exit(0);
    

  
  
################################################################################
# Script



# Check if run as a script
if __name__ == "__main__":

    # Run the main method
    __main()

    # Exit Bad - Should not reach so if it does: error
    sys.exit(1)
