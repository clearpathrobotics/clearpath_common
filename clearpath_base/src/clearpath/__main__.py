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
#  File: clearpath/__main__.py
#  Desc: Clearpath Robotics Installed Python Modules Listing.
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



## @package clearpath.__main__
#  Clearpath Robotics Modules Listing
# 
#  Python Script for listing installed Clearpath python modules.
#
#  @author     Malcolm Robert
#  @date       24/03/10
#  @version    1.0
#
#  @section USE
#
#  This module, the main script for the clearpath package, provides the user
#  with a list of installed Clearpath Robotics, Inc. modules, their versions, 
#  and whether the module can run as a script (x) or just a module (m).
#  Because this script imports modules as it searches, only modules compatible
#  with the version of Python you are running will be listed.
#
#  This script is executed by running
#  'python -m clearpath' for Python 2.7+ and Python 3.1+ or 
#  'python -m clearpath.__main__' for Python 2.6- and Python 3.0.
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
"""Python Script for listing installed Clearpath python modules.

   Copyright © 2010 Clearpath Robotics, Inc.
   All rights reserved
   
   Created: 24/03/10
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
import os.path                  # File Path Manipulation
import pkgutil                  # Python Package Utilities 
import sys                      # Python Interpreter Functionality


# Module Support
## Module Version
__version__  = "1.0"
"""Module Version"""
## SVN Code Revision
__revision__ = "$Revision: 634 $"
""" SVN Code Revision"""




################################################################################
# Module Listing



## List Installed Modules
#  
#  Lists installed modules within the given package.
#  List format is similar to the Unix program tree.
#
#  @param  package The name of the package to list
#  @param  prefix  The string to put at the start of each line
#  @param  pos     The location of the package: 0 - head, 1 - middle, 2 - tail
#  @return list of tuple(line,version,is_script) in the display order
#
#  @pydoc
def list_modules(package, prefix = '', pos = 0):
    """List Installed Modules."""
    mods = []
    
    # import the package for traversal
    try:
        __import__(package)
        pkg = sys.modules[package]
    
        # print package name
        pre = '';
        if pos >= 2:
            mods.append(tuple([prefix + "`-- " + pkg.__name__.split('.')[-1],
                               "%s r%s" % (pkg.__version__,
                                             pkg.__revision__[10:-1].strip()),
                                hasattr(pkg,'__main')]))
            pre = prefix + "    "
        elif pos == 1:
            mods.append(tuple([prefix + "|-- " + pkg.__name__.split('.')[-1],
                               "%s r%s" % (pkg.__version__,
                                             pkg.__revision__[10:-1].strip()),
                                hasattr(pkg,'__main')]))
            pre = prefix + "|   "
        else:
            mods.append(tuple([prefix + pkg.__name__.split('.')[-1],
                               "%s r%s" % (pkg.__version__,
                                             pkg.__revision__[10:-1].strip()),
                                hasattr(pkg,'__main')]))
            pre = prefix
            
        # grab sub-packages & modules and sort them by name
        def mod_key(mod):
            return mod[0]
        children = [tuple([name, ispkg]) for _, name, ispkg in \
                    pkgutil.iter_modules([os.path.dirname(pkg.__file__)])]
        children.sort(key=mod_key)
        
        # traverse children
        for i in range(0,len(children)):
            
            # module -> import & print
            if children[i][1] == False:
                try:
                    __import__(package + "." + children[i][0])
                    chi = sys.modules[package + "." + children[i][0]]
                    if i >= len(children) - 1:
                        mods.append(tuple([pre + "`-- " + children[i][0],
                                           "%s r%s" % (chi.__version__,
                                            chi.__revision__[10:-1].strip()),
                                            hasattr(chi,'__main')]))
                    else:
                        mods.append(tuple([pre + "|-- " + children[i][0],
                                           "%s r%s" % (chi.__version__,
                                            chi.__revision__[10:-1].strip()),
                                            hasattr(chi,'__main')]))
                except:
                    pass
                    
            # package -> recursion
            else:
                if i >= len(children) - 1:
                    mods += list_modules(package + "." + children[i][0], pre, 2)
                else:
                    mods += list_modules(package + "." + children[i][0], pre, 1)
        
    # package must be import-able to be displayed
    except:
        pass
    return mods
        
        
        
## List Installed Modules
#
#  Main Program for Clearpath Robotics, Inc. package
#  - Gets a tree list of installed clearpath modules and their versions
#  - Prints the list to stdout
#
#  @pydoc
def __main():
    """List Installed Modules."""
    
    # Get the tree list
    modules = list_modules('clearpath', '', 0)
    
    # Calculate padding requirements for pretty printing
    def mod_key(mod):
        return mod[0]
    width = max(list(map(len,list(map(mod_key,modules))))) + 1
    def mod_key1(mod):
        return mod[1]
    length = max(list(map(len,list(map(mod_key1,modules)))))
    
    # Print the tree list
    for mod in modules:
        if mod[2] == True:
            print((mod[0] + ' ' + '.'*(width-len(mod[0])) + ' [' + mod[1] + 
                   '.'*(length-len(mod[1])) + '] x'))
        else:
            print((mod[0] + ' ' + '.'*(width-len(mod[0])) + ' [' + mod[1] + 
                   ' '*(length-len(mod[1])) + '] m'))
    
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
