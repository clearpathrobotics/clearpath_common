#!/usr/bin/env python

import roslib; roslib.load_manifest('clearpath_bringup')

import rospy
from std_msgs.msg import String

import time, sys, os

DEFAULT_NS="/clearpath/robots/default/"

class Warn:
    def announce(self, data):
        self.found.add(data.data)

    def __init__(self):
        clearpath_robot_env = os.environ.get("CLEARPATH_ROBOT")

        rospy.init_node('warn', anonymous=True)

        self.found = set()
        model = rospy.get_param('~model', 'husky')
        rospy.Subscriber("/clearpath/announce/robots", String, self.announce)

        time.sleep(0.5)

        if clearpath_robot_env == None:
            rospy.logwarn("Env $CLEARPATH_ROBOT not set, assuming %s" % DEFAULT_NS)
            clearpath_robot_env = DEFAULT_NS

        if clearpath_robot_env in self.found:
            rospy.loginfo("Clearpath robot announced in %s" % clearpath_robot_env)
            rospy.spin()
            exit(0)
        
        if len(self.found) >= 1:
            rospy.logerr("One or more Clearpath robots announced, but none in %s" % clearpath_robot_env)
            rospy.logerr("In your .bashrc file, try adding: export CLEARPATH_ROBOT=%s" % self.found.pop())
            exit(1)    
        else:
            rospy.logerr("No Clearpath announce found.")
            rospy.logerr("Bring up robot and try again: roslaunch {0}_bringup {0}.launch".format(model))
            exit(1)
            # rospy.logerr("sudo service husky start")
            # rospy.logerr("sudo rosrun husky_bringup install")


if __name__=="__main__": Warn()
