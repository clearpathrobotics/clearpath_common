#!/usr/bin/python

# ROS stuff
import roslib; roslib.load_manifest('clearpath_base')
import rospy

from base import Clearpath


class Kinematic(Clearpath):
    def __init__(self):
        Clearpath.__init__(self)
        rospy.loginfo("Using closed-loop kinematic control")
        
        self.accel = rospy.get_param('~accel', 2)
        rospy.loginfo("Using %d m/s^2 acceleration", self.accel)

    def cmd_vel(self, linear_velocity, angular_velocity):
        self.horizon.set_velocity(linear_velocity, angular_velocity, self.accel)


Kinematic().run()
