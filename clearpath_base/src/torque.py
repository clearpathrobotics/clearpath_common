#!/usr/bin/python

# ROS stuff
import roslib; roslib.load_manifest('clearpath_base')
import rospy

from base import Clearpath


class Torque(Clearpath):
    def __init__(self):
        rospy.loginfo("Clearpath Base")
        rospy.loginfo("Using closed-loop current control")
        # TODO: insert detection here for differential platform

        Clearpath.__init__(self)

        if self.horizon:
            self.max_current = rospy.get_param('~max_current', 26)
            self.linear_scale = rospy.get_param('~linear_scale', 1.2)
            self.angular_scale = rospy.get_param('~angular_scale', 2)
            rospy.loginfo("Using %f m/s as %f A output (linear scale)", 
                          self.linear_scale, self.max_current)
            rospy.loginfo("Using %f rad/s as +/- %f A (angular scale)", 
                          self.angular_scale, self.max_current)

    def cmd_vel(self, linear_velocity, angular_velocity):
        linear = linear_velocity * (self.max_current / self.linear_scale)
        diff = angular_velocity * (self.max_current / self.angular_scale)
        left_current = max(min(linear - diff, self.max_current), -self.max_current)
        right_current = max(min(linear + diff, self.max_current), -self.max_current)
        self.horizon.set_differential_current(left_current, right_current)


Torque().run()
