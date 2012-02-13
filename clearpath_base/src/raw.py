#!/usr/bin/python

# ROS stuff
import roslib; roslib.load_manifest('clearpath_base')
import rospy

from base import Clearpath


class Raw(Clearpath):
    def __init__(self):
        Clearpath.__init__(self)
        rospy.loginfo("Using open-loop direct control")

        self.linear_scale = rospy.get_param('~linear_scale', 1.0)
        self.angular_scale = rospy.get_param('~angular_scale', 2)
        rospy.loginfo("Using %f m/s as 100%% output (linear scale)", self.linear_scale)
        rospy.loginfo("Using %f rad/s as +/- 100%% (angular scale)", self.angular_scale)

    def cmd_vel(self, linear_velocity, angular_velocity):
        linear = linear_velocity * (100.0 / self.linear_scale)
        diff = angular_velocity * (100.0 / self.angular_scale)
        left_percent = max(min(linear - diff, 100.0), -100.0)
        right_percent = max(min(linear + diff, 100.0), -100.0)
        self.horizon.set_differential_output(left_percent, right_percent)


Raw().run()
