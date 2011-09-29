#!/usr/bin/python

import roslib; roslib.load_manifest('clearpath_teleop')
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class Teleop:
    def __init__(self):
        rospy.init_node('clearpath_teleop')

        self.turn_scale = rospy.get_param('~turn_scale')
        self.drive_scale = rospy.get_param('~drive_scale')
        self.deadman_button = rospy.get_param('~deadman_button', 0)

        self.cmd = None
        cmd_pub = rospy.Publisher('cmd_vel', Twist)

        announce_pub = rospy.Publisher('/clearpath/announce/teleops',
                                       String, latch=True)
        announce_pub.publish(rospy.get_namespace());

        rospy.Subscriber("joy", Joy, self.callback)
        rate = rospy.Rate(rospy.get_param('~hz', 20))
        
        while not rospy.is_shutdown():
            rate.sleep()
            if self.cmd:
                cmd_pub.publish(self.cmd)

    def callback(self, data):
        """ Receive joystick data, formulate Twist message. """
        cmd = Twist()
        cmd.linear.x = data.axes[1] * self.drive_scale
        cmd.angular.z = data.axes[0] * self.turn_scale

        if data.buttons[self.deadman_button] == 1:
            self.cmd = cmd
        else:
            self.cmd = None


if __name__ == "__main__": Teleop()
