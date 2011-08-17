#!/usr/bin/python


# ROS stuff
import roslib; roslib.load_manifest('clearpath_base')
import rospy

from clearpath_base.msg import ClearpathRobot
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32

# Required Clearpath Modules
from clearpath.horizon import Horizon 
from clearpath.horizon import transports 
from clearpath import utils  
import data

# Standard 
import sys
import logging


class Clearpath:
    def __init__(self):
        rospy.init_node('clearpath_base')

        # Instantiate Clearpath
        self.port = rospy.get_param('~port', '')

        if self.port != '':
            # Serial port 
            rospy.loginfo("Using port %s", self.port)
            self.horizon = Horizon(transport = transports.Serial, 
                                   transport_args = { 'port': self.port })
        else:
            # Not specified. Autodetect.
            rospy.loginfo("Using port autodetection")
            self.horizon = Horizon(transport = transports.Serial.autodetect)

        first_error = True
        while True:
            if rospy.is_shutdown():
                # This case is for when someone kills the node
                # and it never connected.
                self.horizon = None
                return

            try:
                self.horizon.open()
                rospy.loginfo("Connection successful on %s", self.horizon)
                break

            except utils.TransportError:
                if first_error:
                    rospy.logerr("Unable to connect on %s. Will retry every second.", self.port if self.port != '' else '/dev/ttyUSB* or /dev/ttyS*')
                    first_error = False
                rospy.sleep(1.0)


        rospy.on_shutdown(self.shutdown_handler)
        announce_pub = rospy.Publisher('/clearpath/announce/robots', String, latch=True)
        announce_pub.publish(rospy.get_namespace());

        # Fetch robot information so we can publish it (useful for logs, etc.)
        #self.horizon.reset()
        #rospy.sleep(0.5)
        platform_name = self.horizon.request_platform_name(subscription=0)
        platform_info = self.horizon.request_platform_info(subscription=0)
        firmware_info = self.horizon.request_firmware_info(subscription=0)
        robot_msg = data.pkg_robot_info(platform_name, platform_info, firmware_info)
        robot_pub = rospy.Publisher('robot', ClearpathRobot, latch=True)
        robot_pub.publish(robot_msg);

        self.tx = True
        self.hz = 10
        self.do_subscriptions()
        self.comm_error = False

        self.last_cmd = rospy.Time.now()
        self.freq_pub = rospy.Publisher('cmd_freq', Float32, latch=True)
        self.horizon.acks(False);

    # ROS calls this when receiving velocity commands. We impose a rate limit
    # on passing them down to the platform.
    def cmd_vel_handler(self, data):
        if self.tx:
            try:
                self.cmd_vel(data.linear.x, data.angular.z)
                self.freq_pub.publish((rospy.Time.now() - self.last_cmd).to_sec())
                self.last_cmd = rospy.Time.now()
                self.tx = False
                self.comm_error = False
            except IOError as ex:
                if not self.comm_error:
                    rospy.logerr('Problem communicating with platform: %s', ex)
                    self.comm_error = True
            except ValueError as ex:
                rospy.logerr('Platform said Bad Values: %f %f', data.linear.x, data.angular.z)

    # ROS calls this when spinning down.
    def shutdown_handler(self):
        self.horizon.close()
        rospy.loginfo("clearpath_base shutdown")


    def cmd_vel(self, linear_velocity, angular_velocity):
        rospy.logerror('Base class contains no command model. Use kinematic.py or raw.py.')
        raise NotImplementedError()


    def run(self):
        if self.horizon == None: return

        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_handler)
        rate = rospy.Rate(self.hz)

        while not rospy.is_shutdown():
            rate.sleep()
            self.tx = True
        
        self.horizon.close()


    def do_subscriptions(self):
        if hasattr(self, 'publishers'): return

        self.publishers = {}
        for topic, frequency in rospy.get_param('~data', {}).items():
            self.publishers[topic] = rospy.Publisher('data/' + topic, 
                                                     data.msgs[topic],
                                                     latch=True)
            subscribe_func = getattr(self.horizon, 'request_' + topic)
            subscribe_func(frequency)
            rospy.loginfo("Successfully returning data: request_%s", topic)
        self.horizon.add_handler(self._receive)


    def _receive(self, name, payload, timestamp):
        try:
            pkg_func = getattr(data, 'pkg_' + name)
        except AttributeError:
            rospy.loginfo("Unhandled Clearpath message of type: %s", name)
        else:
            msg = pkg_func(payload)
            msg.header.stamp = rospy.Time.now()
            if name in self.publishers:
                self.publishers[name].publish(msg)
            else:
                rospy.loginfo("Unsubscribed Clearpath message of type: %s", name)
