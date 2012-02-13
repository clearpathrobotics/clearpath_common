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
from threading import Event


CMD_TIME_MAX = 0.12
CMD_TIME_MIN = 0.02
RECONNECT_TIMEOUT = 1.0
DATA_TIMEOUT = 1.1


class Clearpath:
    def __init__(self):
        rospy.init_node('clearpath_base')
        self.port = rospy.get_param('~port', '')
        self.cmd_fill = rospy.get_param('~cmd_fill', True)

        if self.port != '':
            # Serial port 
            rospy.loginfo("Using port %s", self.port)
            self.horizon = Horizon(transport = transports.Serial, 
                                   transport_args = { 'port': self.port })
        else:
            # Not specified. Autodetect.
            rospy.loginfo("Using port autodetection")
            self.horizon = Horizon(transport = transports.Serial.autodetect)

        announce_pub = rospy.Publisher('/clearpath/announce/robots', String, latch=True)
        announce_pub.publish(rospy.get_namespace());

        self.freq_pub = rospy.Publisher('cmd_freq', Float32, latch=True)
        self.cmd_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_handler)

        self.cmd_msg = Twist()
        self.cmd_time = rospy.Time.now()
        self.cmd_event = Event()


    def run(self):
        previous_error = False
        cmd_timeout = False

        # Reconnection loop.
        while not rospy.is_shutdown():
            if previous_error:
                rospy.sleep(RECONNECT_TIMEOUT)
                
            try:
                if self.horizon.opened:
                    self.horizon.close()
                self.horizon.open()
                self.unsub_all()
                rospy.loginfo("Connection successful on %s", self.horizon)
                previous_error = False
                 
                # Fetch and publish robot information.
                self.horizon.acks(True);
                platform_name = self.horizon.request_platform_name(subscription=0)
                platform_info = self.horizon.request_platform_info(subscription=0)
                firmware_info = self.horizon.request_firmware_info(subscription=0)
                robot_msg = data.pkg_robot_info(platform_name, platform_info, firmware_info)
                robot_pub = rospy.Publisher('robot', ClearpathRobot, latch=True)
                robot_pub.publish(robot_msg);

                self.do_subscriptions()
                self.horizon.acks(False);

                while not rospy.is_shutdown():
                    self.cmd_event.wait(CMD_TIME_MAX)

                    if (rospy.Time.now() - self.cmd_time).to_sec() > CMD_TIME_MAX:
                        # Timed out waiting on command message. Send zeros instead to
                        # keep connection alive.
                        self.cmd_msg = Twist()
                        if not cmd_timeout:
                            rospy.loginfo("User commands timed out.")
                            cmd_timeout = True
                    else:
                        if cmd_timeout:
                            rospy.loginfo("Receiving user commands.")
                            cmd_timeout = False

                    self.cmd_event.clear()
                    
                    try:
                        if not cmd_timeout or self.cmd_fill:
                            self.cmd_vel(self.cmd_msg.linear.x, self.cmd_msg.angular.z)
                        rospy.sleep(CMD_TIME_MIN)
                    except: 
                        rospy.logerr("Problem issuing command to platform. Attempting reconnection.")
                        break

                    if (rospy.Time.now() - self.data_time).to_sec() > DATA_TIMEOUT:
                        rospy.logerr("Problem receiving data from platform. Attempting reconnection.")
                        break


            except utils.TransportError:
                if not previous_error:
                    rospy.logerr("Connection error on %s. Will retry every second.", self.port if self.port != '' else '/dev/ttyUSB* or /dev/ttyS*')
                    previous_error = True

        self.horizon.close()


    def cmd_vel_handler(self, msg):
        """ Received command from ROS. Signal the main thread to send it to robot. """
        last_cmd_time = self.cmd_time

        self.cmd_msg = msg
        self.cmd_time = rospy.Time.now()
        self.cmd_event.set()

        self.freq_pub.publish((last_cmd_time - self.cmd_time).to_sec())


    def cmd_vel(self, linear_velocity, angular_velocity):
        rospy.logerror('Base class contains no command model. Use kinematic.py or raw.py.')
        raise NotImplementedError()


    def unsub_all(self):
        for topic, cls in data.msgs.items():
            subscribe_func = getattr(self.horizon, 'request_' + topic)
            try:
                subscribe_func(0xffff)
            except:
                pass


    def do_subscriptions(self):
        self.publishers = {}
        for topic, frequency in rospy.get_param('~data', {}).items():
            if not topic in self.publishers:
                self.publishers[topic] = rospy.Publisher('data/' + topic, 
                                                         data.msgs[topic],
                                                         latch=True)
            subscribe_func = getattr(self.horizon, 'request_' + topic)
            subscribe_func(frequency)
            rospy.loginfo("Successfully returning data: request_%s", topic)

        self.horizon.add_handler(self._receive)
        self.data_time = rospy.Time.now()


    def _receive(self, name, payload, timestamp):
        self.data_time = rospy.Time.now()

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
