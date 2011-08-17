#!/usr/bin/python

import math, time, serial

import roslib; roslib.load_manifest('clearpath_sensors')
import rospy

from clearpath_sensors.msg import GPSFix


METERS_PER_SEC_PER_KNOT = 0.514444444
FIRST_TIMEOUT_SECS = 3.0
TIMEOUT_SECS = 1.2
RETRY_OPEN_SECS = 1.0

STATE_UNKNOWN = 0
STATE_NOFIX = 1
STATE_FIX = 2


class GPS:
    def __init__(self):
        rospy.init_node('nmea_gps')
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baud = rospy.get_param('~baud', 4800)
        self.serial = None

        self.start_serial()
        self.state = STATE_UNKNOWN;
        self.pub = rospy.Publisher('fix', GPSFix)
        

    def start_serial(self):
        if self.serial:
            self.serial.close()
            self.serial = None

        error = None
        while True:
            try:
                self.serial = serial.Serial(self.port, self.baud, timeout=0)
                self.timeout = time.clock() + FIRST_TIMEOUT_SECS
                rospy.loginfo("Opened GPS on port: %s" % self.port)
                break;

            except serial.SerialException as e:
                if not error:
                    error = e
                    rospy.logerr("Error opening GPS serial port: %s" % error)
                    rospy.logerr("Will retry opening GPS every second.")
                time.sleep(RETRY_OPEN_SECS)


    def run(self):
        buffer = "";
        while not rospy.is_shutdown():
            time.sleep(0.01)
            raw = self.serial.read(255)
            if len(raw) > 0:
                self.timeout = time.clock() + TIMEOUT_SECS
                chunks = raw.split('\n')
                buffer += chunks.pop(0)
                while len(chunks) > 0:
                    self.process(buffer)
                    buffer = chunks.pop(0)
            else:
                if time.clock() > self.timeout:
                    rospy.logerr("GPS timed out. Attempting to reopen serial port.")
                    self.start_serial()
                    

    def process(self, buffer):
        fields = buffer.split(',')
        if fields[0] == '$GPRMC':
            if fields[2] == 'A':
                if self.state != STATE_FIX:
                    rospy.loginfo("GPS fix acquired. Publishing to %sfix." % rospy.get_namespace())
                    self.state = STATE_FIX
                fix_msg = GPSFix()
                fix_msg.latitude = self._lat(fields)
                fix_msg.longitude = self._lon(fields)
                if fields[8]: fix_msg.track = float(fields[8])
                if fields[7]: fix_msg.speed = float(fields[7]) * METERS_PER_SEC_PER_KNOT
                self.pub.publish(fix_msg)
            else:
                if self.state != STATE_NOFIX:
                    rospy.loginfo("GPS has no fix.")
                    self.state = STATE_NOFIX


    @staticmethod
    def _lat(fields):
        inp = float(fields[3])
        out = math.floor(inp / 100) + (inp % 100) / 60
        if fields[4] == 'S':
            out =- out
        return out

        
    @staticmethod
    def _lon(fields):
        inp = float(fields[5])
        out = math.floor(inp / 100) + (inp % 100) / 60
        if fields[6] == 'W':
            out =- out
        return out


GPS().run()
