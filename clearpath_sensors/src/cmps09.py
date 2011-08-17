#!/usr/bin/env python

# ROS stuff
import roslib; roslib.load_manifest('clearpath_sensors')
import rospy
from geometry_msgs.msg import Pose2D

# Standard
import serial, time

FIRST_TIMEOUT_SECS = 3.0
TIMEOUT_SECS = 0.3
RETRY_OPEN_SECS = 1.0

class CMPS09(object):
    def __init__(self):
        rospy.init_node('cmps09')
       
        # Opens up serial port 
        self.port = rospy.get_param('~port', '/dev/usb-compass')
        self.hz = rospy.get_param('~hz', 10)
        self.offset = rospy.get_param('~offset', 0.0)
	self.serial = None

        if self.port != '':
            rospy.loginfo("Compass using port %s.",self.port)
        else:
            rospy.logerr("No port specified for compass!")
            exit(1)

        self.start_serial()
        self.pub = rospy.Publisher('compass', Pose2D)

    def start_serial(self):
        if self.serial:
            self.serial.close()
            self.serial = None

        error = None
        while True:
            try:
                self.serial = serial.Serial(self.port, 9600, timeout=0.2)
                self.timeout = time.clock() + FIRST_TIMEOUT_SECS
                rospy.loginfo("Opened compass on port: %s" % self.port)
                break;
            except serial.SerialException as e:
                if not error:
                    error = e
                    rospy.logerr("Error opening compass on %s: %s" % (self.port, error))
                    rospy.logerr("Will retry opening compass every second.")
                time.sleep(RETRY_OPEN_SECS)

	    if rospy.is_shutdown():
	        exit(1)


    # Main loop
    def run(self):
        rate = rospy.Rate(self.hz)
        consecutive_errors = 0

        while not rospy.is_shutdown():
            try:
                self.serial.write('\x13')
                raw = self.serial.read(2)
            except OSError as e:
                rospy.logerr("Compass serial error: %s" % e) 
                rospy.loginfo("Reopening compass serial port.")
                self.start_serial()
                continue

            if len(raw) < 2:
                # Error state
                consecutive_errors += 1
                if consecutive_errors >= 3:
                    rospy.logerr("CMPS09 received no data for 3 consecutive requests.")
                    time.sleep(RETRY_OPEN_SECS)
                    rospy.loginfo("Reopening compass serial port.")
                    self.start_serial()
                continue
                    
            # Success state
            consecutive_errors = 0
            
            data = (ord(raw[0]) << 8) | ord(raw[1])
            data2 = (data / 10.0) + self.offset
            if data2 < 0.0: data2 += 360.0
            elif data2 > 360.0: data2 -= 360.0

            self.pub.publish(Pose2D(theta=(data2)))

            rate.sleep()


#Init and run
CMPS09().run()
