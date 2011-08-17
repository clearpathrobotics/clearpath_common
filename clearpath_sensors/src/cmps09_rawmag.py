#!/usr/bin/env python

# ROS stuff
import roslib; roslib.load_manifest('clearpath_sensors')
import rospy
from geometry_msgs.msg import Point

# Standard
import serial

def convert_to_signed(val):
    if val > 2**15:
        return val - 2**16
    else:
        return val

class CMPS09(object):

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        rospy.init_node('cmps09')
       
        # Opens up serial port 
        self.port = rospy.get_param('~port', '')
        self.hz = rospy.get_param('~hz', 20)
        self.timeout = rospy.get_param('~timeout', 0.2)

        if self.port != '':
            rospy.loginfo("CMPS09 using port %s.",self.port)
        else:
            rospy.logerr("No port specified for CMPS09!")
            exit(1)

        try:
            self.transport = serial.Serial(port=self.port, baudrate=9600, timeout=self.timeout)
            self.transport.open()
        except serial.serialutil.SerialException as e:
            rospy.logerr("CMPS09: %s" % e)
            exit(1)
        
        # Registers shutdown handler to close the serial port we just opened.
        rospy.on_shutdown(self.shutdown_handler)
        
        # Registers as publisher
        self.pub = rospy.Publisher('raw_mag', Point)
    
    # Shutdown     
    def shutdown_handler(self):
        self.transport.close()
        rospy.loginfo("CMPS09 shutdown.")
    
    # Main loop
    def run(self):
        rate = rospy.Rate(self.hz)
        consecutive_errors = 0

        while not rospy.is_shutdown():
            try:
                self.transport.write('\x22')
                raw = self.transport.read(6)
            except OSError as e:
                rospy.logerr("CMPS09: %s" % e)
                exit(1)

            if len(raw) < 2:
                # Error state
                consecutive_errors += 1
                if consecutive_errors == 3:
                    rospy.logerr("CMPS09 receiving no data. Will try again every second.")
                elif consecutive_errors > 3:
                    rospy.sleep(1.0)
                continue
                    
            # Success state
            if consecutive_errors >= 3:
                # If we'd previously logged the error message, log that 
                # things are okay again.
                rospy.loginfo("CMPS09 receiving data successfully.")
            consecutive_errors = 0
          
            alpha = 0.0625 
            self.x = (1-alpha)*self.x + alpha*convert_to_signed(ord(raw[0]) << 8 | ord(raw[1]))
            self.y = (1-alpha)*self.y + alpha*convert_to_signed(ord(raw[2]) << 8 | ord(raw[3]))
            self.z = (1-alpha)*self.z + alpha*convert_to_signed(ord(raw[4]) << 8 | ord(raw[5]))
            self.pub.publish(Point(self.x,self.y,self.z))

            rate.sleep()


#Init and run
CMPS09().run()
