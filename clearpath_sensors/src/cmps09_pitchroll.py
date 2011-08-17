#!/usr/bin/env python

# ROS stuff
import roslib; roslib.load_manifest('clearpath_sensors')
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Standard
import serial

class CMPS09(object):
    def __init__(self):
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
        self.pub = rospy.Publisher('cmd_vel', Twist)

        announce_pub = rospy.Publisher('/clearpath/announce/teleops',
                                       String, latch=True)
        announce_pub.publish(rospy.get_namespace());
    
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
                self.transport.write('\x23')
                raw = self.transport.read(4)
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
            
            vel_raw = ord(raw[2])
            if vel_raw > 127:
                vel_raw -= 256
            rot_raw = ord(raw[3])
            if rot_raw > 127:
                rot_raw -= 256
            cmd = Twist()
            cmd.linear.x = vel_raw/-45.0
            cmd.angular.z = rot_raw/-45.0
            self.pub.publish(cmd)

            rate.sleep()


#Init and run
CMPS09().run()
