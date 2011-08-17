#!/usr/bin/env python

# Standard
import serial
import time

class CMPS09(object):
    def __init__(self):
        # Opens up serial port 
        self.port = '/dev/ttyUSB0'
        print "CMPS09 using port %s." % self.port

        try:
            self.transport = serial.Serial(port=self.port, baudrate=9600, timeout=0.2)
            self.transport.open()
        except serial.serialutil.SerialException as e:
            print "CMPS09: %s" % e
            exit(1)
        
    # Main loop
    def run(self):
        print "Beginning calibration"
        self.transport.write('\x31')
        time.sleep(0.1)
        self.transport.write('\x45')
        time.sleep(0.1)
        self.transport.write('\x5A')

        print "Confirm vehicle is pointing north and press ENTER"
        v = raw_input()
        self.transport.write('\x5E')
        print "Turn to 90 degrees and press ENTER"
        v = raw_input()
        self.transport.write('\x5E')
        print "Turn to 180 degrees and press ENTER"
        v = raw_input()
        self.transport.write('\x5E')
        print "Turn to 270 degrees and press ENTER"
        v = raw_input()
        self.transport.write('\x5E')
        print "Calibration complete"

    def destroy(self):
        self.transport.close()
        print "CMPS09 shut down"


#Init and run
try:
    CMPS09().run()
except OSError as e:
    print "Failure in calibration (%s) Retry suggested" % e

CMPS09().destroy()
