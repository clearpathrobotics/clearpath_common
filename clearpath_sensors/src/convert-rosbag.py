#!/usr/bin/python

import roslib; roslib.load_manifest('rosbag')
import rosbag

import sys

del sys.argv[0]
robot_namespace = '/clearpath/robots/default'

if len(sys.argv) < 1:
    print "Usage: ./convert-rosbag.py log1.bag log2.bag"
    print "       ./convert-rosbag.py *.bag"
    print ""
    print "Edit the script to adjust topics and fields to output as csv."

for filename in sys.argv: 
    print "Extracting from %s" % filename
    bag = rosbag.Bag(filename)
    with open(filename + '.mag.csv', 'w') as outfile:
        outfile.write("x,y,z\n")

        for topic, msg, t in bag.read_messages(topics=[robot_namespace + '/raw_mag']):
            fields = [msg.x,msg.y,msg.z]
            outfile.write(','.join([str(f) for f in fields]))
            outfile.write('\n')

    bag.close()
