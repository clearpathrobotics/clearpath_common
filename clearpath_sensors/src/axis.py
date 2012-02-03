#!/usr/bin/env python
#
# Axis camera image driver. Based on:
# https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/sandbox/axis_camera/axis.py
#

import os, sys, string, time, getopt
import urllib

import roslib; roslib.load_manifest('clearpath_sensors') 
import rospy 

from sensor_msgs.msg import CompressedImage, CameraInfo

import threading


class StreamThread(threading.Thread):
  def __init__(self, axis):
    threading.Thread.__init__(self)

    self.axis = axis
    self.daemon = True

    self.axis_frame_id = "axis_camera_optical_frame"

  def run(self):
    while True:
      try:
        self.stream()
      except:
        import traceback
        traceback.print_exc()
      time.sleep(1)

  def stream(self):
    url = 'http://%s/mjpg/video.mjpg' % self.axis.hostname
    url = url + "?resolultion=%dx%d" % (self.axis.width, self.axis.height)
    fp = urllib.urlopen(url)

    while True:
      boundary = fp.readline()

      header = {}
      while 1:
        line = fp.readline()
        if line == "\r\n": break
        line = line.strip()

        parts = line.split(": ", 1)
        header[parts[0]] = parts[1]

      content_length = int(header['Content-Length'])

      img = fp.read(content_length)
      line = fp.readline()
      
      msg = CompressedImage()
      msg.header.stamp = rospy.Time.now()
      msg.header.frame_id = self.axis_frame_id
      msg.format = "jpeg"
      msg.data = img

      self.axis.pub.publish(msg)

      """
      cimsg = CameraInfo()
      cimsg.header.stamp = msg.header.stamp
      cimsg.header.frame_id = self.axis_frame_id
      cimsg.width = self.axis.width
      cimsg.height = self.axis.height

      # Adding the best calibration we have for these cameras
      cimsg.D = [-0.26129794156876202, 0.053510647147691104, -0.004329961180682111, 0.0002979023290858089, 0]
      cimsg.K = [259.79888071407669, 0.0, 332.0316187674498, 0.0, 258.00868558667878, 252.46066959143357, 0.0, 0.0, 1.0]
      cimsg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
      cimsg.P = [259.79888071407669, 0.0, 332.0316187674498, 0.0, 0.0, 258.00868558667878, 252.46066959143357, 0.0, 0.0, 0.0, 1.0, 0.0]

      self.axis.caminfo_pub.publish(cimsg)
      """

class Axis:
  def __init__(self, hostname, username, password, width, height):
    self.hostname = hostname
    self.username = username
    self.password = password
    self.width = width
    self.height = height

    self.st = None
    self.pub = rospy.Publisher("compressed", CompressedImage, self)
    self.caminfo_pub = rospy.Publisher("camera_info", CameraInfo, self)

  def peer_subscribe(self, topic_name, topic_publish, peer_publish):
    # Lazy-start the image-publisher.
    if self.st is None:
      self.st = StreamThread(self)
      self.st.start()


def main():
  rospy.init_node("axis")

  arg_defaults = {
      'hostname': '192.168.0.90',
      'username': '',
      'password': '',
      'width': 640,
      'height': 480
      }
  args = {}
  for name in arg_defaults:
    args[name] = rospy.get_param(rospy.search_param(name), arg_defaults[name])

  Axis(**args)
  rospy.spin()


if __name__ == "__main__":
  main()
