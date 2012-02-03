#!/usr/bin/env python
#
# Basic PTZ node, based on documentation here:
#   http://www.axis.com/files/manuals/vapix_ptz_45621_en_1112.pdf
#

import os, sys, string, time, getopt, threading
import httplib, urllib

import roslib; roslib.load_manifest('clearpath_sensors') 
import rospy 

from clearpath_sensors.msg import Axis


class StateThread(threading.Thread):
  def __init__(self, axis):
    threading.Thread.__init__(self)

    self.axis = axis
    self.daemon = True

  def run(self):
    r = rospy.Rate(10)
    msg = Axis()

    while True:
      conn = httplib.HTTPConnection(self.axis.hostname)
      params = { 'query':'position' }
      conn.request("GET", "/axis-cgi/com/ptz.cgi?%s" % urllib.urlencode(params))
      response = conn.getresponse()
      if response.status == 200:
        body = response.read()
        params = dict([s.split('=',2) for s in body.splitlines()])
        msg.pan = float(params['pan'])
        msg.tilt = float(params['tilt'])
        msg.zoom = float(params['zoom'])
        msg.iris = float(params['iris'])
        msg.focus = 0.0
        if 'focus' in params:
          msg.focus = float(params['focus'])
        msg.autofocus = (params['autofocus'] == 'on')
        self.axis.pub.publish(msg)
      r.sleep()
      

class AxisPTZ:
  def __init__(self, hostname, username, password):
    self.hostname = hostname
    self.username = username
    self.password = password

    self.st = None
    self.pub = rospy.Publisher("state", Axis, self)
    self.sub = rospy.Subscriber("cmd", Axis, self.cmd)

  def peer_subscribe(self, topic_name, topic_publish, peer_publish):
    # Lazy-start the state publisher.
    if self.st is None:
      self.st = StateThread(self)
      self.st.start()

  def cmd(self, msg):
    conn = httplib.HTTPConnection(self.hostname)
    params = { 'pan': msg.pan, 'tilt': msg.tilt, 'zoom': msg.zoom, 'brightness': msg.brightness }
    if msg.autofocus:
      params['autofocus'] = 'on'
    else:
      params['autofocus'] = 'off'
      params['focus'] = msg.focus
    if msg.iris > 0.000001:
      rospy.logwarn("Iris value is read-only.")
    conn.request("GET", "/axis-cgi/com/ptz.cgi?%s" % urllib.urlencode(params))


def main():
  rospy.init_node("axis")

  arg_defaults = {
      'hostname': '192.168.0.90',
      'username': '',
      'password': '',
      }
  args = {}
  for name in arg_defaults:
    args[name] = rospy.get_param(rospy.search_param(name), arg_defaults[name])

  AxisPTZ(**args)
  rospy.spin()


if __name__ == "__main__":
  main()
