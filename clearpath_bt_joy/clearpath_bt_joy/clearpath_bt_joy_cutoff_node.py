# Software License Agreement (BSD)
#
# @author    Chris Iverach-Brereton <civerachb@clearpathrobotics.com>
# @copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import os
import subprocess

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Bool, Int32


class RssiCutoffNode(Node):
    """
    Cuts off joy input if the controller RSSI is too low.

    Monitors the RSSI of a joy device and publishes 2 topics:
        - rssi (std_msgs/Int32) -- the raw RSSI reading for the device
        - rssi_ok  (std_msgs/Bool) -- is the RSSI strong enough to accept the joy inputs?
    """

    def __init__(self):
        super().__init__('rssi_cutoff_node')

        self.declare_parameter('rssi_cutoff', -80)
        self.rssi_cutoff = self.get_parameter('rssi_cutoff').value

        # Create our publishers
        self.rssi_ok_pub = self.create_publisher(Bool, 'rssi_ok', 10)
        self.rssi_pub = self.create_publisher(Int32, 'rssi', 10)

        # Get the 'dev' parameter from the joy_node to determine what device we're using
        cli = self.create_client(GetParameters, 'joy_node/get_parameters')
        cli.wait_for_service()
        req = GetParameters.Request()
        req.names = ['dev']
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.joy_device = future.result().values[0].string_value
        else:
            self.get_logger().warn('Unable to determine joy device')
            self.joy_device = None

        self.mac_addr = self.get_mac()

        if self.mac_addr is not None:
            self.rssi_timer = self.create_timer(0.1, self.check_rssi)
        else:
            self.get_logger().warn(f'Unable to determine MAC address for {self.joy_device}')

    def get_mac(self):
        if self.joy_device is None:
            return None

        # wait until the joy device appears on the local file system
        rate = self.create_rate(1)
        while not os.path.exists(self.joy_device):
            rate.sleep()

        udev_proc = subprocess.Popen(
            [
                'udevadm',
                'info',
                '--attribute-walk',
                self.joy_device,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        grep_proc = subprocess.Popen(
            [
                'grep',
                'ATTRS{uniq}=='
            ],
            stdin=udev_proc.stdout,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        result = grep_proc.communicate()
        if result[0] is not None:
            try:
                return result[0].decode().strip().split('==')[1].replace('"', '')
            except Exception as err:
                self.get_logger().warning(f'Failed to read MAC address: {err}')
                return None
        else:
            self.get_logger().warning('Failed to read MAC address: no output')
            return None

    def check_rssi(self):
        hcitool_proc = subprocess.Popen(
            [
                'hcitool',
                'rssi',
                self.mac_addr
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        try:
            result = hcitool_proc.communicate()
            stdout = result[0].decode().strip()
            stderr = result[1].decode().strip()

            rssi_ok = Bool()
            rssi_level = Int32()
            if 'not connected' in stderr.lower():
                rssi_ok.data = False
                rssi_level.data = -1_000_000  # arbitrarily huge to indicate no connection
            else:
                rssi_level.data = int(stdout.split(':')[-1].strip())
                rssi_ok.data = rssi_level.data >= self.rssi_cutoff

            self.rssi_ok_pub.publish(rssi_ok)
            self.rssi_pub.publish(rssi_level)
        except Exception as err:
            self.get_logger().warning(f'Failed to read RSSI: {err}')


def main():
    rclpy.init()
    node = RssiCutoffNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()