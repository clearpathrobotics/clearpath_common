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
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Bool, Int32


class QualityCutoffNode(Node):
    """
    Cuts off joy input if the controller link quality is too low.

    Monitors the quality of a joy device and publishes 2 topics:
        - quality (std_msgs/Int32) -- the raw link quality for the connection
        - bt_quality_stop  (std_msgs/Bool) -- has the quality dropped too low to be consiered safe?
    """

    def __init__(self):
        super().__init__('bt_cutoff_node')

        # link quality is a 0-255 value, as reported by hcitool lq
        # by default use a quality of 20 or less to indicate a poor connection
        self.declare_parameter('quality_cutoff', 20)
        self.quality_cutoff = self.get_parameter('quality_cutoff').value

        # Create our publishers
        self.stop_pub = self.create_publisher(Bool, 'bt_quality_stop', qos_profile_default)
        self.quality_pub = self.create_publisher(Int32, 'quality', qos_profile_default)

        # Get the 'dev' parameter from the joy_node to determine what device we're using
        self.get_logger().info('Waiting for joy_node parameter service...')
        cli = self.create_client(GetParameters, 'joy_node/get_parameters')
        cli.wait_for_service()
        req = GetParameters.Request()
        req.names = ['dev']
        self.get_logger().info('Getting joy_device parameter...')
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.joy_device = future.result().values[0].string_value
        else:
            self.get_logger().warning('Unable to determine joy device')
            self.joy_device = None

        self.mac_addr = self.get_mac()

        # run the timer at 5Hz
        # originally this was 10Hz, but that resulted in missed deadlines
        self.get_logger().info('Starting quality-publish timer')
        if self.mac_addr is not None:
            self.quality_timer = self.create_timer(0.2, self.check_quality)
        else:
            self.get_logger().info(f'Assuming {self.joy_device} is wired; quality check will be bypassed')  # noqa: E501
            self.quality_timer = self.create_timer(0.2, self.fake_quality)

    def get_mac(self):
        if self.joy_device is None:
            return None

        # wait until the joy device appears on the local file system
        self.get_logger().info(f'Waiting for {self.joy_device} to appear on the local filesystem...')  # noqa: E501
        count = 0
        while not os.path.exists(self.joy_device):
            time.sleep(1)
            count += 1
            if count == 10:
                count = 0
                self.get_logger().warning(f'Still waiting for {self.joy_device} to appear on the local filesystem...')  # noqa: E501

        self.get_logger().info('Getting MAC address from udev...')
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
                mac = result[0].decode().strip().split('==')[1].replace('"', '')
                if mac:
                    self.get_logger().info(f'MAC address of {self.joy_device} is {mac}')
                    return mac
                else:
                    self.get_logger().warning(f'{self.joy_device} has no MAC. Is it a wired controller?') # noqa: E501
                    return None
            except Exception as err:
                self.get_logger().warning(f'Failed to read MAC address: {err}')
                return None
        else:
            self.get_logger().warning('Failed to read MAC address: no output')
            return None

    def check_quality(self):
        """Check the quality of the link and publish it"""
        try:
            hcitool_proc = subprocess.Popen(
                [
                    'hcitool',
                    'lq',
                    self.mac_addr
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            result = hcitool_proc.communicate()
            stdout = result[0].decode().strip()
            stderr = result[1].decode().strip()

            engage_stop = Bool()
            quality_level = Int32()
            if 'not connected' in stderr.lower():
                self.get_logger().warning('Controller not connected')
                engage_stop.data = True
                quality_level.data = 0
            else:
                quality_level.data = int(stdout.split(':')[-1].strip())
                engage_stop.data = quality_level.data < self.quality_cutoff

            self.stop_pub.publish(engage_stop)
            self.quality_pub.publish(quality_level)
        except Exception as err:
            self.get_logger().warning(f'Failed to read quality: {err}')

    def fake_quality(self):
        """Callback for the quality check for wired controllers to avoid locking out the mux."""
        engage_stop = Bool()
        engage_stop.data = False
        quality_level = Int32()
        quality_level.data = 255

        self.stop_pub.publish(engage_stop)
        self.quality_pub.publish(quality_level)

def main():
    rclpy.init()
    node = QualityCutoffNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
