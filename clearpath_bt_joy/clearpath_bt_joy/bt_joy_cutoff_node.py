# Copyright 2026 Rockwell Automation Technologies, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright notice,
#      this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright notice,
#      this list of conditions and the following disclaimer in the documentation
#      and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its contributors
#      may be used to endorse or promote products derived from this software
#      without specific prior written permission.
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

from collections import deque
import os
import select
import threading
import time

from diagnostic_msgs.msg import DiagnosticStatus
import diagnostic_updater
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default
from std_msgs.msg import Bool, Int32


HIDRAW_READ_SIZE = 256


class BtJoyCutoffNode(Node):
    """Fail-closed BT joystick watchdog based on hidraw report-rate."""

    def __init__(self):
        super().__init__('bt_cutoff_node')

        self.publish_rate = float(self.declare_parameter('publish_rate', 10.0).value)
        self.joy_device = self.declare_parameter('joy_device', '').value or None
        self.expected_report_hz = float(
            self.declare_parameter('expected_report_hz', 200.0).value)
        self.quality_threshold_pct = int(
            self.declare_parameter('quality_threshold_pct', 40).value)

        if self.joy_device is None:
            self.get_logger().error('joy_device parameter is required')
            raise RuntimeError('joy_device parameter is required')

        self.stop_pub = self.create_publisher(Bool, 'bt_quality_stop', qos_profile_default)
        # This is reports recieved in a 1 second window.
        self.quality_pub = self.create_publisher(Int32, 'quality', qos_profile_default)

        self._lock = threading.Lock()
        self._stop_thread = False
        self._fd_healthy = False
        self._count = 0
        self._history = deque(maxlen=max(1, int(self.publish_rate)))
        self._quality_pct = 0
        self._rate_hz = 0.0

        self.hidraw_path = self._resolve_hidraw(self.joy_device)

        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

        self.create_timer(1.0 / self.publish_rate, self._publish)

        self._diag = diagnostic_updater.Updater(self)
        self._diag.setHardwareID(self.joy_device or 'unknown')
        self._diag.add('BT Joy Link', self._diag_task)

        self.get_logger().info(
            f'Watchdog active: joy_device={self.joy_device} '
            f'hidraw={self.hidraw_path} '
            f'expected={self.expected_report_hz:.0f} Hz '
            f'threshold={self.quality_threshold_pct}%')

    def destroy_node(self):
        self._stop_thread = True
        super().destroy_node()

    @staticmethod
    def _resolve_hidraw(js_path):
        """Find the /dev/hidrawN that shares the parent HID device with js_path."""
        if not js_path or not os.path.exists(js_path):
            return None
        try:
            real = os.path.realpath(js_path)
            js_name = os.path.basename(real)
            cur = os.path.realpath(f'/sys/class/input/{js_name}')
            for _ in range(6):
                cur = os.path.dirname(cur)
                hidraw_dir = os.path.join(cur, 'hidraw')
                if os.path.isdir(hidraw_dir):
                    for entry in sorted(os.listdir(hidraw_dir)):
                        if entry.startswith('hidraw'):
                            return f'/dev/{entry}'
        except OSError:
            return None
        return None

    def _hidraw_present(self):
        """Return True when the resolved hidraw node still exists in /dev."""
        return bool(self.hidraw_path) and os.path.exists(self.hidraw_path)

    def _reader_loop(self):
        """Hold an fd on the hidraw node; count every report; flag unhealthy on any failure."""
        while not self._stop_thread:
            if not self._hidraw_present():
                with self._lock:
                    self._fd_healthy = False
                self.hidraw_path = self._resolve_hidraw(self.joy_device)
                if not self.hidraw_path:
                    time.sleep(0.1)
                    continue
            try:
                fd = os.open(self.hidraw_path, os.O_RDONLY | os.O_NONBLOCK)
            except OSError as e:
                with self._lock:
                    self._fd_healthy = False
                self.get_logger().warning(f'open({self.hidraw_path}) failed: {e}')
                time.sleep(0.2)
                continue
            with self._lock:
                self._fd_healthy = True
            self.get_logger().info(f'Reading HID reports from {self.hidraw_path}')
            try:
                while not self._stop_thread:
                    r, _, _ = select.select([fd], [], [], 0.1)
                    if not r:
                        if not os.path.exists(self.hidraw_path):
                            break
                        continue
                    try:
                        data = os.read(fd, HIDRAW_READ_SIZE)
                    except OSError:
                        break
                    if not data:
                        break
                    with self._lock:
                        self._count += 1
            finally:
                with self._lock:
                    self._fd_healthy = False
                try:
                    os.close(fd)
                except OSError:
                    pass
            self.get_logger().warning(f'{self.hidraw_path} closed; will retry')
            time.sleep(0.1)

    def _sample_rate(self):
        """Rotate the per-tick counter into the 1 s history and return (rate_hz, quality_pct)."""
        with self._lock:
            self._history.append(self._count)
            self._count = 0
            window_s = len(self._history) / self.publish_rate
            count_1s = sum(self._history)
        rate_hz = count_1s / window_s if window_s > 0 else 0.0
        quality_pct = min(100, int(round(100.0 * rate_hz / self.expected_report_hz)))
        return rate_hz, quality_pct

    def _link_state(self, quality_pct):
        """Return (stop, reason) for the current link health."""
        with self._lock:
            fd_ok = self._fd_healthy
        if not self.hidraw_path:
            return True, 'No hidraw path resolved'
        if not self._hidraw_present():
            return True, 'hidraw node missing (link lost)'
        if not fd_ok:
            return True, 'Reader fd unhealthy'
        if quality_pct < self.quality_threshold_pct:
            return True, f'Link quality {quality_pct}% < {self.quality_threshold_pct}%'
        return False, f'Link healthy ({quality_pct}%)'

    def _publish(self):
        """Publish the current stop flag and quality percentage."""
        self._rate_hz, self._quality_pct = self._sample_rate()
        stop, _ = self._link_state(self._quality_pct)
        self.stop_pub.publish(Bool(data=stop))
        self.quality_pub.publish(Int32(data=0 if stop else self._quality_pct))

    def _diag_task(self, stat):
        """Populate a DiagnosticStatus with link state and rate metrics."""
        rate_hz, quality_pct = self._rate_hz, self._quality_pct
        stop, reason = self._link_state(quality_pct)
        with self._lock:
            fd_ok = self._fd_healthy
        stat.summary(DiagnosticStatus.ERROR if stop else DiagnosticStatus.OK, reason)
        stat.add('joy_device', str(self.joy_device))
        stat.add('hidraw_path', str(self.hidraw_path))
        stat.add('hidraw_present', str(self._hidraw_present()))
        stat.add('fd_healthy', str(fd_ok))
        stat.add('reports_per_sec', f'{rate_hz:.1f}')
        stat.add('expected_report_hz', f'{self.expected_report_hz:.0f}')
        stat.add('quality_pct', str(quality_pct))
        stat.add('quality_threshold_pct', str(self.quality_threshold_pct))
        return stat


def main():
    rclpy.init()
    node = BtJoyCutoffNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
