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

"""
Scan for a DualSense (PS5) controller, then trust, pair, and connect it.

Usage::

    pair_bt_controller [MAC]

With no argument the script scans for up to ``SCAN_TIMEOUT`` seconds and
picks the first DualSense that is actively advertising (i.e. in pairing mode).
Devices that are merely cached/previously paired in BlueZ are ignored unless
they are seen advertising during the scan.
With a MAC argument it waits for that specific controller to advertise.

If the selected controller was paired before, its stale BlueZ bond is removed
first so a fresh link key is negotiated.

Before running, put the DualSense into pairing mode:
    Hold Create + PS until the light bar pulses blue.

After the first successful pair the controller will reconnect automatically
on every subsequent power-on without running this script again.
"""

import collections
import os
import pty
import re
import select
import subprocess
import sys
import time

SCAN_TIMEOUT = int(os.environ.get('SCAN_TIMEOUT', 60))
CMD_TIMEOUT = 30
REDISCOVER_TIMEOUT = 20
STABLE_TIME = 5
DEVICE_NAME = 'DualSense'
MAC = r'(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}'
MAC_RE = re.compile(MAC)
ANSI_RE = re.compile(r'\x1b\[[0-9;?]*[A-Za-z]|[\x01\x02\r]')
DEVICE_EVENT_RE = re.compile(rf'\[(NEW|CHG)\] Device ({MAC}) (.*)')
CONNECTED_RE = re.compile(rf'\[CHG\] Device ({MAC}) Connected: (yes|no)')
AGENT_PROMPT_RE = re.compile(r'\(yes/no\)')


class PairingError(Exception):
    """Raised when a bluetoothctl step fails or times out."""


def cached_devices():
    """Return {MAC: name} for every device already in BlueZ's cache."""
    out = subprocess.run(
        ['bluetoothctl', 'devices'], capture_output=True, text=True
    ).stdout
    return {
        m.group(1).upper(): m.group(2)
        for m in re.finditer(rf'Device ({MAC}) (.*)', out)
    }


class BluetoothCtl:
    """A single long-lived interactive bluetoothctl session on a PTY."""

    def __init__(self):
        self._master, slave = pty.openpty()
        self._proc = subprocess.Popen(
            ['bluetoothctl'],
            stdin=slave,
            stdout=slave,
            stderr=slave,
            close_fds=True,
        )
        os.close(slave)
        self._buf = ''
        self._pending_lines = collections.deque()
        self.connected = {}

    def send(self, cmd):
        """Write a command to the bluetoothctl prompt."""
        os.write(self._master, (cmd + '\n').encode())

    def lines(self, timeout):
        """Yield output lines until timeout, auto-accepting agent prompts."""
        deadline = time.monotonic() + timeout
        while True:
            if self._pending_lines:
                line = self._pending_lines.popleft()
                m = CONNECTED_RE.search(line)
                if m:
                    self.connected[m.group(1).upper()] = m.group(2) == 'yes'
                yield line
                continue

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            r, _, _ = select.select([self._master], [], [], min(1.0, remaining))
            if not r:
                continue
            try:
                data = os.read(self._master, 4096)
            except OSError:
                data = b''
            if not data:
                raise PairingError('bluetoothctl exited unexpectedly')
            self._buf += ANSI_RE.sub('', data.decode('utf-8', errors='replace'))
            *complete, self._buf = self._buf.split('\n')
            # Agent prompts have no trailing newline.
            if AGENT_PROMPT_RE.search(self._buf):
                self.send('yes')
                self._buf = ''
            self._pending_lines.extend(complete)

    def wait_for(self, success, failure=None, timeout=CMD_TIMEOUT):
        """Return the first success match, or raise on failure/timeout."""
        for line in self.lines(timeout):
            if failure and re.search(failure, line):
                raise PairingError(line.strip())
            m = re.search(success, line)
            if m:
                return m
        return None

    def close(self):
        """Quit bluetoothctl and release the PTY."""
        try:
            self.send('quit')
            self._proc.wait(timeout=3)
        except (OSError, subprocess.TimeoutExpired):
            self._proc.terminate()
            self._proc.wait()
        try:
            os.close(self._master)
        except OSError:
            pass


def wait_for_advertising(bt, cache, timeout, mac=None):
    """Return the MAC of a DualSense (or `mac`) that is actively advertising."""
    for line in bt.lines(timeout):
        m = DEVICE_EVENT_RE.search(line)
        if not m:
            continue
        found, rest = m.group(2).upper(), m.group(3)
        if mac and found != mac:
            continue
        if found in cache:
            # Cached devices are replayed as [NEW] at startup; only a fresh RSSI
            # report proves the controller is advertising right now.
            if rest.startswith('RSSI') and (mac or DEVICE_NAME in cache[found]):
                return found
        elif mac or DEVICE_NAME in rest:
            return found
    return None


def trust_pair_connect(bt, mac):
    """Trust, pair, and connect `mac`, then verify the link stays up."""
    not_available = rf'Device {mac} not available'

    bt.send('scan off')
    bt.wait_for(r'Discovery stopped|Discovering: no', timeout=5)

    # Trusting first lets BlueZ accept the controller's incoming HID
    # connection without an authorization prompt.
    print(f'Trusting {mac} ...')
    bt.send(f'trust {mac}')
    if not bt.wait_for(
        rf'trust succeeded|{mac} Trusted: yes',
        rf'Failed to set .*trust|{not_available}',
    ):
        raise PairingError(f'Timed out trusting {mac}')

    print(f'Pairing with {mac} ...')
    bt.send(f'pair {mac}')
    if not bt.wait_for(
        rf'Pairing successful|{mac} (Paired|Bonded): yes',
        rf'Failed to pair|{not_available}',
    ):
        raise PairingError(f'Timed out pairing {mac}')

    if not bt.connected.get(mac):
        print(f'Connecting to {mac} ...')
        bt.send(f'connect {mac}')
        if not bt.wait_for(
            rf'Connection successful|{mac} Connected: yes|AlreadyConnected',
            rf'Failed to connect(?!.*(AlreadyConnected|InProgress))|{not_available}',
        ):
            raise PairingError(f'Timed out connecting to {mac}')

    if bt.wait_for(rf'{mac} Connected: no', timeout=STABLE_TIME):
        raise PairingError(f'{mac} disconnected right after connecting')


def main():
    """Entry point: pair a DualSense controller over Bluetooth."""
    mac = None
    if len(sys.argv) >= 2:
        mac = sys.argv[1].upper()
        if not MAC_RE.fullmatch(mac):
            print(
                f"Error: '{sys.argv[1]}' is not a valid Bluetooth MAC address.",
                file=sys.stderr,
            )
            sys.exit(1)

    cache = cached_devices()
    print(
        f'Hold Create + PS on the DualSense until the light bar pulses blue, '
        f'then wait for up to {SCAN_TIMEOUT}s...'
    )

    bt = BluetoothCtl()
    try:
        bt.send('power on')
        bt.wait_for(r'power on succeeded|Powered: yes', timeout=5)
        bt.send('agent NoInputNoOutput')
        bt.send('default-agent')
        bt.send('scan on')

        found = wait_for_advertising(bt, cache, SCAN_TIMEOUT, mac)
        if not found:
            raise PairingError(
                f'No {"device " + mac if mac else DEVICE_NAME} found in pairing mode '
                f'within {SCAN_TIMEOUT}s'
            )
        print(f'Found: {cache.get(found, DEVICE_NAME)} ({found})')

        if found in cache:
            # A stale bond makes the controller reject the link and power off.
            print(f'Removing previous pairing for {found} ...')
            bt.send(f'remove {found}')
            bt.wait_for(rf'\[DEL\] Device {found}|has been removed', timeout=10)
            del cache[found]
            if not wait_for_advertising(bt, cache, REDISCOVER_TIMEOUT, found):
                raise PairingError(f'{found} was not rediscovered after removing it')

        trust_pair_connect(bt, found)
    except PairingError as e:
        print(f'\nError: {e}', file=sys.stderr)
        print(
            'Put the controller back in pairing mode '
            '(hold Create + PS until the light bar pulses blue) and try again.',
            file=sys.stderr,
        )
        sys.exit(1)
    finally:
        bt.close()

    print(f"\nDone. '{found}' is now paired, trusted, and connected.")
    print('It will reconnect automatically on future power-ons.')


if __name__ == '__main__':
    main()
