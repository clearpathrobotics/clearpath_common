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
Scan for a DualSense (PS5) controller, then pair, trust, and connect it.

Usage::

    pair_bt_controller [MAC]

With no argument the script scans for up to ``SCAN_TIMEOUT`` seconds,
auto-detects the first DualSense that appears, and pairs it.
With a MAC argument it skips scanning and goes straight to pairing.

Before running, put the DualSense into pairing mode:
    Hold Create + PS until the light bar pulses blue.

After the first successful pair the controller will reconnect automatically
on every subsequent power-on without running this script again.
"""

import os
import pty
import re
import select
import subprocess
import sys
import time

SCAN_TIMEOUT = int(os.environ.get('SCAN_TIMEOUT', 60))
DEVICE_NAME = 'DualSense'
MAC_RE = re.compile(r'([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}')
ANSI_RE = re.compile(r'\x1b\[[0-9;]*[A-Za-z]')


def bluetoothctl(*commands):
    """Send a sequence of commands to bluetoothctl via stdin."""
    stdin = '\n'.join(commands) + '\n'
    subprocess.run(['bluetoothctl'], input=stdin, text=True, check=True)


def pair_and_trust(mac):
    """Pair, trust, and connect the controller at the given MAC address."""
    print(f'Pairing with {mac} ...')
    bluetoothctl('power on', 'agent on', 'default-agent',
                 f'pair {mac}', f'trust {mac}', f'connect {mac}')
    print(f"\nDone. '{mac}' is now paired, trusted, and connected.")
    print('It will reconnect automatically on future power-ons.')


def scan_for_dualsense():
    """Open a PTY-backed bluetoothctl session and return the MAC of the first DualSense found."""
    print(
        f'Hold Create + PS on the DualSense until the light bar pulses blue, '
        f'then waiting up to {SCAN_TIMEOUT}s...'
    )
    master, slave = pty.openpty()
    proc = subprocess.Popen(
        ['bluetoothctl'],
        stdin=slave, stdout=slave, stderr=slave,
        close_fds=True,
    )
    os.close(slave)
    os.write(master, b'power on\nscan on\n')

    mac = None
    buf = b''
    deadline = time.monotonic() + SCAN_TIMEOUT
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        r, _, _ = select.select([master], [], [], min(1.0, remaining))
        if r:
            try:
                buf += os.read(master, 4096)
            except OSError:
                break
            lines = buf.split(b'\n')
            buf = lines[-1]
            for line in lines[:-1]:
                text = ANSI_RE.sub('', line.decode('utf-8', errors='replace'))
                if DEVICE_NAME in text:
                    m = MAC_RE.search(text)
                    if m:
                        mac = m.group(0)
                        print(f'Found: {DEVICE_NAME} ({mac})')
                        break
        if mac:
            break

    try:
        os.write(master, b'scan off\nquit\n')
    except OSError:
        pass
    proc.terminate()
    proc.wait()
    try:
        os.close(master)
    except OSError:
        pass
    return mac


def main():
    """Entry point: pair a DualSense controller over Bluetooth."""
    if len(sys.argv) >= 2:
        mac = sys.argv[1]
        if not MAC_RE.fullmatch(mac):
            print(f"Error: '{mac}' is not a valid Bluetooth MAC address.", file=sys.stderr)
            sys.exit(1)
        # Brief scan so the adapter registers the device before pairing
        subprocess.run(['bluetoothctl', 'power', 'on'], check=True)
        proc = subprocess.Popen(
            ['bluetoothctl', 'scan', 'on'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        time.sleep(2)
        proc.terminate()
        proc.wait()
        subprocess.run(['bluetoothctl', 'scan', 'off'], capture_output=True)
        pair_and_trust(mac)
        return

    mac = scan_for_dualsense()
    if not mac:
        print(f'No DualSense found within {SCAN_TIMEOUT}s.', file=sys.stderr)
        print(
            'Ensure the controller is in pairing mode '
            '(hold Create + PS until the light bar pulses blue) and try again.',
            file=sys.stderr
        )
        sys.exit(1)

    pair_and_trust(mac)


if __name__ == '__main__':
    main()
