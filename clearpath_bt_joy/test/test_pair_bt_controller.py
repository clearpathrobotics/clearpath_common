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

"""Tests for the Bluetooth controller pairing helper."""

import collections

from clearpath_bt_joy import pair_bt_controller
import pytest


def test_pairing_reports_disconnect_after_connect_success_same_pty_read(monkeypatch):
    """Catch a disconnect queued after connect success in the same PTY read."""
    mac = 'AA:BB:CC:DD:EE:FF'
    controller = pair_bt_controller.BluetoothCtl.__new__(
        pair_bt_controller.BluetoothCtl
    )
    controller._master = 1
    controller._buf = ''
    controller._pending_lines = collections.deque()
    controller.connected = {}
    read_output = iter(
        [
            (
                'Discovery stopped\n'
                'trust succeeded\n'
                'Pairing successful\n'
                f'[CHG] Device {mac} Connected: yes\n'
                'Connection successful\n'
                f'[CHG] Device {mac} Connected: no\n'
            ).encode()
        ]
    )

    monkeypatch.setattr(
        pair_bt_controller.select,
        'select',
        lambda readable, writable, exceptional, timeout: (readable, [], []),
    )
    monkeypatch.setattr(
        pair_bt_controller.os,
        'read',
        lambda file_descriptor, size: next(read_output),
    )
    monkeypatch.setattr(controller, 'send', lambda command: None)

    with pytest.raises(
        pair_bt_controller.PairingError,
        match='disconnected right after connecting',
    ):
        pair_bt_controller.trust_pair_connect(controller, mac)
    assert controller.connected[mac] is False
