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

"""Tests for BlueZ D-Bus controller discovery and error handling."""

import sys
from types import ModuleType, SimpleNamespace

from clearpath_bt_joy import pair_bt_controller
import pytest


class FakeVariant:
    """Minimal variant stand-in for property conversion tests."""

    def __init__(self, value):
        self.value = value


class FakeDBusError(Exception):
    """Minimal named D-Bus error stand-in."""

    def get_dbus_name(self):
        return 'org.bluez.Error.AuthenticationFailed'


def test_unwrap_nested_dbus_values():
    """Recursively convert D-Bus variants and containers to Python values."""
    data = {'Name': FakeVariant('DualSense'), 'RSSI': FakeVariant(-48)}

    assert pair_bt_controller._unwrap(data) == {'Name': 'DualSense', 'RSSI': -48}


def test_is_dualsense_matches_name_and_optional_mac():
    """Match the gamepad interface and optionally require its address."""
    properties = {
        'Name': 'DualSense Wireless Controller',
        'Address': 'D4:2F:4B:1E:C0:13',
    }

    assert pair_bt_controller._is_dualsense(properties)
    assert pair_bt_controller._is_dualsense(properties, 'D4:2F:4B:1E:C0:13')
    assert not pair_bt_controller._is_dualsense(properties, 'AA:BB:CC:DD:EE:FF')
    assert not pair_bt_controller._is_dualsense(
        {
            'Name': 'DualSense Wireless Controller Motion Sensors',
            'Address': properties['Address'],
        }
    )


def test_bluez_error_formats_name_and_message():
    """Preserve BlueZ's named error and message for user diagnostics."""
    error = FakeDBusError('Pairing authentication failed')

    assert pair_bt_controller._bluez_error(error) == (
        'org.bluez.Error.AuthenticationFailed: Pairing authentication failed'
    )


def test_call_raises_structured_bluez_error():
    """Raise a D-Bus error name instead of parsing command output."""

    class FakeBus:
        def get_object(self, service, path, introspect=False):
            return object()

    class FakeInterface:
        def start_discovery(self, reply_handler, error_handler, timeout):
            error_handler(FakeDBusError('Pairing authentication failed'))

        def __getattr__(self, member):
            if member == 'StartDiscovery':
                return self.start_discovery
            raise AttributeError(member)

    class FakeDBus:
        @staticmethod
        def interface(proxy, interface):
            return FakeInterface()

        Interface = interface

    pairing = pair_bt_controller.BlueZPairing()
    pairing.dbus = FakeDBus
    pairing.bus = FakeBus()

    with pytest.raises(
        pair_bt_controller.PairingError,
        match='org.bluez.Error.AuthenticationFailed: Pairing authentication failed',
    ):
        pairing._call('/org/bluez/hci0', pair_bt_controller.ADAPTER, 'StartDiscovery')


def test_agent_exports_bluez_names_and_limits_authorization():
    """Expose Agent1 member names and authorize only HID for the target."""

    class FakeDBusError(Exception):
        pass

    class FakeObject:
        def __init__(self, bus, path):
            self.path = path

    class FakeService:
        Object = FakeObject

        @staticmethod
        def method(interface, in_signature='', out_signature=''):
            def decorate(callback):
                callback.dbus_member = callback.__name__
                callback.dbus_in_signature = in_signature
                callback.dbus_out_signature = out_signature
                return callback

            return decorate

    fake_dbus = SimpleNamespace(
        DBusException=FakeDBusError,
        ObjectPath=str,
        service=FakeService,
    )
    pairing = pair_bt_controller.BlueZPairing()
    pairing.dbus = fake_dbus
    pairing.bus = object()
    pairing._call = lambda *args: ()
    target_path = '/org/bluez/hci0/dev_D4_2F_4B_1E_C0_13'
    pairing.register_agent(target_path)
    agent = pairing.agent

    assert agent.authorize_service.dbus_member == 'AuthorizeService'
    assert agent.authorize_service.dbus_in_signature == 'os'
    agent.authorize_service(target_path, pair_bt_controller.HID_UUID)
    with pytest.raises(FakeDBusError):
        agent.authorize_service(target_path, '00000000-0000-0000-0000-000000000000')
    with pytest.raises(FakeDBusError):
        agent.request_authorization('/org/bluez/hci0/dev_unexpected')


def test_properties_changed_updates_rssi_and_queues_device():
    """Track a Device1 RSSI update as a discovery candidate event."""
    pairing = pair_bt_controller.BlueZPairing()
    path = '/org/bluez/hci0/dev_D4_2F_4B_1E_C0_13'
    pairing.objects[path] = {
        pair_bt_controller.DEVICE: {
            'Adapter': '/org/bluez/hci0',
            'Address': 'D4:2F:4B:1E:C0:13',
            'Name': 'DualSense Wireless Controller',
        }
    }

    pairing._on_properties_changed(
        pair_bt_controller.DEVICE,
        {'RSSI': FakeVariant(-51)},
        [],
        path=path,
    )

    assert pairing.objects[path][pair_bt_controller.DEVICE]['RSSI'] == -51
    assert pairing.device_events.get_nowait() == path


def test_connect_filters_dbus_signals_by_bluez_sender(monkeypatch):
    """Register signal receivers using dbus-python's bus_name filter argument."""
    receivers = []

    class FakeBus:
        def add_signal_receiver(self, handler, **kwargs):
            receivers.append((handler, kwargs))
            return object()

    bus = FakeBus()
    dbus = ModuleType('dbus')
    dbus.SystemBus = lambda: bus
    dbus.mainloop = ModuleType('dbus.mainloop')
    dbus.mainloop.glib = ModuleType('dbus.mainloop.glib')
    dbus.mainloop.glib.DBusGMainLoop = lambda **kwargs: None
    dbus.service = ModuleType('dbus.service')
    gi = ModuleType('gi')
    gi.repository = ModuleType('gi.repository')
    gi.repository.GLib = SimpleNamespace(
        MainContext=SimpleNamespace(default=lambda: object())
    )
    monkeypatch.setitem(sys.modules, 'dbus', dbus)
    monkeypatch.setitem(sys.modules, 'dbus.mainloop', dbus.mainloop)
    monkeypatch.setitem(sys.modules, 'dbus.mainloop.glib', dbus.mainloop.glib)
    monkeypatch.setitem(sys.modules, 'dbus.service', dbus.service)
    monkeypatch.setitem(sys.modules, 'gi', gi)
    monkeypatch.setitem(sys.modules, 'gi.repository', gi.repository)

    pairing = pair_bt_controller.BlueZPairing()
    pairing.connect()

    assert len(receivers) == 2
    assert all(
        kwargs['bus_name'] == pair_bt_controller.BLUEZ for _, kwargs in receivers
    )
    assert all('sender' not in kwargs for _, kwargs in receivers)
