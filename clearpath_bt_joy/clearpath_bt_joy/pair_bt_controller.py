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

import os
import queue
import re
import sys
import time

SCAN_TIMEOUT = int(os.environ.get('SCAN_TIMEOUT', 60))
CMD_TIMEOUT = 30
REDISCOVER_TIMEOUT = 20
STABLE_TIME = 5
DEVICE_NAME = 'DualSense'
MAC = r'(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}'
MAC_RE = re.compile(MAC)
BLUEZ = 'org.bluez'
OBJECT_MANAGER = 'org.freedesktop.DBus.ObjectManager'
PROPERTIES = 'org.freedesktop.DBus.Properties'
ADAPTER = 'org.bluez.Adapter1'
DEVICE = 'org.bluez.Device1'
AGENT_MANAGER = 'org.bluez.AgentManager1'
AGENT_PATH = '/com/clearpath/bt_joy/agent'
HID_UUID = '00001124-0000-1000-8000-00805f9b34fb'


class PairingError(Exception):
    """Raised when a BlueZ D-Bus operation fails or times out."""


def _unwrap(value):
    """Convert D-Bus variants and containers into ordinary Python values."""
    if hasattr(value, 'value'):
        return _unwrap(value.value)
    if isinstance(value, dict):
        return {key: _unwrap(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_unwrap(item) for item in value]
    return value


def _is_dualsense(properties, mac=None):
    """Return whether Device1 properties identify the requested DualSense."""
    name = properties.get('Name') or properties.get('Alias') or ''
    address = properties.get('Address', '').upper()
    return (name == 'DualSense Wireless Controller') and (
        (mac is None) or (address == mac)
    )


def _bluez_error(reply):
    """Format a structured D-Bus error reply for a user-facing pairing error."""
    if hasattr(reply, 'get_dbus_name'):
        name = reply.get_dbus_name()
    else:
        name = type(reply).__name__
    return f'{name}: {reply}'.rstrip(': ')


class BlueZPairing:
    """Pair a DualSense through BlueZ's structured system D-Bus API."""

    def __init__(self, mac=None):
        self.mac = mac  # Optional address to restrict which DualSense is selected.
        self.bus = None  # System D-Bus connection used to communicate with BlueZ.
        self.dbus = None  # Imported dbus-python module used for D-Bus types and calls.
        self.context = None  # GLib event loop context that dispatches D-Bus callbacks.
        self.objects = {}  # Latest cached BlueZ adapter and device properties.
        self.adapter_path = None  # BlueZ object path of the selected Bluetooth radio.
        self.discovery_started = False  # If instance must stop scanning on exit.
        self.agent_registered = False  # If instance must unregister its pairing agent.

        # Exported Agent1 object used to authorize controller pairing.
        self.agent = None

        # Device paths reported during Bluetooth discovery.
        self.device_events = queue.Queue()

        # D-Bus signal subscriptions removed during cleanup.
        self.signal_matches = []

    def connect(self):
        """Connect to the system bus and prepare the BlueZ client."""
        try:
            import dbus
            from dbus.mainloop.glib import DBusGMainLoop
            import dbus.service
            from gi.repository import GLib
        except ImportError as exc:
            raise PairingError(
                'Python D-Bus bindings are required; install python3-dbus '
                'and python3-gi'
            ) from exc

        self.dbus = dbus
        try:
            DBusGMainLoop(set_as_default=True)
            self.bus = dbus.SystemBus()
            self.context = GLib.MainContext.default()
        except Exception as exc:
            raise PairingError(f'Could not connect to the system D-Bus: {exc}') from exc

        # Setup Listener for InterfacesAdded signals.
        # New Device1 objects can appear as controllers start advertising.
        self.signal_matches.append(
            self.bus.add_signal_receiver(
                self._on_interfaces_added,
                signal_name='InterfacesAdded',
                dbus_interface=OBJECT_MANAGER,
                path='/',
                bus_name=BLUEZ,
            )
        )
        # Listen for Device1 RSSI and connection-state updates.
        self.signal_matches.append(
            self.bus.add_signal_receiver(
                self._on_properties_changed,
                signal_name='PropertiesChanged',
                dbus_interface=PROPERTIES,
                bus_name=BLUEZ,
                path_keyword='path',
            )
        )

    def _wait_until(self, predicate, timeout):
        """Dispatch GLib D-Bus work until a callback completes or times out."""
        deadline = time.monotonic() + timeout
        while not predicate():
            self.context.iteration(False)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            time.sleep(min(0.01, remaining))
        return True

    def _call(self, path, interface, member, *args, timeout=CMD_TIMEOUT):
        """Call a BlueZ method asynchronously while dispatching D-Bus callbacks."""
        result = {'done': False, 'body': None, 'error': None}

        def on_reply(*body):
            result['body'] = body
            result['done'] = True

        def on_error(error):
            result['error'] = error
            result['done'] = True

        try:
            proxy = self.bus.get_object(BLUEZ, path, introspect=False)
            method = getattr(self.dbus.Interface(proxy, interface), member)
            method(
                *args,
                reply_handler=on_reply,
                error_handler=on_error,
                timeout=timeout,
            )
        except Exception as exc:
            raise PairingError(f'BlueZ {interface}.{member} failed: {exc}') from exc

        if self._wait_until(lambda: result['done'], timeout):
            if result['error'] is not None:
                raise PairingError(_bluez_error(result['error']))
            return result.get('body', ())

        # If we reach here, the call did not complete within the timeout.
        raise PairingError(f'BlueZ {interface}.{member} timed out')

    def get_objects(self):
        """Fetch and cache BlueZ's current ObjectManager snapshot."""
        body = self._call('/', OBJECT_MANAGER, 'GetManagedObjects')
        self.objects = _unwrap(body[0])
        return self.objects

    def _on_interfaces_added(self, path, interfaces):
        """Cache new BlueZ objects and enqueue newly discovered devices."""
        path = str(path)
        interfaces = _unwrap(interfaces)
        self.objects.setdefault(path, {}).update(interfaces)
        device = self.objects[path].get(DEVICE)
        if device and 'RSSI' in device:
            self.device_events.put(path)

    def _on_properties_changed(self, interface, changed, invalidated, path=None):
        """Update cached properties and signal newly reported RSSI values."""
        path = str(path)
        if interface != DEVICE or path not in self.objects:
            return
        properties = self.objects[path].setdefault(DEVICE, {})
        properties.update(_unwrap(changed))
        for name in invalidated:
            properties.pop(name, None)
        if 'RSSI' in changed:
            self.device_events.put(path)

    def select_adapter(self):
        """Choose a powered adapter, powering on the first if necessary."""
        objects = self.get_objects()
        adapters = [
            (path, interfaces[ADAPTER])
            for path, interfaces in objects.items()
            if ADAPTER in interfaces
        ]
        if not adapters:
            raise PairingError('BlueZ has no Bluetooth adapter')
        self.adapter_path, properties = next(
            ((path, props) for path, props in adapters if props.get('Powered')),
            adapters[0],
        )
        if not properties.get('Powered'):
            print('Powering on the Bluetooth adapter ...')
            self._call(
                self.adapter_path,
                PROPERTIES,
                'Set',
                ADAPTER,
                'Powered',
                self.dbus.Boolean(True, variant_level=1),
            )
        for path, interfaces in objects.items():
            if DEVICE in interfaces:
                self.objects[path] = interfaces

    def start_discovery(self):
        """Start BlueZ device discovery on the selected adapter."""
        self._call(self.adapter_path, ADAPTER, 'StartDiscovery')
        self.discovery_started = True

    def stop_discovery(self):
        """Stop this client's BlueZ discovery session."""
        if self.discovery_started:
            self._call(self.adapter_path, ADAPTER, 'StopDiscovery')
            self.discovery_started = False

    def find_advertising_device(self, timeout):
        """Wait for a matching DualSense with a fresh RSSI discovery update."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.context.iteration(False)
            try:
                path = str(self.device_events.get_nowait())
            except queue.Empty:
                time.sleep(0.01)
                continue
            properties = self.objects.get(path, {}).get(DEVICE, {})
            if properties.get('Adapter') != self.adapter_path:
                continue
            if _is_dualsense(properties, self.mac):
                return path, properties
        return None

    def remove_existing_bond(self, path, properties):
        """Remove an existing bond and rediscover the device before pairing."""
        if not properties.get('Paired'):
            return path, properties
        address = properties['Address'].upper()
        print(f'Removing previous pairing for {address} ...')
        self.stop_discovery()
        self._call(
            self.adapter_path,
            ADAPTER,
            'RemoveDevice',
            self.dbus.ObjectPath(path),
        )
        self.objects.pop(path, None)
        while not self.device_events.empty():
            self.device_events.get_nowait()
        self.start_discovery()
        result = self.find_advertising_device(REDISCOVER_TIMEOUT)
        if not result:
            raise PairingError(
                f'{address} was not rediscovered after removing its bond'
            )
        return result

    def register_agent(self, device_path):
        """Register an agent that authorizes only this controller's HID service."""
        dbus = self.dbus

        class Rejected(dbus.DBusException):
            _dbus_error_name = 'org.bluez.Error.Rejected'

        def agent_method(member, in_signature='', out_signature=''):
            def decorate(callback):
                callback.__name__ = member
                return dbus.service.method(
                    'org.bluez.Agent1',
                    in_signature=in_signature,
                    out_signature=out_signature,
                )(callback)

            return decorate

        class PairingAgent(dbus.service.Object):
            """Authorize pairing callbacks for the selected controller only."""

            def __init__(self, bus, target_path):
                super().__init__(bus, AGENT_PATH)
                self.target_path = str(target_path)

            def _check_device(self, path):
                if str(path) != self.target_path:
                    raise Rejected('Unexpected Bluetooth device')

            @agent_method('Release')
            def release(self):
                return

            @agent_method('RequestAuthorization', 'o')
            def request_authorization(self, device):
                self._check_device(device)

            @agent_method('AuthorizeService', 'os')
            def authorize_service(self, device, uuid):
                self._check_device(device)
                if uuid.lower() != HID_UUID:
                    raise Rejected('Only the HID service is allowed')

            @agent_method('RequestConfirmation', 'ou')
            def request_confirmation(self, device, passkey):
                self._check_device(device)

            @agent_method('RequestPinCode', 'o', 's')
            def request_pin_code(self, device):
                self._check_device(device)
                raise Rejected('PIN entry is not supported')

            @agent_method('RequestPasskey', 'o', 'u')
            def request_passkey(self, device):
                self._check_device(device)
                raise Rejected('Passkey entry is not supported')

            @agent_method('DisplayPinCode', 'os')
            def display_pin_code(self, device, pincode):
                self._check_device(device)

            @agent_method('DisplayPasskey', 'ouq')
            def display_passkey(self, device, passkey, entered):
                self._check_device(device)

            @agent_method('Cancel')
            def cancel(self):
                return

        self.agent = PairingAgent(self.bus, device_path)
        self._call(
            '/org/bluez',
            AGENT_MANAGER,
            'RegisterAgent',
            self.dbus.ObjectPath(AGENT_PATH),
            'NoInputNoOutput',
        )
        self.agent_registered = True

    def trust_pair_connect(self, path, properties):
        """Trust, pair, connect, and verify stable connection state."""
        address = properties['Address'].upper()
        self.register_agent(path)
        self.stop_discovery()
        self._call(
            path,
            PROPERTIES,
            'Set',
            DEVICE,
            'Trusted',
            self.dbus.Boolean(True, variant_level=1),
        )
        print(f'Pairing with {address} ...')
        self._call(path, DEVICE, 'Pair')

        properties = self.get_objects().get(path, {}).get(DEVICE, {})
        if not properties.get('Connected'):
            print(f'Connecting to {address} ...')
            self._call(path, DEVICE, 'Connect')

        deadline = time.monotonic() + CMD_TIMEOUT
        while time.monotonic() < deadline:
            properties = self.get_objects().get(path, {}).get(DEVICE, {})
            if properties.get('Connected'):
                break
            self.context.iteration(False)
            time.sleep(0.2)
        else:
            raise PairingError(f'Timed out connecting to {address}')

        print(f'Connected to {address}; checking link stability ...')
        deadline = time.monotonic() + STABLE_TIME
        while time.monotonic() < deadline:
            time.sleep(0.2)
            properties = self.get_objects().get(path, {}).get(DEVICE, {})
            if not properties.get('Connected'):
                raise PairingError(f'{address} disconnected right after connecting')

    def close(self):
        """Release discovery, the temporary agent, and the system bus."""
        if self.bus is None:
            return
        if self.discovery_started:
            try:
                self.stop_discovery()
            except PairingError:
                pass
        if self.agent_registered:
            try:
                self._call(
                    '/org/bluez',
                    AGENT_MANAGER,
                    'UnregisterAgent',
                    self.dbus.ObjectPath(AGENT_PATH),
                )
            except PairingError:
                pass
        if self.agent is not None:
            self.agent.remove_from_connection()
        for match in self.signal_matches:
            match.remove()
        self.bus.close()
        self.bus = None

    def run(self):
        """Run controller discovery and pairing, cleaning up even on failure."""
        try:
            self.connect()
            self.select_adapter()
            print(
                f'Hold Create + PS on the DualSense until the light bar pulses blue, '
                f'then wait for up to {SCAN_TIMEOUT}s...'
            )
            self.start_discovery()
            result = self.find_advertising_device(SCAN_TIMEOUT)
            if not result:
                target = f'device {self.mac}' if self.mac else DEVICE_NAME
                raise PairingError(
                    f'No {target} found in pairing mode within {SCAN_TIMEOUT}s'
                )
            path, properties = result
            address = properties['Address'].upper()
            name = properties.get('Alias') or properties.get('Name')
            print(f'Found: {name} ({address})')

            path, properties = self.remove_existing_bond(path, properties)
            self.trust_pair_connect(path, properties)
        finally:
            self.close()


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

    try:
        BlueZPairing(mac).run()
    except PairingError as exc:
        print(f'\nError: {exc}', file=sys.stderr)
        print(
            'Put the controller back in pairing mode '
            '(hold Create + PS until the light bar pulses blue) and try again.',
            file=sys.stderr,
        )
        sys.exit(1)

    print('\nDone. Controller is now paired, trusted, and connected.')
    print('It will reconnect automatically on future power-ons.')


if __name__ == '__main__':
    main()
