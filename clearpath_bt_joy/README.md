# clearpath_bt_joy

Monitors a Bluetooth joystick's link quality and cuts off its input if the link degrades or
disappears.

This provides a safety mechanism by which controllers that go out-of-range and latch their
last-received input will not force the robot into an uncontrollable state.

## How it works

`bt_joy_cutoff_node` reads raw HID reports from the controller's `/dev/hidrawN` node directly
(resolved from the `joy_device` parameter), rather than listening on `/joy`. This catches the
case where `joy_linux` keeps republishing the last-received values after the BT link drops.
