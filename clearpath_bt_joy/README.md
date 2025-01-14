# clearpath_bt_joy

Monitors a bluetooth joy controller's link quality and cuts it off if the signal strength drops
below a given threshold.

This provides a safety mechanism by which controllers that go out-of-range and latch their
last-received input will not force the robot into an uncontrollable state.