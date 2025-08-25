^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_bt_joy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.6.4 (2025-07-29)
------------------

2.6.3 (2025-07-18)
------------------

2.6.2 (2025-07-14)
------------------

2.6.1 (2025-07-07)
------------------

2.6.0 (2025-07-04)
------------------

2.5.1 (2025-06-17)
------------------

2.5.0 (2025-05-29)
------------------
* Disable symlink rule to rely on DS4DRV instead
* Contributors: Luis Camero

2.3.2 (2025-04-30)
------------------

2.3.1 (2025-04-16)
------------------

2.3.0 (2025-04-11)
------------------

2.2.2 (2025-04-09)
------------------

2.2.1 (2025-04-07)
------------------

2.2.0 (2025-03-11)
------------------

2.1.0 (2025-01-31)
------------------

2.0.3 (2025-01-21)
------------------
* Bypass bluetooth link quality check (`#151 <https://github.com/clearpathrobotics/clearpath_common/issues/151>`_)
  * Comment-out the bluetooth cutoff node, the mux, and the remap from the joy_teleop to bypass the link quality check
  * Add missing newlines to joy teleop config files
  * Change the link quality check to be exclusive instead of inclusive
* Contributors: Chris Iverach-Brereton

2.0.2 (2025-01-20)
------------------
* Add additional logging, replace rate.sleep() with time.sleep(); on boot as part of a systemd job rate.sleep() is never exiting (`#148 <https://github.com/clearpathrobotics/clearpath_common/issues/148>`_)
* Contributors: Chris Iverach-Brereton

2.0.1 (2025-01-17)
------------------
* Explicitly use the default qos profile for the quality & cutoff pubs. Reduce the publish rate from 10Hz to 5Hz (on A300-000006 the 10Hz rate was missing deadlines, resulting in a deadlocked controller)
* Contributors: Chris Iverach-Brereton

2.0.0 (2025-01-16)
------------------
* Improve joy telep support (`#131 <https://github.com/clearpathrobotics/clearpath_common/issues/131>`_)
  * Update description, add bluez dependency
  * Add the quality_cutoff parameter to all of the PS4 configuration files
  * Update the launch file to start the new cutoff node and additional twist_mux
  * Add udev rules for various controllers
  * Add dependency on the new bt cutoff package
  * Add xbox controller parameters for all platforms
  * Fix the PS4/5 axis ordering; the left stick shows up as axis 3/4, with the l/r analogue triggers being 2 & 5. Update omni control configurations accordingly
  * Add PS5 udev rules, config files
  * Add a timeout to the quality lock so we lock out the controller if the quality-checker node crashes
  * Set the respawn flag for the BT cutoff node and the joy_linux node
  * Add additional logging when blocking for services & IPC. Since we've added a timeout to the lock, publish fake quality data when using wired controllers. Log when this happens
* Contributors: Chris Iverach-Brereton, Tony Baltovski

0.3.2 (2024-09-29)
------------------

0.3.1 (2024-09-23)
------------------

0.3.0 (2024-09-19)
------------------

0.2.11 (2024-08-08)
-------------------

0.2.10 (2024-07-25)
-------------------

0.2.9 (2024-05-28)
------------------

0.2.8 (2024-05-14)
------------------

0.2.7 (2024-04-08)
------------------

0.2.6 (2024-01-18)
------------------

0.2.5 (2024-01-15)
------------------

0.2.4 (2024-01-11)
------------------

0.2.3 (2024-01-08)
------------------

0.2.2 (2024-01-04)
------------------

0.2.1 (2023-12-21)
------------------

0.2.0 (2023-12-08)
------------------

0.1.3 (2023-11-03)
------------------

0.1.2 (2023-10-02)
------------------

0.1.1 (2023-08-25)
------------------

0.1.0 (2023-08-17)
------------------

0.0.9 (2023-07-31)
------------------

0.0.8 (2023-07-24)
------------------

0.0.7 (2023-07-19)
------------------

0.0.6 (2023-07-13)
------------------

0.0.5 (2023-07-12)
------------------

0.0.4 (2023-07-07)
------------------

0.0.3 (2023-07-05)
------------------

0.0.2 (2023-07-04)
------------------

0.0.1 (2023-06-21)
------------------
