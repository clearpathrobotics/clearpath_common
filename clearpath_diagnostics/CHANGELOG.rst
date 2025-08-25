^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_diagnostics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.0 (2025-04-30)
------------------

2.3.3 (2025-04-17)
------------------

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
* Add foxglove dependency (`#230 <https://github.com/clearpathrobotics/clearpath_common/issues/230>`_)
* Fix/expected diag rates (`#227 <https://github.com/clearpathrobotics/clearpath_common/issues/227>`_)
  * loosen rate window for MCU to allow 1 message to be missed
  * Add lighting category to diagnostics aggregator conditionally
  * Set expected BMS Status rate based on battery type
  * Only create the Odometry category if ekf set true
  * Only create the Networking category if wireless watcher is enabled
  * Configurable tolerance window on diagnostics
  * Monitor estop instead of stop_status if stop status does not exist
  * Safety check for size of the labels lists
  * Add warning on estop status
  * Disable stop status rate by platform
* Contributors: Hilary Luo, luis-camero

2.5.1 (2025-06-17)
------------------

2.5.0 (2025-05-29)
------------------
* Allow broad window of BMS message rates (`#218 <https://github.com/clearpathrobotics/clearpath_common/issues/218>`_)
* Fix/fw version check (`#217 <https://github.com/clearpathrobotics/clearpath_common/issues/217>`_)
  * Add condition for mcu firmware version newer than apt package
  * Clarify wording for checks that are evaluated on boot
* Fix: Foxglove Bridge Superclient (`#214 <https://github.com/clearpathrobotics/clearpath_common/issues/214>`_)
* Feature: Foxglove Bridge (`#213 <https://github.com/clearpathrobotics/clearpath_common/issues/213>`_)
  * Add foxglove bridge launch and parameter file
  * Add foxglove bridge parameter to generator
* Move clearpath_diagnostics to clearpath_common (`#211 <https://github.com/clearpathrobotics/clearpath_common/issues/211>`_)
* Contributors: Hilary Luo, luis-camero

2.3.2 (2025-04-16)
------------------

2.3.1 (2025-04-14)
------------------
* Remove duplicate and incorrect current / voltage labels (`#197 <https://github.com/clearpathrobotics/clearpath_robot/issues/197>`_)
  The terms "measured voltage" or "measured current" are prepended to these labels where they are used.
* Contributors: Hilary Luo

2.3.0 (2025-04-11)
------------------
* Lint: Diagnostic move log to new line (`#191 <https://github.com/clearpathrobotics/clearpath_robot/issues/191>`_)
* Fix/estop diagnostics (`#184 <https://github.com/clearpathrobotics/clearpath_robot/issues/184>`_)
* Feature/clear stale diagnostics (`#183 <https://github.com/clearpathrobotics/clearpath_robot/issues/183>`_)
* Feature/diagnostic categories (`#175 <https://github.com/clearpathrobotics/clearpath_robot/issues/175>`_)
* Feature/inventus diagnostics (`#170 <https://github.com/clearpathrobotics/clearpath_robot/issues/170>`_)

* Contributors: Hilary Luo, Tony Baltovski, Luis Camero

2.2.4 (2025-04-07)
------------------

2.2.3 (2025-03-20)
------------------
* [clearpath_diagnostics] Updated for changes to MCU status message.
* Contributors: Tony Baltovski

2.2.2 (2025-03-17)
------------------

2.2.1 (2025-03-12)
------------------

2.2.0 (2025-03-11)
------------------
* Feature/generated agg yaml (`#158 <https://github.com/clearpathrobotics/clearpath_robot/issues/158>`_)
  * Clear out sensor categories that will be generated
  * Improve logging and error handling
  * Correction to dependencies
* Fix BMS frequency diagnostic (`#157 <https://github.com/clearpathrobotics/clearpath_robot/issues/157>`_)
* Added Lynx motor driver diagnostics (`#149 <https://github.com/clearpathrobotics/clearpath_robot/issues/149>`_)
  * Removed trailing spaces
  * Added Lynx motor diagnostics
* Feature/lighting diagnostics (`#144 <https://github.com/clearpathrobotics/clearpath_robot/issues/144>`_)
  * Add lighting diagnostics
  * Remap lighting diagnostic topic
  * Set diagnostic updater hardware id to platform since serial isn't locally available
  * Improve clarity of diagnostic summary text
* Feature/battery diagnostics (`#142 <https://github.com/clearpathrobotics/clearpath_robot/issues/142>`_)
  * Add missing DiagnosticStatus dependency
  * Monitor BatteryState, Power and StopStatus messages
  * Declare methods before member variables
  * Move the template function to the cpp since it is private
  * Only display error summaries if a message has been received
* Contributors: Hilary Luo

2.1.2 (2025-02-28)
------------------

2.1.1 (2025-02-06)
------------------

2.1.0 (2025-01-31)
------------------
* Feature/diagnostics (`#135 <https://github.com/clearpathrobotics/clearpath_robot/issues/135>`_)
  * Initial port of diagnostics to C++
  * Remap axis camera topics to match API
  * Monitor MCU Status message frequency
  * Added firmware version check
  * Group MCU diagnostics together
  * Improve messaging around firmware versions
  * Disable MCU diagnostics for A200
* Contributors: Hilary Luo

2.0.4 (2025-01-22)
------------------

2.0.3 (2025-01-17)
------------------
* [clearpath_diagnostics] Fixed version.
* Contributors: Tony Baltovski

2.0.2 (2025-01-17)
------------------

2.0.1 (2025-01-17)
------------------

2.0.0 (2025-01-17)
------------------
* Fix hard-coded humble packages (`#117 <https://github.com/clearpathrobotics/clearpath_robot/issues/117>`_)
* Contributors: Chris Iverach-Brereton, Luis Camero, Tony Baltovski

1.1.0 (2025-01-15)
------------------

1.0.1 (2024-11-28)
------------------

1.0.0 (2024-11-26)
------------------
* Added minimum version.
* Remove battery_state from CMakeLists
* Move battery_state to clearpath_hardware_interfaces
* Contributors: Luis Camero, Tony Baltovski

0.3.2 (2024-10-04)
------------------

0.3.1 (2024-09-23)
------------------

0.3.0 (2024-09-19)
------------------
* R100 Initial Battry
* Contributors: Luis Camero, luis-camero

0.2.15 (2024-08-12)
-------------------

0.2.14 (2024-08-08)
-------------------

0.2.13 (2024-07-30)
-------------------

0.2.12 (2024-07-22)
-------------------
* Use PathJoinSubstitution for setup_path
* Contributors: Luis Camero

0.2.11 (2024-05-28)
-------------------

0.2.10 (2024-05-16)
-------------------

0.2.9 (2024-05-16)
------------------

0.2.8 (2024-05-14)
------------------
* Even more lint errors
* More linting changes
* Fixed linting errors
* Contributors: Luis Camero

0.2.7 (2024-04-10)
------------------

0.2.6 (2024-04-08)
------------------

0.2.5 (2024-03-07)
------------------

0.2.4 (2024-01-19)
------------------

0.2.3 (2024-01-18)
------------------

0.2.2 (2024-01-10)
------------------
* Get topic without namespace to address duplicate namespacing
* Contributors: Hilary Luo

0.2.1 (2023-12-18)
------------------

0.2.0 (2023-12-13)
------------------
* Added S1P2 battery configuration
* Set battery charging status
* Added dingo to battery state control
* Added D100 and D150 to generator and battery node
* Generate lighting node
* Fixed status message firmware version
* J100 -> W200
* Removed shebang
* Use battery model and configuration from clearpath_config
* Removed HMI msg, encode Uint8 instead
* Initial battery control node
* Renamed to battery_state_estimator
  Added to robot generator
* Properties, capacity, voltage
  Create pub/sub only for LiION and SLA
* Added LUT for SLA
* Battery types and configurations
* rolling average
* Initial battery state publisher
* Pass setup path
* Get namespace from robot.yaml for diagnostics launch
  Added diagnostics launch to generator
* Check ros-humble-clearpath-firmware package version
* Add all sensors
* Firmware and sensor diagnostics
* Contributors: Roni Kreinin

0.1.3 (2023-10-04)
------------------

0.1.2 (2023-09-27)
------------------

0.1.1 (2023-09-11)
------------------

0.1.0 (2023-08-31)
------------------

0.0.3 (2023-08-15)
------------------

0.0.2 (2023-07-25)
------------------

0.0.1 (2023-07-20)
------------------
