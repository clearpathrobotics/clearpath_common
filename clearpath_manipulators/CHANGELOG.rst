^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_manipulators
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.8.5 (2026-01-30)
------------------

2.8.4 (2025-12-22)
------------------

2.8.3 (2025-12-16)
------------------

2.8.2 (2025-11-06)
------------------

2.8.1 (2025-10-28)
------------------

2.8.0 (2025-10-23)
------------------

2.7.4 (2025-09-18)
------------------

2.7.3 (2025-09-16)
------------------

2.7.2 (2025-09-08)
------------------

2.7.1 (2025-09-08)
------------------
* Feature: Franka in Jazzy (`#255 <https://github.com/clearpathrobotics/clearpath_common/issues/255>`_)
  * Forward from Humble: Feature Franka
  * Initial franka addition
  * Create joint instead of using connected_to parameter
  * Custom entry for generating Franka param
  * Franka control for multiple types
  * Modify entire Franka arm xacro
  * Add franka gripper
  * Copy and modify franka description
  * Add cutout for Franka arm_id
  * Add dependency for franka_description
  * Fix: Franka Update (`#238 <https://github.com/clearpathrobotics/clearpath_common/issues/238>`_)
  * Pass version parameter to Franka Hardware Interface
  * Use franka_arm macro and always add position and velocity interfaces
  * Only use effort
  * Feature: Franka Hand (`#244 <https://github.com/clearpathrobotics/clearpath_common/issues/244>`_)
  * Remove ros2_control controllers
  * Move gazebo control to a separate file
  * Pass arm_id to Franka SRDF
  * Pass poses from config with arm_id on Franka
  * Common method to replace placeholders in parameter files
  * Use appropriate placeholders in parameters
  * Replace controller_name placeholders at all points
  * Lint
* Contributors: luis-camero

2.7.0 (2025-08-25)
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

2.5.1 (2025-06-17)
------------------

2.5.0 (2025-05-29)
------------------

2.3.2 (2025-04-30)
------------------
* Add srdf plugins to package xml (`#204 <https://github.com/clearpathrobotics/clearpath_common/issues/204>`_)
* Contributors: Luis Camero

2.3.1 (2025-04-16)
------------------

2.3.0 (2025-04-11)
------------------
* Added delay to manipulator controller (`#191 <https://github.com/clearpathrobotics/clearpath_common/issues/191>`_)
* Added MoveIt Parameters and Enable (`#166 <https://github.com/clearpathrobotics/clearpath_common/issues/166>`_) (`#189 <https://github.com/clearpathrobotics/clearpath_common/issues/189>`_)
* Contributors: Luis Camero

2.2.2 (2025-04-09)
------------------

2.2.1 (2025-04-07)
------------------
* Fix: Rename ompl to _ompl and update to match Jazzy (`#186 <https://github.com/clearpathrobotics/clearpath_common/issues/186>`_)
* Contributors: Luis Camero

2.2.0 (2025-03-11)
------------------

2.1.0 (2025-01-31)
------------------

2.0.3 (2025-01-21)
------------------

2.0.2 (2025-01-20)
------------------

2.0.1 (2025-01-17)
------------------

2.0.0 (2025-01-16)
------------------

1.0.0 (2024-11-25)
------------------
* Added minimum version.
* Fix controller names and kinematics (`#109 <https://github.com/clearpathrobotics/clearpath_common/issues/109>`_)
* Contributors: Tony Baltovski, luis-camero

0.3.4 (2024-10-08)
------------------

0.3.3 (2024-10-04)
------------------

0.3.2 (2024-09-29)
------------------

0.3.1 (2024-09-23)
------------------

0.3.0 (2024-09-19)
------------------
* Fixed package version on new packages to match old packages.
* Changes.
* Fixed all license headers
* Added MoveIt to manipulation service launch
* Removed old ompl
* Added '_' to omp to make it always first alphabetically
* Added all chomp parameters explicitly to config
* Renamed manipulators package
* Contributors: Luis Camero, Tony Baltovski

* Fixed all license headers
* Added MoveIt to manipulation service launch
* Removed old ompl
* Added '_' to omp to make it always first alphabetically
* Added all chomp parameters explicitly to config
* Renamed manipulators package
* Contributors: Luis Camero

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
