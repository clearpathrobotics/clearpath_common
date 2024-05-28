^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.9 (2024-05-28)
------------------

0.2.8 (2024-05-14)
------------------

0.2.7 (2024-04-08)
------------------

0.2.6 (2024-01-18)
------------------
* Disable tf_frame_prefix_enable
* Contributors: Luis Camero

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
* Pass robot description to controller manager over topic
* Added W200 Hardware interface.
* Fixed dingo control param
* Added DD150
* Fixed name
* Removed comments
* Reduced speed on turbo dd100
* Added dd100
* Added fixes to control and localization
* Extended timeout to a minute
* Added configuration files for generic robots
* Fixes to control parameters and naming
* Fixed package names and added w200 urdf macro
* Initial Warthog addition
* Contributors: Hilary Luo, Luis Camero, Roni Kreinin, Tony Baltovski

0.1.3 (2023-11-03)
------------------

0.1.2 (2023-10-02)
------------------

0.1.1 (2023-08-25)
------------------

0.1.0 (2023-08-17)
------------------
* Formatting
* Removed joy_teleop namespace, remap topics to that namespace instead
* Contributors: Roni Kreinin

0.0.9 (2023-07-31)
------------------
* Update platform nodes from extra ros parameters
  Flattened default parameter files
* Contributors: Roni Kreinin

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
* Updated localization configs
* J100 use Vyaw for localization
* Updated husky track value
* Wheel slip plugin
  Significantly improved jackal odom in sim
* Contributors: Roni Kreinin

0.0.2 (2023-07-04)
------------------

0.0.1 (2023-06-21)
------------------
* Updated launch writer make writing different object types easier
  Localization parameter fixes
  Updated gazebo wheel friction
* Added namespacing support
* Added clearpath_generator_common
  Moved clearpath_platform to clearpath_common
  Fixed use_sim_time parameter issue with ekf_node
* Use generated configs for control, localization, teleop
* use_sim_time support
  Added lidar gazebo plugins
* Fixed dependencies
* Moved description generator to clearpath_generators
  Added accessory urdf's
  Use launch arg for choosing controller
* Moved IMU filter to platform launch
  Moved localization into a separate launch file
  Updated decoration urdfs
  Added structure urdf
* Remapped topics to match API
* Corrected imu_filter_node topics and parameter node name
  Use joy_linux
* Bishop sensors/mounts
* Added realsense description
* [clearpath_control] Renamed robot_model to platform_model.
* control launch fixes
  Added ark enclosure for j100 top_plate
* Move clearpath_description to clearpath_platform_description and switched robot names to robot model number.
* [clearpath_control] Switched to using model number.
* [clearpath_control] Changed depends to exec_depends.
* [clearpath_control] Updated platform names to model.
* Select launch configuration without launch context
* Initial commit of clearpath_control.
* Contributors: Roni Kreinin, Tony Baltovski
