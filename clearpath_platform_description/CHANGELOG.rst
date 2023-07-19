^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_platform_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.7 (2023-07-19)
------------------
* Renamed URDF and meshes directories
* Contributors: Luis Camero

0.0.6 (2023-07-13)
------------------

0.0.5 (2023-07-12)
------------------
* [clearpath_platform_description] Fixed unused dependency in CMakeLists.txt.
* Contributors: Tony Baltovski

0.0.4 (2023-07-07)
------------------

0.0.3 (2023-07-05)
------------------
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
* Increased J100 navsat update rate to 10hz
* Jackal sim support
* Added GPS
  Added realsense gazebo parameters
* Added gazebo IMU plugin
* use_sim_time support
  Added lidar gazebo plugins
* Sim fixes
* Fixed dependencies
* Moved description generator to clearpath_generators
  Added accessory urdf's
  Use launch arg for choosing controller
* [clearpath_platform_description] Made the serial_port an arg for the a200 and reduced polling timeout.
* Moved IMU filter to platform launch
  Moved localization into a separate launch file
  Updated decoration urdfs
  Added structure urdf
* Remapped topics to match API
* Bishop sensors/mounts
* Added velodyne
* [clearpath_platform_description] Fixed hardware plugin for A200.
* control launch fixes
  Added ark enclosure for j100 top_plate
* J100 support
* Standard urdf and yaml file name and path
  Fixed spacing in urdfs
* Description classes
* PACS mounts
  Common PACS Riser
  Hokuyo and novatel description fixes
* Initial commit with platform, decoration and mounts generating
* [clearpath_platform_description] Fixed mesh paths.
* [clearpath_sensors_description] Moved Novatel and Hokuyo into sensors from J100.
* [clearpath_platform_description] Renamed all dashes to underscores.
* [clearpath_platform_description] Fixed incorrect path.
* Move clearpath_description to clearpath_platform_description and switched robot names to robot model number.
* Contributors: Roni Kreinin, Tony Baltovski
