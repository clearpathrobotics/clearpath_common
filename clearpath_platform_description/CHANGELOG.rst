^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_platform_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.9 (2024-05-28)
------------------

0.2.8 (2024-05-14)
------------------

0.2.7 (2024-04-08)
------------------
* All Warthog attachments default to 0
* Aligned attachment links
* Contributors: Luis Camero

0.2.6 (2024-01-18)
------------------

0.2.5 (2024-01-15)
------------------
* switch finding meshes to use the package:// command
* Contributors: Hilary Luo

0.2.4 (2024-01-11)
------------------

0.2.3 (2024-01-08)
------------------

0.2.2 (2024-01-04)
------------------

0.2.1 (2023-12-21)
------------------
* PACS top plate was not centred with generated frames
* Contributors: Hilary Luo

0.2.0 (2023-12-08)
------------------
* Added W200 Hardware interface.
* Use path substitution
* Removed testing visual for track virtual wheel
* Added wheel parameters to all robot
* Color and wheel are now parameters
* Updated collision model
* Added w200 tracks
* First pass
* Removed serial from URDF
* Accurate mounts shift
* Added DD150
* Re-add caster
* Added default mount
* Removed friction on rear_caster
* Added dd100
* Removed transmission and cleaned up URDF
* Removed erroneaous change
* Removed print
* Updated gazebo simulation
* Changed default parent to default_mount
* Set color appropriately
* Arm Mount at base
* Mounts re-numbering
* Formatted W200 attachment URDF
* Changed top_chassis_link to default_mount
* Changed mid_mount to default_mount
* Renamed sensor_arch namespace
* Changed sensor arch parent
* Added mid_mount
* Origin to bumper
* Added  to materials
* Attachment URDF match changes
* Add gazebo controller to URDF without macro
* Moved gazebo controller to common
* Fixes to control parameters and naming
* Base diff drive hardware and hardware interface class
  J100 and W200 inherit from diff drive
  Moved each platform into its own folder
* Fixed package names and added w200 urdf macro
* Initial Warthog addition
* Contributors: Hilary Luo, Luis Camero, Roni Kreinin, Tony Baltovski

0.1.3 (2023-11-03)
------------------
* [clearpath_platform_description] Removed un-used ros2_control params.
* Contributors: Tony Baltovski

0.1.2 (2023-10-02)
------------------

0.1.1 (2023-08-25)
------------------
* Update gps and imu names to fix generated topic
* Added Ignition frame names to simulate the real robot
* Contributors: Hilary Luo

0.1.0 (2023-08-17)
------------------
* Added fenders for J100
* Contributors: Roni Kreinin

0.0.9 (2023-07-31)
------------------

0.0.8 (2023-07-24)
------------------
* Updated J100 imu and navsat links
* Contributors: Roni Kreinin

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
