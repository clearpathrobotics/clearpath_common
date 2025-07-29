^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_platform_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.6.4 (2025-07-29)
------------------

2.6.3 (2025-07-18)
------------------
* Fix: Disable Platform Controllers in Manipulator Controllers (`#239 <https://github.com/clearpathrobotics/clearpath_common/issues/239>`_)
  Add flag to disable platform controllers for manipulation controller manager
* Contributors: luis-camero

2.6.2 (2025-07-14)
------------------

2.6.1 (2025-07-07)
------------------

2.6.0 (2025-07-04)
------------------
* Add the third antenna (bluetooth) to the top of the AMP frame (`#231 <https://github.com/clearpathrobotics/clearpath_common/issues/231>`_)
* Add A300 AMP attachments (`#200 <https://github.com/clearpathrobotics/clearpath_common/issues/200>`_)
  * Add meshes, URDFs for the A300 AMP and AMP Observer attachments
  * Add spotlight attachment with light plugin for simulation
  * Add Seyond description with lower-than-reality resolution & correct FoV for simulation. Fix math for calculating the number of samples
  * Add GPS plugins for the Fixposition description
* Contributors: Chris Iverach-Brereton

2.5.1 (2025-06-17)
------------------

2.5.0 (2025-05-29)
------------------
* Fix: PACS Limits (`#215 <https://github.com/clearpathrobotics/clearpath_common/issues/215>`_)
* Contributors: luis-camero

2.3.2 (2025-04-30)
------------------

2.3.1 (2025-04-16)
------------------
* [clearpath_platform_description] Updated Ridgeback R100 raiser height to 0.07m from 0.055m. (`#201 <https://github.com/clearpathrobotics/clearpath_common/issues/201>`_)
* Contributors: Tony Baltovski

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

2.0.2 (2025-01-20)
------------------

2.0.1 (2025-01-17)
------------------

2.0.0 (2025-01-16)
------------------
* Add the A200 Observer backpack attachment (`#122 <https://github.com/clearpathrobotics/clearpath_common/issues/122>`_)
  * Add the A200 Observer backpack attachment
* [clearpath_platform_description] Fixed Ridgeback R100 rear light colour. (`#127 <https://github.com/clearpathrobotics/clearpath_common/issues/127>`_)
* Add PTZ sim support (`#125 <https://github.com/clearpathrobotics/clearpath_common/issues/125>`_)
  * Now that axis_camera is released via OSRF, depend on the official package, remove duplicate meshes
  * Rename Gazebo plugins for Jazzy compatibility
  * Modify Axis camera URDFs to using the axis_camera meshes. This lets us control the gazebo topics. Fix the GZ topic names. Camera data is now visible in the simulation
  * Add joint controllers for the pan & tilt actuators. This provides velocity control over the simulated camera
* Add a placeholer URDF for the AMP mount, update meshes (`#123 <https://github.com/clearpathrobotics/clearpath_common/issues/123>`_)
  * Add a placeholer URDF for the AMP mount; STL & final dimensions to come at a later date
  * Default to treaded wheels, flip all the wheel models so the treads visually go in the correct direction
  * Update the top plate, chassis, livery, smooth wheel, and status light meshes. Closes CPE87-2102
  * Catch unsupported platforms/accessories in vcan generation tests
* A300 (`#118 <https://github.com/clearpathrobotics/clearpath_common/issues/118>`_)
  * Add A300 meshes
  * Move A300 meshes
  * Add A300 URDF
  * Added A300 control configuration files
  * Remove unstamped parameter, deprecated
  * Use clearpath_hardware_interface LynxHardware
  ---------
  Co-authored-by: Luis Camero <lcamero@clearpathrobotics.com>
* Add collision tags for MoveIt to avoid these objects (`#108 <https://github.com/clearpathrobotics/clearpath_common/issues/108>`_)
* Remove all references to clearpath_platform
* Add collision tags for MoveIt to avoid these objects (`#108 <https://github.com/clearpathrobotics/clearpath_common/issues/108>`_)
* Update simulation support for Jazzy (`#117 <https://github.com/clearpathrobotics/clearpath_common/issues/117>`_)
  * Rename gazebo plugins to use new gz nomenclature instead of ign/ignition. Use stamped velocity messages.
  * Restructure the twist_mux yaml file to be more legible, remove the parameters that are overwritten by the launch file anyway
  * Put use_stamped back just for the sake of being explicit. Add use_stamped directly to the launch file
  * Fix the tests to catch unsupported platforms & accessories
* Rename ign\_ -> gz\_ for gazebo dependencies, comment-out missing jazzy dependencies (for now)
* Contributors: Chris Iverach-Brereton, Luis Camero, Roni Kreinin, Tony Baltovski, luis-camero

1.0.0 (2024-11-25)
------------------
* [clearpath_platform_description] Fixed gazebo_controllers param file. (`#107 <https://github.com/clearpathrobotics/clearpath_common/issues/107>`_)
* Remove all references to clearpath_platform
* Add collision tags for MoveIt to avoid these objects (`#108 <https://github.com/clearpathrobotics/clearpath_common/issues/108>`_)
* [clearpath_platform_description] Fixed gazebo_controllers param file. (`#107 <https://github.com/clearpathrobotics/clearpath_common/issues/107>`_)
* Contributors: Luis Camero, Tony Baltovski, luis-camero

0.3.4 (2024-10-08)
------------------

0.3.3 (2024-10-04)
------------------

0.3.2 (2024-09-29)
------------------
* Fix outstanding merge conflict
* Contributors: Luis Camero

0.3.1 (2024-09-23)
------------------

0.3.0 (2024-09-19)
------------------
* Changes.
* Add PACS mounting locations to Dingo top plate
* Standard mesh names and height parameter for tower shoulder
* R100 attachment rework
* Add dingo top plate URDF
* 0.3.0 Release Candidate with Main Changes (`#81 <https://github.com/clearpathrobotics/clearpath_common/issues/81>`_)
  * Added tests
  * Added action to build from release and source
  * Generator linting erros
  * Customization linting errors
  * Linting
  * Fix: Remove IP address from discovery server launch so it listens on all NICs
  * Changes.
  * 0.2.8
  * Add sysctl config file that changes ipfrag settings to support receiving large messages
  * Added Zed URDF
  * Added Zed to description generator
  * Modified common parameter generation to always flatten
  * Changes.
  * 0.2.9
  * Missing important remapping to mirror hardware topics
  * Added topic to gazebo plugins
  * Updated topic names to match gazebo message types
  * Topics of simulated onboard sensors
  * Realsense adds optical links when in simulator
  * Changes.
  * 0.2.10
  * Modifies platform param to add GQ7 IMU data to ekf_localization and adds GQ7 URDF
  * Fixes styling issues
  * Set spawner as super client
  * Changes.
  * 0.2.11
  * Removed duplicate class
  * Use ROS1 covariance values
  * Updated renamed macanum drive controller
  * Enable gazebo friction plugin on DingoO
  ---------
  Co-authored-by: Hilary Luo <hluo@clearpathrobotics.com>
  Co-authored-by: Tony Baltovski <tbaltovski@clearpathrobotics.com>
  Co-authored-by: Steve Macenski <stevenmacenski@gmail.com>
  Co-authored-by: robbiefish <rob.fisher@hbkworld.com>
* Set PumaHardware in R100
* URDF Mecanum Updates
* Switched to puma_hardware
* DingoD 1.5 Puma Hardware
* 0.2.8
* Changes.
* 0.2.7
* Changes.
* All Warthog attachments default to 0
* Aligned attachment links
* Added argument to prevent platform hardware from loading
* Define inertial box xacro in the arm plate urdf to be used by other platforms
* Added argument to toggle manipulator controllers
* 0.2.6
* Changes.
* 0.2.5
* Changes.
* switch finding meshes to use the package:// command
* 0.2.4
* Changes.
* 0.2.3
* Changes.
* 0.2.2
* Changes.xx
* 0.2.1
* Changes.
* PACS top plate was not centred with generated frames
* Added Ridgeback attachments
* Added do150 as copy of do100
* R100 mecanum
* Only load mecanum wheels in simulation
* Removed wheel slip plugin
* Dingo O mecanum wheels
* Fixed rocker and small covariance
* Initial Ridgeback
* Initial add do100
* Contributors: Hilary Luo, Luis Camero, Tony Baltovski, luis-camero

* Add PACS mounting locations to Dingo top plate
* Standard mesh names and height parameter for tower shoulder
* R100 attachment rework
* Add dingo top plate URDF
* Added tests
* Added action to build from release and source
* Generator linting erros
* Customization linting errors
* Linting
* Fix: Remove IP address from discovery server launch so it listens on all NICs
* Add sysctl config file that changes ipfrag settings to support receiving large messages
* Added Zed URDF
* Added Zed to description generator
* Modified common parameter generation to always flatten
* Missing important remapping to mirror hardware topics
* Added topic to gazebo plugins
* Updated topic names to match gazebo message types
* Topics of simulated onboard sensors
* Realsense adds optical links when in simulator
* Modifies platform param to add GQ7 IMU data to ekf_localization and adds GQ7 URDF
* Fixes styling issues
* Set spawner as super client
* Removed duplicate class
* Use ROS1 covariance values
* Updated renamed macanum drive controller
* Enable gazebo friction plugin on DingoO
* Contributors: Hilary Luo, Luis Camero, Tony Baltovski, luis-camero

0.2.11 (2024-08-08)
-------------------

0.2.10 (2024-07-25)
-------------------
* Topics of simulated onboard sensors
* Missing important remapping to mirror hardware topics
* Contributors: Luis Camero, Steve Macenski

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
