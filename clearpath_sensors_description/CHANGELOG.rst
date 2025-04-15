^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_sensors_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.1 (2025-04-15)
------------------

1.3.0 (2025-04-15)
------------------

1.2.0 (2025-03-13)
------------------
* Feature Humble Ouster (`#161 <https://github.com/clearpathrobotics/clearpath_common/issues/161>`_)
  * Add OusteOS1 description
  * Custom OusterOS1 generator
  * Ouster use custom description generator
* Contributors: Luis Camero

1.1.1 (2025-01-16)
------------------

1.1.0 (2025-01-15)
------------------

1.0.0 (2024-11-25)
------------------
* Add support for Axis cameras (`#113 <https://github.com/clearpathrobotics/clearpath_common/issues/113>`_)
  * Add axis camera URDFs & meshes
  * Fix the path for the meshes
  * Add the AxisCameraDescription class
  * Remove the axis_camera files, add a dependency on axis_description. Add a new meta-macro that uses the camera type
  * Use the device_type to set the model for the new description macro
  * Add the update_rate to the URDF
  * Add default topic for the URDF
  * Remove the update_rate parameter; it's not supported by axis_camera
  * Add the update_rate parameter back to the meta macro, but don't pass it
  * Add the axis camera URDFs & STLs from axis_description, use the local copies instead of having an external dependency
  * Add RGBA values for the "black" material, rename it to avoid conflicting with any other material definitions
* Contributors: Chris Iverach-Brereton

0.3.4 (2024-10-08)
------------------
* Fixed OakD URDF.
* Contributors: Luis Camero

0.3.3 (2024-10-04)
------------------
* Add OAKD
* Added phidget spatial URDF
* Contributors: Luis Camero

0.3.2 (2024-09-29)
------------------

0.3.1 (2024-09-23)
------------------

0.3.0 (2024-09-19)
------------------
* Changes.
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
* 0.2.8
* Changes.
* 0.2.7
* Changes.
* 0.2.6
* Changes.
* 0.2.5
* Changes.
* 0.2.4
* Changes.
* 0.2.3
* Changes.
* 0.2.2
* Changes.xx
* 0.2.1
* Changes.
* Contributors: Tony Baltovski, luis-camero

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
* Contributors: Tony Baltovski, luis-camero

0.2.11 (2024-08-08)
-------------------
* Modifies platform param to add GQ7 IMU data to ekf_localization and adds GQ7 URDF
* Contributors: robbiefish

0.2.10 (2024-07-25)
-------------------
* Realsense adds optical links when in simulator
* Updated topic names to match gazebo message types
* Added topic to gazebo plugins
* Contributors: Luis Camero

0.2.9 (2024-05-28)
------------------
* Added Zed URDF
* Contributors: Luis Camero

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
* Updated material on flir
* Adds Blackfly camera to sensor description (`#33 <https://github.com/clearpathrobotics/clearpath_common/issues/33>`_)
  * Adds Blackfly camera to sensor description
  ---------
  Co-authored-by: fazzrazz <danielduranrojas@gmail.com>
* Added  to materials
* Contributors: Hilary Luo, Luis Camero

0.1.3 (2023-11-03)
------------------

0.1.2 (2023-10-02)
------------------
* Adds Blackfly camera to sensor description (`#33 <https://github.com/clearpathrobotics/clearpath_common/issues/33>`_)
  * Adds Blackfly camera to sensor description
  ---------
  Co-authored-by: fazzrazz <danielduranrojas@gmail.com>
* Contributors: Hilary Luo

0.1.1 (2023-08-25)
------------------
* Added Ignition frame names to simulate the real robot
* Contributors: Hilary Luo

0.1.0 (2023-08-17)
------------------
* Renamed UST10 to UST
  Added parameter node list
* Added UM6/7
* Contributors: Roni Kreinin

0.0.9 (2023-07-31)
------------------
* Added Garmin 18x, Novatel smart 6 and 7
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

0.0.2 (2023-07-04)
------------------

0.0.1 (2023-06-21)
------------------
* Added GPS
  Added realsense gazebo parameters
* Added gazebo IMU plugin
* use_sim_time support
  Added lidar gazebo plugins
* Sim fixes
* Bishop sensors/mounts
* Added microstrain
* Added velodyne
* Added realsense description
* Updated sensor naming
* Sensor descriptions
* Standard urdf and yaml file name and path
  Fixed spacing in urdfs
* Description classes
* PACS mounts
  Common PACS Riser
  Hokuyo and novatel description fixes
* [clearpath_sensors_description] Moved Novatel and Hokuyo into sensors from J100.
* Added clearpath_sensors_description.
* Contributors: Roni Kreinin, Tony Baltovski
