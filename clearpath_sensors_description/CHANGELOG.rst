^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_sensors_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

2.3.2 (2025-04-30)
------------------

2.3.1 (2025-04-16)
------------------

2.3.0 (2025-04-11)
------------------
* Added Wiferion Charger (`#197 <https://github.com/clearpathrobotics/clearpath_common/issues/197>`_)
* Added support for INS sensors + Fixposition XVN (`#184 <https://github.com/clearpathrobotics/clearpath_common/issues/184>`_)
* Contributors: Chris Iverach-Brereton, Luis Camero

2.2.2 (2025-04-09)
------------------

2.2.1 (2025-04-07)
------------------

2.2.0 (2025-03-11)
------------------
* Feature Jazzy Ouster (`#170 <https://github.com/clearpathrobotics/clearpath_common/issues/170>`_)
  * Add OusteOS1 description
  * Custom OusterOS1 generator
  * Ouster use custom description generator
  * Use appropriate generator for Ouster
* Add D455, D456 support (`#176 <https://github.com/clearpathrobotics/clearpath_common/issues/176>`_)
  * Add URDFs for all supported RealSense cameras
  * Pass the `device_type` as the model parameter to the master URDF, move specific types into a sub-directory and include them as necessary
* Add support for OAK-D Pro W PoE (`#174 <https://github.com/clearpathrobotics/clearpath_common/issues/174>`_)
  * Add the model for the OAK-D Pro W; it's identical to the OAK-D Pro. Add model type support to the OAK-D description
* Add URDF, STL files for the Seyond Robin W lidar (`#169 <https://github.com/clearpathrobotics/clearpath_common/issues/169>`_)
  * Add URDF, STL files for the Seyond Robin W lidar
* Contributors: Chris Iverach-Brereton, luis-camero

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
* Fix test errors (`#132 <https://github.com/clearpathrobotics/clearpath_common/issues/132>`_)
  * Add continue clause to the unsupported device/platform exceptions so we don't try any further tests with them
  * Fix URDF parameters so the source CI passes with the axis cameras
* Add plugins to get the PTZ joint states out of gazebo and into ROS (`#126 <https://github.com/clearpathrobotics/clearpath_common/issues/126>`_)
* Add PTZ sim support (`#125 <https://github.com/clearpathrobotics/clearpath_common/issues/125>`_)
  * Now that axis_camera is released via OSRF, depend on the official package, remove duplicate meshes
  * Rename Gazebo plugins for Jazzy compatibility
  * Modify Axis camera URDFs to using the axis_camera meshes. This lets us control the gazebo topics. Fix the GZ topic names. Camera data is now visible in the simulation
  * Add joint controllers for the pan & tilt actuators. This provides velocity control over the simulated camera
* Update simulation support for Jazzy (`#117 <https://github.com/clearpathrobotics/clearpath_common/issues/117>`_)
  * Rename gazebo plugins to use new gz nomenclature instead of ign/ignition. Use stamped velocity messages.
  * Restructure the twist_mux yaml file to be more legible, remove the parameters that are overwritten by the launch file anyway
  * Put use_stamped back just for the sake of being explicit. Add use_stamped directly to the launch file
  * Fix the tests to catch unsupported platforms & accessories
* Contributors: Chris Iverach-Brereton, Luis Camero, Tony Baltovski, luis-camero

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
