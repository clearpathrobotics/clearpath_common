^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_generator_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.2 (2025-04-15)
------------------
* Skip failed tests
* Contributors: Luis Camero

1.3.1 (2025-04-15)
------------------
* Skip stereolabs test
* Contributors: Luis Camero

1.3.0 (2025-04-15)
------------------
* Feature: Add delay to manipulator controller (`#182 <https://github.com/clearpathrobotics/clearpath_common/issues/182>`_)
* Feature: Manipulator URDF Parameters (`#181 <https://github.com/clearpathrobotics/clearpath_common/issues/181>`_)
* Contributors: Luis Camero

1.2.0 (2025-03-13)
------------------
* Fix: Only use IMU for angular velocity (`#175 <https://github.com/clearpathrobotics/clearpath_common/issues/175>`_)
* Feature Franka (`#171 <https://github.com/clearpathrobotics/clearpath_common/issues/171>`_)
  * Initial franka addition
  * Create joint instead of using connected_to parameter
  * Custom entry for generating Franka param
  * Franka control for multiple types
  * Modify entire Franka arm xacro
  * Add franka gripper
  * Copy and modify franka description
  * Add cutout for Franka arm_id
  * Add dependency for franka_description
* Feature Humble Ouster (`#161 <https://github.com/clearpathrobotics/clearpath_common/issues/161>`_)
  * Add OusteOS1 description
  * Custom OusterOS1 generator
  * Ouster use custom description generator
* Feature: MoveIt Parameters and Enable (`#166 <https://github.com/clearpathrobotics/clearpath_common/issues/166>`_)
* Feature: Manipulator Samples and Poses (`#163 <https://github.com/clearpathrobotics/clearpath_common/issues/163>`_)
  * Add group_state macros to arm SRDF
  * Add group_state macros to grippers SRDF
  * Generate pose macros in URDF
* Feature: Link Material (`#162 <https://github.com/clearpathrobotics/clearpath_common/issues/162>`_)
  * Add material to link generator
  * Add material xacro macro
  * Catch exception for frame without material
* Backport Fix: Sensor depends (`#129 <https://github.com/clearpathrobotics/clearpath_common/issues/129>`_) (`#167 <https://github.com/clearpathrobotics/clearpath_common/issues/167>`_)
  * Remove the package initializations that depend on robot packages
  * Add a copy of the imu_filter parameters from clearpath_sensors to clearpath_control. Change the default IMU filter config path to point to this file. Remove more unneeded initializations of clearpath_robot packages
  Co-authored-by: Chris Iverach-Brereton <59611394+civerachb-cpr@users.noreply.github.com>
* Fix: Change dependency on SRDF plugins (`#165 <https://github.com/clearpathrobotics/clearpath_common/issues/165>`_)
* Contributors: luis-camero

1.1.1 (2025-01-16)
------------------

1.1.0 (2025-01-15)
------------------
* Ewellix Lift (`#136 <https://github.com/clearpathrobotics/clearpath_common/issues/136>`_)
  Ewellix Lift
  -  Remove upper_joint
  - Add moveit jpc
  - Add control for joint position controller
  - Add hardware parameters
  - Add lifts to generators
  - Initial add of Ewellix lift description files
* Add `enable_ekf` launch parameter to platform -> localization launch files. Disable the EKF node if enable_ekf is false. (`#133 <https://github.com/clearpathrobotics/clearpath_common/issues/133>`_) (`#134 <https://github.com/clearpathrobotics/clearpath_common/issues/134>`_)
* Contributors: Chris Iverach-Brereton, luis-camero

1.0.0 (2024-11-25)
------------------
* Added minimum version.
* Remove all references to clearpath_platform
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
* Add UR arm (`#110 <https://github.com/clearpathrobotics/clearpath_common/issues/110>`_)
* VCAN Rework (`#112 <https://github.com/clearpathrobotics/clearpath_common/issues/112>`_)
  * VCan service script generator
  * Use formatted strings to shorten line length and expose variables
* Add test exception for Zed (`#100 <https://github.com/clearpathrobotics/clearpath_common/issues/100>`_)
* Contributors: Chris Iverach-Brereton, Luis Camero, Tony Baltovski, luis-camero

0.3.4 (2024-10-08)
------------------

0.3.3 (2024-10-04)
------------------

0.3.2 (2024-09-29)
------------------

0.3.1 (2024-09-23)
------------------
* Add manipulator dependencies
* Fixed linting issues for manipulator generation
* Contributors: Luis Camero

0.3.0 (2024-09-19)
------------------
* Changes.
* Add meshes and URDF for robotiq 2f 140
* Standard mesh names and height parameter for tower shoulder
* R100 attachment rework
* Add Dingo plate to generator
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
* Fix: Remove IP address from discovery server launch so it listens on all NICs
* 0.2.7
* Changes.
* ARM_MOUNT to ARM_PLATE
* Linting issues
* Use if statement
* Fixed all license headers
* Fixed linting issues of collision updater node
* Pass parameters to Kinova URDF
* Updated generators to deal with grippers as part of arms
* Create control file for manipulator controller manager
* Only add manipulator controllers if simulation
* Added virtual method for manipulator launch generation
* Added semantic description generator
* Added manipulators to parameter generator
* Add manipulators to description generator
* Modifications to allow arms to function
* Added simple package writer to copy package from template
* Check terminal to set ROS_SUPER_CLIENT
* Generate script to start the discovery server
* Updated setup.bash generation for discovery server
* 0.2.6
* Changes.
* 0.2.5
* Changes.
* switch finding meshes to use the package:// command
* 0.2.4
* Changes.
* [clearpath_generator_common] Added package description.
* 0.2.3
* Changes.
* Handle file paths with no directory (files in root directory of the package)
* 0.2.2
* Changes.xx
* Enable extras urdf and meshes to be linked by package (`#53 <https://github.com/clearpathrobotics/clearpath_common/issues/53>`_)
* 0.2.1
* Changes.
* Contributors: Hilary Luo, Luis Camero, Tony Baltovski, luis-camero

* Add meshes and URDF for robotiq 2f 140
* Standard mesh names and height parameter for tower shoulder
* R100 attachment rework
* Add Dingo plate to generator
* Added tests
* Added action to build from release and source
* Generator linting erros
* Customization linting errors
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
* Fixes styling issues
* Modifies platform param to add GQ7 IMU data to ekf_localization and adds GQ7 URDF
* Contributors: robbiefish

0.2.10 (2024-07-25)
-------------------

0.2.9 (2024-05-28)
------------------
* Modified common parameter generation to always flatten
* Added Zed to description generator
* Add sysctl config file that changes ipfrag settings to support receiving large messages
* Linting
* Generator linting erros
* Added tests
* Contributors: Hilary Luo, Luis Camero

0.2.8 (2024-05-14)
------------------
* Fix: Remove IP address from discovery server launch so it listens on all NICs
* Contributors: Hilary Luo

0.2.7 (2024-04-08)
------------------
* ARM_MOUNT to ARM_PLATE
* Added simple package writer to copy package from template
* Check terminal to set ROS_SUPER_CLIENT
* Generate script to start the discovery server
* Updated setup.bash generation for discovery server
* Contributors: Hilary Luo, Luis Camero

0.2.6 (2024-01-18)
------------------

0.2.5 (2024-01-15)
------------------
* switch finding meshes to use the package:// command
* Contributors: Hilary Luo

0.2.4 (2024-01-11)
------------------
* [clearpath_generator_common] Added package description.
* Contributors: Tony Baltovski

0.2.3 (2024-01-08)
------------------
* Handle file paths with no directory (files in root directory of the package)
* Contributors: Hilary Luo

0.2.2 (2024-01-04)
------------------
* Enable extras urdf and meshes to be linked by package (`#53 <https://github.com/clearpathrobotics/clearpath_common/issues/53>`_)
* Contributors: Hilary Luo

0.2.1 (2023-12-21)
------------------

0.2.0 (2023-12-08)
------------------
* Added wheel parameters to all robot
* Wheel is now parameter
* Adds Blackfly camera to sensor description (`#33 <https://github.com/clearpathrobotics/clearpath_common/issues/33>`_)
  * Adds Blackfly camera to sensor description
  ---------
  Co-authored-by: fazzrazz <danielduranrojas@gmail.com>
* Removed print in platform description generator
* Add imu0 to ekf_node for all platforms except A200
* Added W200 attachments to generator
* Platform no longer required
* Added  to materials
* Removed unecessary SimpleDescription
* Attachments not restricted by platform
* Simplified attachment generation
* Removed debug print
* Removed gazebo include from generator
* Read control.yaml directly from clearpath config specified file
* Allow for no macro to be added
* Moved gazebo controller to common
* Added Generic platform
* Contributors: Hilary Luo, Luis Camero, Roni Kreinin

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

0.1.0 (2023-08-17)
------------------
* Removed joy_teleop namespace, remap topics to that namespace instead
* Added fenders for J100
* Renamed UST10 to UST
  Added parameter node list
* Removed disk import
* Added disk and post
  Set default values to model dictionaries
* Inverted and upright sick stand
* Added UM6/7
* Contributors: Roni Kreinin

0.0.9 (2023-07-31)
------------------
* Added Garmin 18x, Novatel smart 6 and 7
* Update platform nodes from extra ros parameters
  Flattened default parameter files
* Contributors: Roni Kreinin

0.0.8 (2023-07-24)
------------------
* Linting
* use_sim_time support
* Description and Bash generator cleanup
* Minor cleanup
* Param generator
* Launch generator cleanup
* Contributors: Roni Kreinin

0.0.7 (2023-07-19)
------------------
* Renamed description to attachments
* Rnamed accessories to links
* Contributors: Luis Camero

0.0.6 (2023-07-13)
------------------
* Merge pull request `#18 <https://github.com/clearpathrobotics/clearpath_common/issues/18>`_ from clearpathrobotics/updated-config
  Updated common generators to match config
* Fixed getters
* Updated common generators to match config
* Contributors: Luis Camero, Roni Kreinin

0.0.5 (2023-07-12)
------------------

0.0.4 (2023-07-07)
------------------

0.0.3 (2023-07-05)
------------------
* Linters
* Updated localization configs
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
* Updated dependencies
* Added clearpath_generator_common
  Moved clearpath_platform to clearpath_common
  Fixed use_sim_time parameter issue with ekf_node
* Contributors: Roni Kreinin
