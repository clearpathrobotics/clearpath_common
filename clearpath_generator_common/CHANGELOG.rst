^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_generator_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.6.4 (2025-07-29)
------------------
* Fix: Update diagnostics rate for Inventus  (`#243 <https://github.com/clearpathrobotics/clearpath_common/issues/243>`_)
* Contributors: luis-camero

2.6.3 (2025-07-18)
------------------

2.6.2 (2025-07-14)
------------------
* Only add sensors to the description if their urdf_enabled flag is true (`#235 <https://github.com/clearpathrobotics/clearpath_common/issues/235>`_)
* Contributors: Chris Iverach-Brereton

2.6.1 (2025-07-07)
------------------
* Forward Fix: Universal Robots Rate (`#233 <https://github.com/clearpathrobotics/clearpath_common/issues/233>`_)
  * Fix/Feature: UR Arm Controller Update Rate (`#225 <https://github.com/clearpathrobotics/clearpath_common/issues/225>`_)
  * Change controller update rate for universal robots to 500
  * Use UniversalRobots update_rate parameters if available
  * Remove Franka
* Contributors: luis-camero

2.6.0 (2025-07-04)
------------------
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
* Use argparse instead of getopt for processing the command-line arguments (`#223 <https://github.com/clearpathrobotics/clearpath_common/issues/223>`_)
* Add A300 AMP attachments (`#200 <https://github.com/clearpathrobotics/clearpath_common/issues/200>`_)
  * Add meshes, URDFs for the A300 AMP and AMP Observer attachments
  * Add spotlight attachment with light plugin for simulation
  * Add Seyond description with lower-than-reality resolution & correct FoV for simulation. Fix math for calculating the number of samples
  * Add GPS plugins for the Fixposition description
* Contributors: Chris Iverach-Brereton, Hilary Luo

2.5.1 (2025-06-17)
------------------
* Fix permissions on the generate_zenoh_router executable (`#221 <https://github.com/clearpathrobotics/clearpath_common/issues/221>`_)
* Contributors: Chris Iverach-Brereton

2.5.0 (2025-05-29)
------------------
* Feature: Foxglove Bridge (`#213 <https://github.com/clearpathrobotics/clearpath_common/issues/213>`_)
  * Add foxglove bridge launch and parameter file
  * Add foxglove bridge parameter to generator
* Move clearpath_diagnostics to clearpath_common (`#211 <https://github.com/clearpathrobotics/clearpath_common/issues/211>`_)
* Add cooling and ekf-node diagnostic settings conditionally (`#209 <https://github.com/clearpathrobotics/clearpath_common/issues/209>`_)
  * Add cooling and ekf-node diagnostic settings conditionally
* Contributors: Hilary Luo, luis-camero

2.3.2 (2025-04-30)
------------------

2.3.1 (2025-04-16)
------------------

2.3.0 (2025-04-11)
------------------
* Feature: Add CAN adapters (`#196 <https://github.com/clearpathrobotics/clearpath_common/issues/196>`_)
* Fast Forward Feature: Add delay to manipulator controller (`#191 <https://github.com/clearpathrobotics/clearpath_common/issues/191>`_)
* Feature: Manipulator URDF Parameters (`#181 <https://github.com/clearpathrobotics/clearpath_common/issues/181>`_) (`#190 <https://github.com/clearpathrobotics/clearpath_common/issues/190>`_)
* Feature: MoveIt Parameters and Enable (`#166 <https://github.com/clearpathrobotics/clearpath_common/issues/166>`_) (`#189 <https://github.com/clearpathrobotics/clearpath_common/issues/189>`_)
* Feature: Manipulator Samples and Poses (`#163 <https://github.com/clearpathrobotics/clearpath_common/issues/163>`_) (`#188 <https://github.com/clearpathrobotics/clearpath_common/issues/188>`_)
* Add exception handling to the generators to print a nicer error message in the case of self-generated Unsupported* exceptions (`#192 <https://github.com/clearpathrobotics/clearpath_common/issues/192>`_)
* Move extras launch into a new service (`#185 <https://github.com/clearpathrobotics/clearpath_common/issues/185>`_)
* Add support for INS sensors + Fixposition XVN (`#184 <https://github.com/clearpathrobotics/clearpath_common/issues/184>`_)
* Add MCU diagnostic category if not A200 (`#183 <https://github.com/clearpathrobotics/clearpath_common/issues/183>`_)
* Contributors: Chris Iverach-Brereton, Hilary Luo, Luis Camero

2.2.2 (2025-04-09)
------------------

2.2.1 (2025-04-07)
------------------
* Fix: Handle empty namespace (`#187 <https://github.com/clearpathrobotics/clearpath_common/issues/187>`_)
  * Handle empty namespace
  * Resolve formatting issues
* Contributors: luis-camero

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
* Feature/generate aggregator config (`#173 <https://github.com/clearpathrobotics/clearpath_common/issues/173>`_)
  * Generate diagnostic aggregator sensor params based on robot.yaml
  * Remove redundant read of the default param file
* Add URDF, STL files for the Seyond Robin W lidar (`#169 <https://github.com/clearpathrobotics/clearpath_common/issues/169>`_)
  * Add URDF, STL files for the Seyond Robin W lidar
* Contributors: Chris Iverach-Brereton, Hilary Luo, luis-camero

2.1.0 (2025-01-31)
------------------
* Jazzy Phidgets IMU Filter (`#159 <https://github.com/clearpathrobotics/clearpath_common/issues/159>`_)
  * Add imu data topic to Phidget diagnostics
  * Check for IMU index
* Feature/ekf diagnostics (`#158 <https://github.com/clearpathrobotics/clearpath_common/issues/158>`_)
  * Enable ekf node diagnostics
  * Fix CI
* Ewellix Lift (`#136 <https://github.com/clearpathrobotics/clearpath_common/issues/136>`_) (`#153 <https://github.com/clearpathrobotics/clearpath_common/issues/153>`_)
  Ewellix Lift
  - Add lift description
  - Add lifts to generators
* Feature/diagnostics (`#156 <https://github.com/clearpathrobotics/clearpath_common/issues/156>`_)
  * Generate params for new diagnostics package
  * Link sim rates to config objects or match number with todo note
  * Added firmware version check
  * Disable MCU diagnostics for A200
* Replace .h with .hpp per compilation warning (`#154 <https://github.com/clearpathrobotics/clearpath_common/issues/154>`_)
* Contributors: Chris Iverach-Brereton, Hilary Luo, luis-camero

2.0.3 (2025-01-21)
------------------

2.0.2 (2025-01-20)
------------------

2.0.1 (2025-01-17)
------------------

2.0.0 (2025-01-16)
------------------
* clearpath_generator_common/clearpath_generator_common/zenoh_router/generator.py (`#146 <https://github.com/clearpathrobotics/clearpath_common/issues/146>`_)
* Use the .profile field for the Zenoh router config; don't add a new variable just for that (`#143 <https://github.com/clearpathrobotics/clearpath_common/issues/143>`_)
* Catch the new unsupported platform/accessory exceptions raised by the generator so the tests pass, use the envar when setting the default value of ROS_DISTRO (`#104 <https://github.com/clearpathrobotics/clearpath_common/issues/104>`_)
  Co-authored-by: Tony Baltovski <tbaltovski@clearpathrobotics.com>
* Add Zenoh support (`#138 <https://github.com/clearpathrobotics/clearpath_common/issues/138>`_)
  * Add generator for zenoh router service script
  * Add the ZENOH_ROUTER_CONFIG_URI envar to the generated bash file, populated with either the default path or the user-specified one as needed
* Add `enable_ekf` launch parameter to platform -> localization launch files. Disable the EKF node if enable_ekf is false. (`#133 <https://github.com/clearpathrobotics/clearpath_common/issues/133>`_)
* Fix test errors (`#132 <https://github.com/clearpathrobotics/clearpath_common/issues/132>`_)
  * Add continue clause to the unsupported device/platform exceptions so we don't try any further tests with them
  * Fix URDF parameters so the source CI passes with the axis cameras
* Add the A200 Observer backpack attachment (`#122 <https://github.com/clearpathrobotics/clearpath_common/issues/122>`_)
  * Add the A200 Observer backpack attachment
* Fix sensor depends (`#129 <https://github.com/clearpathrobotics/clearpath_common/issues/129>`_)
  * Remove the package initializations that depend on robot packages
  * Add a copy of the imu_filter parameters from clearpath_sensors to clearpath_control. Change the default IMU filter config path to point to this file. Remove more unneeded initializations of clearpath_robot packages
* A300 VCAN (`#130 <https://github.com/clearpathrobotics/clearpath_common/issues/130>`_)
  * A300 vcan1
  * Set vcan0 to be default can interface for lynx control
  * Fix to prevent including the same package multiple times
  * Added filename argument to LaunchFile
  * Linting
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
* Update simulation support for Jazzy (`#117 <https://github.com/clearpathrobotics/clearpath_common/issues/117>`_)
  * Rename gazebo plugins to use new gz nomenclature instead of ign/ignition. Use stamped velocity messages.
  * Restructure the twist_mux yaml file to be more legible, remove the parameters that are overwritten by the launch file anyway
  * Put use_stamped back just for the sake of being explicit. Add use_stamped directly to the launch file
  * Fix the tests to catch unsupported platforms & accessories
* Fix the discovery server to use the new path too
* Create `ros` module to contain default distro & default setup.bash path to make updating distributions easier
* Contributors: Chris Iverach-Brereton, Luis Camero, Roni Kreinin, Tony Baltovski, luis-camero

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
