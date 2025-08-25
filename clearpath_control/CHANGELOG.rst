^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Re-enable publishing controller input to joy_teleop/cmd_vel (`#232 <https://github.com/clearpathrobotics/clearpath_common/issues/232>`_)
* Fix: W200 Diff Drive Parameters (`#228 <https://github.com/clearpathrobotics/clearpath_common/issues/228>`_)
  * Disable using position for odom
  * Lower acceleration to reasonable values
* Feature/lynx 1.0.0 (`#226 <https://github.com/clearpathrobotics/clearpath_common/issues/226>`_)
  * Added deceleration limits to A300
  * Update to A300 angular parameters
  * Reduced angular velocity defaults for controllers
* Drop A200 controller manager update rate (`#224 <https://github.com/clearpathrobotics/clearpath_common/issues/224>`_)
  the update rate is limited by the mcu capabilities
* Contributors: Chris Iverach-Brereton, Hilary Luo, Roni Kreinin, luis-camero

2.5.1 (2025-06-17)
------------------

2.5.0 (2025-05-29)
------------------
* Fix the velocity limits for Dingo-O using Logitech controllers (`#207 <https://github.com/clearpathrobotics/clearpath_common/issues/207>`_)
* Increase A300 teleop normal speed to 0.4, reduce turbo speed to 1.5 (`#206 <https://github.com/clearpathrobotics/clearpath_common/issues/206>`_)
* Contributors: Chris Iverach-Brereton

2.3.2 (2025-04-30)
------------------

2.3.1 (2025-04-16)
------------------

2.3.0 (2025-04-11)
------------------
* [clearpath_control] Added remapping for upstream mecanum controller. (`#199 <https://github.com/clearpathrobotics/clearpath_common/issues/199>`_)
* Re-add reference remapping (`#198 <https://github.com/clearpathrobotics/clearpath_common/issues/198>`_)
* added cmd_vel_out to dingo Ds
* remove duplicate remap
* Jazzy dingo and ridgeback fixes (`#179 <https://github.com/clearpathrobotics/clearpath_common/issues/179>`_)
  * Updated mecanum controller type
  * Changed puma control frequency to 20hz
  * Added mecanum package dep
* initial changes for adding the linited velocity output
* Ensure the right horizontal axis is used for yaw control, left horizontal axis for linear-y (`#178 <https://github.com/clearpathrobotics/clearpath_common/issues/178>`_)
* Contributors: Chris Iverach-Brereton, José Mastrangelo, Roni Kreinin, Tony Baltovski, luis-camero

2.2.2 (2025-04-09)
------------------

2.2.1 (2025-04-07)
------------------

2.2.0 (2025-03-11)
------------------

2.1.0 (2025-01-31)
------------------
* Reduced angular velocity limits from joysticks (`#160 <https://github.com/clearpathrobotics/clearpath_common/issues/160>`_)
* Feature/ekf diagnostics (`#158 <https://github.com/clearpathrobotics/clearpath_common/issues/158>`_)
* Diff drive parameter updates for improved odometry (`#155 <https://github.com/clearpathrobotics/clearpath_common/issues/155>`_)
* Contributors: Hilary Luo, Roni Kreinin

2.0.3 (2025-01-21)
------------------
* Bypass bluetooth link quality check (`#151 <https://github.com/clearpathrobotics/clearpath_common/issues/151>`_)
  * Comment-out the bluetooth cutoff node, the mux, and the remap from the joy_teleop to bypass the link quality check
  * Add missing newlines to joy teleop config files
  * Change the link quality check to be exclusive instead of inclusive
* Contributors: Chris Iverach-Brereton

2.0.2 (2025-01-20)
------------------

2.0.1 (2025-01-17)
------------------

2.0.0 (2025-01-16)
------------------
* Update diff_drive controller settings (`#137 <https://github.com/clearpathrobotics/clearpath_common/issues/137>`_)
  * Update diff_drive controller settings
  * Update all diff_drive settings
* Improve joy telep support (`#131 <https://github.com/clearpathrobotics/clearpath_common/issues/131>`_)
  * Update description, add bluez dependency
  * Add the quality_cutoff parameter to all of the PS4 configuration files
  * Update the launch file to start the new cutoff node and additional twist_mux
  * Add udev rules for various controllers
  * Add dependency on the new bt cutoff package
  * Add xbox controller parameters for all platforms
  * Fix the PS4/5 axis ordering; the left stick shows up as axis 3/4, with the l/r analogue triggers being 2 & 5. Update omni control configurations accordingly
  * Add PS5 udev rules, config files
  * Add a timeout to the quality lock so we lock out the controller if the quality-checker node crashes
  * Set the respawn flag for the BT cutoff node and the joy_linux node
  * Add additional logging when blocking for services & IPC. Since we've added a timeout to the lock, publish fake quality data when using wired controllers. Log when this happens
* Add `enable_ekf` launch parameter to platform -> localization launch files. Disable the EKF node if enable_ekf is false. (`#133 <https://github.com/clearpathrobotics/clearpath_common/issues/133>`_)
* Fix sensor depends (`#129 <https://github.com/clearpathrobotics/clearpath_common/issues/129>`_)
  * Remove the package initializations that depend on robot packages
  * Add a copy of the imu_filter parameters from clearpath_sensors to clearpath_control. Change the default IMU filter config path to point to this file. Remove more unneeded initializations of clearpath_robot packages
* A300 VCAN (`#130 <https://github.com/clearpathrobotics/clearpath_common/issues/130>`_)
  * A300 vcan1
  * Set vcan0 to be default can interface for lynx control
  * Fix to prevent including the same package multiple times
  * Added filename argument to LaunchFile
  * Linting
* A300 (`#118 <https://github.com/clearpathrobotics/clearpath_common/issues/118>`_)
  * Add A300 meshes
  * Move A300 meshes
  * Add A300 URDF
  * Added A300 control configuration files
  * Remove unstamped parameter, deprecated
  * Use clearpath_hardware_interface LynxHardware
* Fix controller names and kinematics (`#109 <https://github.com/clearpathrobotics/clearpath_common/issues/109>`_)
* Update simulation support for Jazzy (`#117 <https://github.com/clearpathrobotics/clearpath_common/issues/117>`_)
  * Rename gazebo plugins to use new gz nomenclature instead of ign/ignition. Use stamped velocity messages.
  * Restructure the twist_mux yaml file to be more legible, remove the parameters that are overwritten by the launch file anyway
  * Put use_stamped back just for the sake of being explicit. Add use_stamped directly to the launch file
  * Fix the tests to catch unsupported platforms & accessories
* Rename ign\_ -> gz\_ for gazebo dependencies, comment-out missing jazzy dependencies (for now)
* Removed unstamped remappings
* Set minimum version of teleop_twist_joy to 2.6.1 (the lowest version that supports stamped messages)
* Pass use_stamped to twist_mux
* Remap twist_mux output to stamped cmd_vel
* Pass publish_stamped_twist to joystick teleop node
* Remap stamped cmd_vel controller topic
* Contributors: Chris Iverach-Brereton, Christoph Fröhlich, Luis Camero, Roni Kreinin, Tony Baltovski

1.0.0 (2024-11-25)
------------------
* Fix controller names and kinematics (`#109 <https://github.com/clearpathrobotics/clearpath_common/issues/109>`_)
* Contributors: luis-camero

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
* Changes.
* Add dependency clearpath_mecanum_drive_controller
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
* Add Y to controllers
* Updated wheel separation
* Keep both joint definitions
* Control updates
* Changed default canbus device to vcan0
* Added Puma control configuration
* 0.2.8
* Changes.
* 0.2.7
* Changes.
* Control adds manipulators if simulation
* Modifications to allow arms to function
* 0.2.6
* Changes.
* Disable tf_frame_prefix_enable
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
* Added do150 control
* Dingo O mecanum wheels
* Fixed rocker and small covariance
* Fixed wheel radius parameter
* Initial add do100
* Contributors: Luis Camero, Tony Baltovski, luis-camero

* Add dependency clearpath_mecanum_drive_controller
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
* Add Y to controllers
* Updated wheel separation
* Keep both joint definitions
* Control updates
* Changed default canbus device to vcan0
* Added Puma control configuration
* Control adds manipulators if simulation
* Modifications to allow arms to function
* Added do150 control
* Dingo O mecanum wheels
* Fixed rocker and small covariance
* Fixed wheel radius parameter
* Initial add do100
* Contributors: Tony Baltovski, luis-camero, Steve Macenski, Hilary Luo, robbiefish

0.2.11 (2024-08-08)
-------------------
* Set spawner as super client
* Contributors: Luis Camero

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
