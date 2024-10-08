^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_platform
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Add headers to Puma hardware
* Updated puma topics
* PumaHardwareInterface
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
* Fixed lighting lib install
* 0.2.3
* Changes.
* 0.2.2
* Changes.xx
* Fixed status topic names
* 0.2.1
* Changes.
* Added needs reset lighting pattern
* Contributors: Luis Camero, Roni Kreinin, Tony Baltovski, luis-camero

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
* Contributors: Luis Camero, Roni Kreinin, Tony Baltovski, luis-camero

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
* Fixed lighting lib install
* Contributors: Roni Kreinin

0.2.3 (2024-01-08)
------------------

0.2.2 (2024-01-04)
------------------
* Fixed status topic names
* Contributors: Roni Kreinin

0.2.1 (2023-12-21)
------------------

0.2.0 (2023-12-08)
------------------
* Pass robot description to controller manager over topic
* [clearpath_platform] Re-added position state to hardware interface.
* Added W200 Hardware interface.
* Use path substitution
* Updated lighting patterns
  Added charged state
* Comments
* Cleanup
* Fill lights by platform
* Lighting states
* Working HSV
* Initial lighting node
* Whitespace
* Base diff drive hardware and hardware interface class
  J100 and W200 inherit from diff drive
  Moved each platform into its own folder
* Contributors: Luis Camero, Roni Kreinin, Tony Baltovski

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
* Added namespacing support
* Updated dependencies
* Added clearpath_generator_common
  Moved clearpath_platform to clearpath_common
  Fixed use_sim_time parameter issue with ekf_node
* Contributors: Roni Kreinin
