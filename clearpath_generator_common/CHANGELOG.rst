^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_generator_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
