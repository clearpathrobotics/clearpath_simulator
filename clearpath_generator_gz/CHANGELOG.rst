^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_generator_gz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.9.3 (2026-07-08)
------------------

2.9.2 (2026-05-21)
------------------
* Feature: PTU (`#109 <https://github.com/clearpathrobotics/clearpath_simulator/issues/109>`_)
* Contributors: Tony Baltovski

2.9.1 (2026-04-21)
------------------
* Feature: Generator Sample Tests (`#107 <https://github.com/clearpathrobotics/clearpath_simulator/issues/107>`_)
  * Add generic platform to look up table
  * Add generator tests to CI
  * Create sensor directory only if sensors are present
  * Only run base CI if PR
  * Add generator tests to README
* Contributors: luis-camero

2.9.0 (2026-02-11)
------------------
* Update cmake version to 3.20 (`#100 <https://github.com/clearpathrobotics/clearpath_simulator/issues/100>`_)
* Add SeyondLidar support in sensors.py (param generator) (`#98 <https://github.com/clearpathrobotics/clearpath_simulator/issues/98>`_)
  * Add SeyondLidar support in sensors.py (param generator)
  * Reorganize imports for 3D lidars in sensors.py
  Fix imports out-of-alphabetical-order and line too long
* Contributors: Andrei Costinescu, luis-camero

2.7.1 (2025-12-16)
------------------
* Added OusterOS1 to param generator (`#96 <https://github.com/clearpathrobotics/clearpath_simulator/issues/96>`_)
* Contributors: Roni Kreinin

2.7.0 (2025-08-25)
------------------

2.3.1 (2025-04-30)
------------------

2.3.0 (2025-04-11)
------------------
* Add exception handlers to the generators for Unsupported* exceptions (`#78 <https://github.com/clearpathrobotics/clearpath_simulator/issues/78>`_)
* Contributors: Chris Iverach-Brereton

2.2.0 (2025-03-11)
------------------

2.0.0 (2025-01-30)
------------------
* Fix typo in IMU bridge node
* Non-functional linting issues resulting in CI failures (`#67 <https://github.com/clearpathrobotics/clearpath_simulator/issues/67>`_)
* Add A300 to supported platforms (`#63 <https://github.com/clearpathrobotics/clearpath_simulator/issues/63>`_)
  * Add A300 to the supported platforms
  * Move the cmd_vel and odom bridges to a common components array
* Implement PTZ action server interface (`#65 <https://github.com/clearpathrobotics/clearpath_simulator/issues/65>`_)
  * Add remappings & bridge configuration for the PTZ joint states
  * Remap the raw velocity topics from Gazebo, implement the PTZ action server interface. Add a 5x digital zoom to simulate real-world zoom control
* Add pan & tilt velocity commands (`#64 <https://github.com/clearpathrobotics/clearpath_simulator/issues/64>`_)
* Rename Gazbo libraries (`#55 <https://github.com/clearpathrobotics/clearpath_simulator/issues/55>`_)
  * IGN_GAZEBO_RESOURCE_PATH -> GZ_SIM_RESOURCE_PATH
  * More ignition -> gz fixes. Use stamped messages for cmd_vel
  * Change the clock bridge to unidirectional instead of bidirectional
* Add a newline that the source CI is complaining about that didn't show up locally
* Fix formatting for Jazzy
* Contributors: Chris Iverach-Brereton

0.3.0 (2024-09-19)
------------------
* Removed prefix_launch_arg no longer used
* Add ridgeback
* Fixed change log order
* Added manipulators to generator and spawn
* Contributors: Luis Camero

0.2.6 (2024-07-25)
------------------
* Add gz image transport for cameras
* IMU data raw remap
* Generate gazebo bridge config parameters
* Removed bridging using remappings and arguments
* Contributors: Luis Camero

0.2.5 (2024-05-28)
------------------
* Added Zed to simulator
* Contributors: Luis Camero

0.2.4 (2024-04-15)
------------------

0.2.3 (2024-01-18)
------------------
* Removed print
* Removed namespaced tf_static
* 0.2.2
* Changes.
* 0.2.1
* Changes.
* Added all dingo to generator
* 0.2.0
* Changes.
* Added Warthog and Dingo to generator
* Added Generic robot components
* Sensor static tf should use robot namespace
* Contributors: Luis Camero, Roni Kreinin, Tony Baltovski, luis-camero

0.2.2 (2024-01-15)
------------------

0.2.1 (2023-12-11)
------------------
* Added all dingo to generator
* Contributors: Luis Camero

0.2.0 (2023-12-08)
------------------
* Added Warthog and Dingo to generator
* Added Generic robot components
* Sensor static tf should use robot namespace
* Contributors: Luis Camero, Roni Kreinin

0.1.3 (2023-11-03)
------------------

0.1.2 (2023-10-04)
------------------
* Initial addition of Blackfly (`#19 <https://github.com/clearpathrobotics/clearpath_simulator/issues/19>`_)
  * Initial addition of Blackfly
* Contributors: Hilary Luo

0.1.1 (2023-08-25)
------------------
* Linting
* Added additional supported IMUs and GPSs
* Fix imu and gps bridge topic remapping
* Remove static tf nodes, no longer necessary
* Update topic names to match other packages
* Contributors: Hilary Luo

0.1.0 (2023-08-17)
------------------
* Renamed UST10 to UST
* Contributors: Roni Kreinin

0.0.3 (2023-07-24)
------------------
* Linting
* Added prefix launch arg for A200
* Updated param generator 'use_sim_time' implementation
* Launch generator cleanup
* Contributors: Roni Kreinin

0.0.2 (2023-07-13)
------------------
* Updated imports and getters
* Contributors: Luis Camero

0.0.1 (2023-07-05)
------------------
* Changed colour to color
* Added dependencies.repos
  Updated topic names to match API
* Support for empty namespace
  Generate tf and cmd_vel bridges
* Namespacing support
* Renamed clearpath_simulator to clearpath_gz
  clearpath_simulator is now a metapackage
  Added clearpath_generator_gz
* Contributors: Roni Kreinin
