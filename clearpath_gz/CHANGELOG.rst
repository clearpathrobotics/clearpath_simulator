^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_gz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.9.3 (2026-07-08)
------------------
* Added automapping for teleop topic on Gazebo GUI. (`#115 <https://github.com/clearpathrobotics/clearpath_simulator/issues/115>`_)
* Contributors: Tony Baltovski

2.9.2 (2026-05-21)
------------------

2.9.1 (2026-04-21)
------------------
* Fix: robot_spawn.launch.py fails with generate:=false (`#110 <https://github.com/clearpathrobotics/clearpath_simulator/issues/110>`_)
  * Fix: robot_spawn.launch.py fails with generate:=false
  Use UnlessCondition with the launch configuration directly instead of
  bool() on the perform()ed string (which is always truthy for non-empty
  strings, including 'false'). This restores the Humble behaviour where
  generate:=false correctly spawns the robot without regenerating files.
  Fixes `#91 <https://github.com/clearpathrobotics/clearpath_simulator/issues/91>`_
  * Wrap generate actions in GroupAction to mirror humble structure
* Contributors: Tony Baltovski

2.9.0 (2026-02-11)
------------------
* Update cmake version to 3.20 (`#100 <https://github.com/clearpathrobotics/clearpath_simulator/issues/100>`_)
* Rename node from 'generate_launch' to 'generate_param' (`#97 <https://github.com/clearpathrobotics/clearpath_simulator/issues/97>`_)
  Fix copy-paste issue when naming the generate_param node
* Contributors: Andrei Costinescu, luis-camero

2.7.1 (2025-12-16)
------------------

2.7.0 (2025-08-25)
------------------
* Add explicit gravity & realtime update rate to warehouse world (`#90 <https://github.com/clearpathrobotics/clearpath_simulator/issues/90>`_)
  All other worlds include these parameters, so to keep things consistent add them to the Warehouse too.
* Contributors: Chris Iverach-Brereton

2.3.1 (2025-04-30)
------------------
* Convert `generate` to a boolean in if-statement (`#84 <https://github.com/clearpathrobotics/clearpath_simulator/issues/84>`_)
* Contributors: Chris Iverach-Brereton

2.3.0 (2025-04-11)
------------------

2.2.0 (2025-03-11)
------------------
* Forward port: Add argument to disable generation  (`#75 <https://github.com/clearpathrobotics/clearpath_simulator/issues/75>`_)
  * Add argument to disable generation (`#74 <https://github.com/clearpathrobotics/clearpath_simulator/issues/74>`_)
* Contributors: luis-camero

2.0.0 (2025-01-30)
------------------
* Minor cleanup to remove unnecessary/relocated variables
* Add GeoTif files, instructions for generating map tiles (`#60 <https://github.com/clearpathrobotics/clearpath_simulator/issues/60>`_)
  * Add the geotif files & instructions for generating map tiles of the outdoor worlds. Add a column to the worlds table indicating the rough geographic region the world is tagged with
  * Add the .gitignore file
* Rotate the pipeline world so the solar panels face south. Adjust the pose of the ground so the robot's default spawn position is on flatter ground
* Rotate the solar_farm world so the solar panels are correctly facing south
* Increase the XY scaling of the pipeline world (`#58 <https://github.com/clearpathrobotics/clearpath_simulator/issues/58>`_)
* Add more simulation environments (`#57 <https://github.com/clearpathrobotics/clearpath_simulator/issues/57>`_)
  * Add the old cpr_inspection -> pipeline, cpr_agriculture -> solar_farm worlds
  * Add the office & office_construction -> construction worlds
  * Fix the path to the solar_farm models
  * Add the orchard world
  * Fix the world names for the office & construction worlds
  * Add Dave N as an author, since he created the meshes used in the new worlds
* Rename Gazbo libraries (`#55 <https://github.com/clearpathrobotics/clearpath_simulator/issues/55>`_)
  * IGN_GAZEBO_RESOURCE_PATH -> GZ_SIM_RESOURCE_PATH
  * More ignition -> gz fixes. Use stamped messages for cmd_vel
  * Change the clock bridge to unidirectional instead of bidirectional
* Fix import orders in clearpath_gz
* Play sim automatically (`#52 <https://github.com/clearpathrobotics/clearpath_simulator/issues/52>`_)
  * Play sim automatically
  * Accept auto_start as a launch configuration to support previous behavior although default is true
* Rename ign\_ -> gz\_ for gazebo dependencies
* Contributors: Chris Iverach-Brereton, Hilary Luo

0.3.0 (2024-09-19)
------------------
* Added manipulators to generator and spawn
* Contributors: Luis Camero

0.2.6 (2024-07-25)
------------------

0.2.5 (2024-05-28)
------------------

0.2.4 (2024-04-15)
------------------
* Commented out models that are currently broken in the fuel server
* Contributors: Hilary Luo

0.2.3 (2024-01-18)
------------------
* 0.2.2
* Changes.
* Add all sourced ros packages to gazebo resource path to address user workspaces with custom meshes
* 0.2.1
* Changes.
* 0.2.0
* Changes.
* Lifted platform to 0.3 m
* Updated links in the warehouse
* Contributors: Hilary Luo, Luis Camero, Tony Baltovski

0.2.2 (2024-01-15)
------------------
* Add all sourced ros packages to gazebo resource path to address user workspaces with custom meshes
* Contributors: Hilary Luo

0.2.1 (2023-12-11)
------------------

0.2.0 (2023-12-08)
------------------
* Lifted platform to 0.3 m
* Updated links in the warehouse
* Contributors: Luis Camero

0.1.3 (2023-11-03)
------------------
* closes `#16 <https://github.com/clearpathrobotics/clearpath_simulator/issues/16>`_ (`#17 <https://github.com/clearpathrobotics/clearpath_simulator/issues/17>`_)
  Fixes Lidar rays when running without a GPU, still works with GPU
* Contributors: Arthur Gomes

0.1.2 (2023-10-04)
------------------

0.1.1 (2023-08-25)
------------------

0.1.0 (2023-08-17)
------------------
* Linter
* Renamed UST10 to UST
* Contributors: Roni Kreinin

0.0.3 (2023-07-24)
------------------
* Linting
* Contributors: Roni Kreinin

0.0.2 (2023-07-13)
------------------
* [clearpath_gz] Removed ros_gz from CMakeLists.txt.
* Updated imports and getters
* Contributors: Luis Camero, Tony Baltovski

0.0.1 (2023-07-05)
------------------
* Renamed launch file to simulation.launch.py
* Support for empty namespace
  Generate tf and cmd_vel bridges
* Namespacing support
* Renamed clearpath_simulator to clearpath_gz
  clearpath_simulator is now a metapackage
  Added clearpath_generator_gz
* Contributors: Roni Kreinin
