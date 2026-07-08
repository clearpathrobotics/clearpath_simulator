^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_gz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.3 (2026-07-08)
------------------
* Added automapping for teleop topic on Gazebo GUI.
* [Humble] Fix: CI in Container (`#104 <https://github.com/clearpathrobotics/clearpath_simulator/issues/104>`_)
  Co-authored-by: luis-camero <88782189+luis-camero@users.noreply.github.com>
  Co-authored-by: Tony Baltovski <tbaltovski@clearpathrobotics.com>
* Contributors: mergify[bot]

1.3.2 (2025-07-14)
------------------
* Fixed `generate` flag for `robot_spawn` (`#89 <https://github.com/clearpathrobotics/clearpath_simulator/issues/89>`_)
  * Update simulation.launch.py to have generate setting
  * properlly define the generate flag usage
  * added missing import
* Contributors: Marius Juston

1.3.1 (2025-04-30)
------------------
* Convert `generate` to a boolean in if-statement (`#84 <https://github.com/clearpathrobotics/clearpath_simulator/issues/84>`_) (`#85 <https://github.com/clearpathrobotics/clearpath_simulator/issues/85>`_)
* Contributors: Chris Iverach-Brereton

1.3.0 (2025-04-15)
------------------
* Fix: Generation disable (`#76 <https://github.com/clearpathrobotics/clearpath_simulator/issues/76>`_)
* Add argument to disable generation (`#74 <https://github.com/clearpathrobotics/clearpath_simulator/issues/74>`_)
* Feature: MoveIt Parameters and Enable (`#70 <https://github.com/clearpathrobotics/clearpath_simulator/issues/70>`_)
* Contributors: Luis Camero

1.0.0 (2024-11-25)
------------------
* Added minimum version.
* Play sim automatically (`#52 <https://github.com/clearpathrobotics/clearpath_simulator/issues/52>`_)
  * Play sim automatically
  * Accept auto_start as a launch configuration to support previous behavior although default is true
* Contributors: Hilary Luo, Tony Baltovski

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
