^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_gz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
