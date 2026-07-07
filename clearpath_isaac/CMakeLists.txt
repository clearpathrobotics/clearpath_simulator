cmake_minimum_required(VERSION 3.20)
project(clearpath_isaac)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY docker launch scripts urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # The Python files in scripts/ are not a ROS Python package; they are
  # entry points run inside the Isaac Sim container (which requires
  # SimulationApp() to be constructed before any pxr/omni imports). Skip
  # the Python linters that don't tolerate that pattern.
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
