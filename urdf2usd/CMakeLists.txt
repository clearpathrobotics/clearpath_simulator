cmake_minimum_required(VERSION 3.16)
project(urdf2usd)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the ros2-run entry point. This is the ROS-side (system Python)
# launcher; it exec's the Isaac-Sim-side worker under ${ISAAC_SIM_PATH}/python.sh.
# See urdf2usd/cli.py and urdf2usd/_isaac_worker.py.
install(PROGRAMS
  scripts/urdf2usd_export
  DESTINATION lib/${PROJECT_NAME}
)

# Install supporting resources (example URDFs, etc.)
install(
  DIRECTORY resources scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Install the Python package
ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(parser_tests test/test_parser.py)
endif()

ament_package()
