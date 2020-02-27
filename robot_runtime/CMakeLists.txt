cmake_minimum_required(VERSION 3.5)
project(robot_runtime)

# This is a launch-only metapackage so only linting is required
find_package(ament_cmake REQUIRED)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
