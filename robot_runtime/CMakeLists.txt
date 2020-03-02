cmake_minimum_required(VERSION 3.5)
project(robot_runtime)

# This is a launch-only metapackage so only linting is required
find_package(ament_cmake REQUIRED)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
