cmake_minimum_required(VERSION 3.5)
project(robot_stack)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
