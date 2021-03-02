cmake_minimum_required(VERSION 3.5)
project(robot_gazebo_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(robot_dock_ir SHARED src/dock_ir.cpp)
target_include_directories(robot_dock_ir PUBLIC src)
ament_target_dependencies(robot_dock_ir
  gazebo_dev
  gazebo_ros
  geometry_msgs
  kobuki_ros_interfaces
  rclcpp
)
ament_export_libraries(robot_dock_ir)

add_library(robot_dock_vis SHARED src/dock_vis.cpp)
target_include_directories(robot_dock_vis PUBLIC src)
ament_target_dependencies(robot_dock_vis
  gazebo_dev
  gazebo_ros
  geometry_msgs
  kobuki_ros_interfaces
  rclcpp
)
ament_export_libraries(robot_dock_vis)

install(
  TARGETS robot_dock_ir robot_dock_vis
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
