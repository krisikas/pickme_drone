cmake_minimum_required(VERSION 3.8)
project(pickme_offboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(pickme_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(offboard_node
  src/heartbeat_manager.cpp
  src/command_sender.cpp
  src/safety_guard.cpp
  src/trajectory_tracker.cpp
  src/offboard_node.cpp
)
ament_target_dependencies(offboard_node
  rclcpp
  std_srvs
  geometry_msgs
  nav_msgs
  px4_msgs
  pickme_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  offboard_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()