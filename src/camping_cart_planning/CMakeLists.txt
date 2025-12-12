cmake_minimum_required(VERSION 3.8)
project(camping_cart_planning)

# HH_251203: Global path planner package (refactored from camping_cart_navigation)
# Contains only planning logic; map subsystem moved to camping_cart_map
# See: /home/hong/camping_cart_ws/src/camping_cart_planning/README.md

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Lanelet2 (for routing and path planning)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(lanelet2_traffic_rules REQUIRED)
find_package(lanelet2_routing REQUIRED)

# Include directories
include_directories(
  include
)

# HH_251202: Global Planner Node
add_executable(global_planner_node
  src/global_planner_node.cpp
)

# HH_251202: Global Planner dependencies
ament_target_dependencies(global_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  lanelet2_traffic_rules
  lanelet2_routing
)

# HH_251202: Global Planner include directories
target_include_directories(global_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install
install(
  TARGETS global_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
