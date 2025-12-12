cmake_minimum_required(VERSION 3.8)
project(camping_cart_map)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)

include_directories(
  include
)

add_library(camping_cart_map_lib
  src/lanelet2_map_loader.cpp
)

ament_target_dependencies(camping_cart_map_lib
  rclcpp
  lanelet2_core
  lanelet2_io
  lanelet2_projection
)

add_executable(lanelet2_map_node
  src/lanelet2_map_node.cpp
)

target_link_libraries(lanelet2_map_node
  camping_cart_map_lib
)

ament_target_dependencies(lanelet2_map_node
  rclcpp
  std_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  lanelet2_core
  lanelet2_io
  lanelet2_projection
)

install(TARGETS
  camping_cart_map_lib
  lanelet2_map_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
