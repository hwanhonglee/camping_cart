cmake_minimum_required(VERSION 3.8)
project(camping_cart_common)

###############################################
# 1. Find packages
###############################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

###############################################
# 2. Message generation
###############################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotSpecifications.msg"
  "msg/SensorPose.msg"
  "msg/RobotInfo.msg"
)

ament_export_dependencies(rosidl_default_runtime)

###############################################
# 3. Include directories
###############################################
include_directories(
  include
)

###############################################
# 4. Build robot_info_node AFTER msg is generated
###############################################
add_executable(robot_info_node
  src/robot_info_node.cpp
  src/robot_params.cpp
)

ament_target_dependencies(robot_info_node
  rclcpp
  rosidl_default_runtime
)
rosidl_target_interfaces(robot_info_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# important, Include msg header in build/include 
target_include_directories(robot_info_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

###############################################
# 5. Install
###############################################
install(
  TARGETS robot_info_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
###############################################
ament_package()
