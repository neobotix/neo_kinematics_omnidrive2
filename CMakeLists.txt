cmake_minimum_required(VERSION 3.5)
project(neo_kinematics_omnidrive2)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(neo_srvs2 REQUIRED)
find_package(neo_msgs2 REQUIRED)
find_package(neo_common2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(angles REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(neo_omnidrive_node src/neo_omnidrive_node.cpp)
# add_executable(neo_omnidrive_simulation_node src/neo_omnidrive_simulation_node.cpp)
add_executable(neo_omnidrive_socketcan_node src/neo_omnidrive_socketcan.cpp)

# ament_target_dependencies(neo_omnidrive_simulation_node
#   rclcpp
#   tf2_ros
#   sensor_msgs
#   geometry_msgs
#   std_msgs
#   nav_msgs
#   trajectory_msgs
#   neo_common2
#   angles)

ament_target_dependencies(neo_omnidrive_node
  rclcpp
  tf2_ros
  sensor_msgs
  geometry_msgs
  std_msgs
  neo_msgs2
  neo_srvs2
  nav_msgs
  trajectory_msgs
  tf2_geometry_msgs
  neo_common2
  angles)

ament_target_dependencies(neo_omnidrive_socketcan_node
  rclcpp
  tf2_ros
  sensor_msgs
  neo_msgs2
  geometry_msgs
  std_msgs
  nav_msgs
  trajectory_msgs
  tf2_geometry_msgs
  neo_common2
  angles)

ament_export_include_directories(include)

install(DIRECTORY include/
   DESTINATION include/
)

install(TARGETS 
#neo_omnidrive_simulation_node 
neo_omnidrive_socketcan_node 
neo_omnidrive_node
DESTINATION lib/${PROJECT_NAME}
	)

install(DIRECTORY launch
  	
  DESTINATION share/${PROJECT_NAME})

ament_package()
