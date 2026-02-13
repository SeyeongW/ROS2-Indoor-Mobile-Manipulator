cmake_minimum_required(VERSION 3.8)
project(omx_moveit_pick_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)

# ==========================================
# 1. Main Node (omx_moveit_pick_node) - 수정됨
# ==========================================
add_executable(omx_moveit_pick_node src/omx_moveit_pick_node.cpp)
ament_target_dependencies(omx_moveit_pick_node
  rclcpp
  rclcpp_action
  control_msgs
  geometry_msgs
  trajectory_msgs
  tf2_ros
  tf2_geometry_msgs
  ros2_aruco_interfaces
  sensor_msgs
  moveit_ros_planning_interface
  moveit_ros_move_group
)

# ==========================================
# 2. Grip Node (omx_grip_node) - 유지
# ==========================================
add_executable(omx_grip_node src/omx_grip_node.cpp)
ament_target_dependencies(omx_grip_node
  rclcpp
  rclcpp_action
  control_msgs
  geometry_msgs
  trajectory_msgs
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  ros2_aruco_interfaces
  moveit_ros_planning_interface
)

# ==========================================
# 3. Install
# ==========================================
install(TARGETS
  omx_moveit_pick_node
  omx_grip_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/aruco_markers_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()