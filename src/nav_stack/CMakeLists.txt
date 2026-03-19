cmake_minimum_required(VERSION 3.8)
project(nav_stack)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(aruco_markers_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# BT dependencies - now uncommented for Nav2 behaviors
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)

# Define BT dependencies for reuse
set(bt_dependencies
  rclcpp
  geometry_msgs
  aruco_markers_msgs
  tf2_ros
  tf2_geometry_msgs
  nav2_behavior_tree
  nav2_core
  nav2_msgs
  behaviortree_cpp
  pluginlib
  std_srvs
  sensor_msgs
)

# C++ executable for multi-camera ArUco detection (commented out)
# add_executable(multi_camera_aruco_detector src/multi_camera_aruco_detector.cpp)
# ament_target_dependencies(multi_camera_aruco_detector
#   rclcpp
#   geometry_msgs
#   aruco_markers_msgs
#   tf2_ros
#   tf2_geometry_msgs
# )

# Create the multi-camera BT plugin library
# add_library(multi_camera_bt_nodes SHARED
#   src/multi_camera_bt_nodes.cpp
#   src/bt_plugin_registration.cpp
# )

# ament_target_dependencies(multi_camera_bt_nodes ${bt_dependencies})

# # Include directories for BT nodes
# target_include_directories(multi_camera_bt_nodes PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# Install BT plugin library
# install(TARGETS multi_camera_bt_nodes
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# Install Python executables
install(PROGRAMS
  src/aruco_nav_goal.py
  src/pose_map.py
  src/buffer.py
  src/topic_remapper.py
  src/monitor_node.py
  src/pc2_stablize.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install C++ executables (commented out since executable is commented)
# install(TARGETS
#   multi_camera_aruco_detector
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install launch files, config, and behavior trees
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install BT plugin description file for custom BT nodes
# install(FILES
#   bt_plugins.xml
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()