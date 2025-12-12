cmake_minimum_required(VERSION 3.5)
project(nav2_dstar_lite)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

add_library(dstar_lite_planner SHARED
  src/dstar_lite_planner.cpp
)

ament_target_dependencies(dstar_lite_planner
  rclcpp
  nav2_core
  nav2_costmap_2d
  nav_msgs
  geometry_msgs
  tf2_ros
)

pluginlib_export_plugin_description_file(nav2_core pluginlib_plugins.xml)

install(
  TARGETS dstar_lite_planner
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
