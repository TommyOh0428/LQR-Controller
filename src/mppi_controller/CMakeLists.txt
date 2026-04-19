cmake_minimum_required(VERSION 3.8)
project(mppi_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/mppi_controller.cpp
  src/mppi_solver.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  nav2_core
  nav2_costmap_2d
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  angles
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(nav2_core mppi_controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES mppi_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
