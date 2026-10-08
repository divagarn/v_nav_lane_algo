cmake_minimum_required(VERSION 3.5)
project(buggy_nav_costmap)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
  std_msgs
)

include_directories(include)

add_executable(costmap_lane_planner_node
  src/costmap_lane_planner_main.cpp
  src/costmap_lane_planner.cpp
  src/pose_graph_walker.cpp
)
ament_target_dependencies(costmap_lane_planner_node ${dependencies})

add_executable(dynamic_obstacle_detector_node
  src/dynamic_obstacle_detector_main.cpp
  src/dynamic_obstacle_detector.cpp
)
ament_target_dependencies(dynamic_obstacle_detector_node ${dependencies})

add_executable(path_obstacle_monitor_node
  src/path_obstacle_monitor_main.cpp
  src/path_obstacle_monitor.cpp
)
ament_target_dependencies(path_obstacle_monitor_node ${dependencies})

install(TARGETS
  costmap_lane_planner_node
  dynamic_obstacle_detector_node
  path_obstacle_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
