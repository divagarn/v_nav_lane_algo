cmake_minimum_required(VERSION 3.8)
project(radar_fusion)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(fusion_common REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# ROS-independent core (IMM, preprocessing, track management, camera association)
add_library(radar_fusion_core SHARED
  src/motion_models.cpp
  src/imm_filter.cpp
  src/ego_motion.cpp
  src/preprocessors.cpp
  src/fusion_core.cpp
  src/byte_tracker.cpp
  src/path_predictor.cpp)
target_include_directories(radar_fusion_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(radar_fusion_core PUBLIC fusion_common::fusion_common Eigen3::Eigen)

add_executable(radar_fusion_node src/radar_fusion_node.cpp)
target_link_libraries(radar_fusion_node radar_fusion_core)
ament_target_dependencies(radar_fusion_node rclcpp sensor_msgs nav_msgs std_msgs geometry_msgs perception_msgs tf2 tf2_ros)

install(TARGETS radar_fusion_core EXPORT export_radar_fusion
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS radar_fusion_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fusion_core test/test_fusion_core.cpp)
  target_link_libraries(test_fusion_core radar_fusion_core)
  ament_add_gtest(test_byte_tracker test/test_byte_tracker.cpp)
  target_link_libraries(test_byte_tracker radar_fusion_core)
endif()

ament_export_targets(export_radar_fusion HAS_LIBRARY_TARGET)
ament_export_dependencies(fusion_common Eigen3)
ament_package()
