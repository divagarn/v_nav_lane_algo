cmake_minimum_required(VERSION 3.8)
project(buggy_mapping_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)

add_executable(save_octomap_node src/save_octomap_node.cpp)
target_include_directories(save_octomap_node PUBLIC ${OCTOMAP_INCLUDE_DIRS})
ament_target_dependencies(save_octomap_node rclcpp octomap_msgs)
target_link_libraries(save_octomap_node ${OCTOMAP_LIBRARIES})

install(TARGETS save_octomap_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
