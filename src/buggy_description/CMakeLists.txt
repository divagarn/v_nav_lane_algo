cmake_minimum_required(VERSION 3.8)
project(buggy_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz meshes config worlds models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
