cmake_minimum_required(VERSION 3.5)
project(sambirion_description)

find_package(ament_cmake REQUIRED)
# Find the gazebo_ros package to ensure its resources are available
find_package(gazebo_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()