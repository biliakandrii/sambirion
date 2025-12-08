cmake_minimum_required(VERSION 3.8)
project(sambirion_application)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs)

add_executable(odometry_node src/odometry_node.cpp)
ament_target_dependencies(odometry_node rclcpp std_msgs nav_msgs tf2_ros geometry_msgs tf2)

install(TARGETS
  odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(joint_state_publisher_node src/joint_state_publisher_node.cpp)
ament_target_dependencies(joint_state_publisher_node rclcpp std_msgs sensor_msgs)

install(TARGETS
joint_state_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(initial_pose_pub src/initial_pose_pub.cpp)
ament_target_dependencies(initial_pose_pub rclcpp std_msgs sensor_msgs)

install(TARGETS
initial_pose_pub
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS
    src/goal_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
