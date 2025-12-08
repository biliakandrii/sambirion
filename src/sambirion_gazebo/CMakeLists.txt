cmake_minimum_required(VERSION 3.8)
project(sambirion_gazebo)

# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(sambirion_description REQUIRED)

# Find Gazebo Classic.
# This requires you to source the Gazebo setup file before building,
# e.g., `source /usr/share/gazebo/setup.sh`
find_package(gazebo REQUIRED)

#--- Plugin Libraries ---

# Define the BasicSystem plugin library
add_library(BasicSystem SHARED src/BasicSystem.cc)

# Specify include directories for BasicSystem
target_include_directories(BasicSystem PRIVATE
  "include"
  ${GAZEBO_INCLUDE_DIRS}
)

# Link BasicSystem against Gazebo libraries
target_link_libraries(BasicSystem PRIVATE ${GAZEBO_LIBRARIES})

# Define the FullSystem plugin library
add_library(FullSystem SHARED src/FullSystem.cc)

# Specify include directories for FullSystem
target_include_directories(FullSystem PRIVATE
  "include"
  ${GAZEBO_INCLUDE_DIRS}
)

# Link FullSystem against Gazebo libraries
target_link_libraries(FullSystem PRIVATE ${GAZEBO_LIBRARIES})

#--- Installation ---

# Install the compiled plugin libraries
install(
  TARGETS BasicSystem FullSystem
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

# Install the world files
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

#--- Environment Hooks ---
# This hook ensures Gazebo can find your plugins after sourcing the workspace.
# You will need to create the hook file as explained in the comments below.
#
# In a `hooks` folder, create a file named `sambirion_gazebo.sh.in` with this content:
# ament_prepend_unique_value GAZEBO_PLUGIN_PATH "$AMENT_CURRENT_PREFIX/lib/sambirion_gazebo"
#
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/sambirion_gazebo.sh.in"
)

#--- Testing ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize the ament package configuration
ament_package()