cmake_minimum_required(VERSION 3.10)
project(open_apero_ros_bridge)

# Compiler configuration
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  eigen_catkin
  geometry_msgs
  jsk_rviz_plugins
  open_apero_core
  open_apero_msgs
  roscpp
  std_msgs
  visualization_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Package exports. Header-only, so there is no library to build or link -- but INCLUDE_DIRS and
# CATKIN_DEPENDS must both be exported: consumers need this package's headers *and* the core/message
# headers those headers include, and find_package(catkin COMPONENTS open_apero_ros_bridge) resolves
# them from exactly these two lists.
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
