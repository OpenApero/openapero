cmake_minimum_required(VERSION 3.10)
project(open_apero_ros)

# Compiler configuration
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Werror=return-type)

# Dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  rospy
  open_apero_core
  open_apero_msgs
  open_apero_ros_bridge
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2
  tf2_ros
  message_filters
  pcl_conversions
  pcl_ros
  cv_bridge
  jsk_rviz_plugins
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Python module (python/open_apero_ros): the mission coordinator nodes/mission_coordinator_node runs, and
# the hand-run tools scripts/ runs, one module each.
catkin_python_setup()

# Package exports
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Build
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

# SHARED: PointsEvaluator is only selected by name; a static library could drop its registration.
add_library(${PROJECT_NAME} SHARED
  src/plugin/plugin_factory_ros.cpp
  src/plugin/evaluator/points_evaluator.cpp
  src/agent/ros_agent.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(collectible_points_visualization_node
  src/collectible_points_visualization_node.cpp
)
add_dependencies(collectible_points_visualization_node ${catkin_EXPORTED_TARGETS})
# Renders CollectiblePointArray messages; does not touch the planner library.
target_link_libraries(collectible_points_visualization_node
  ${catkin_LIBRARIES}
)

add_executable(rgbd_to_pointcloud_node
  src/rgbd_to_pointcloud_node.cpp
)
add_dependencies(rgbd_to_pointcloud_node ${catkin_EXPORTED_TARGETS})
# Pure cv_bridge/PCL sensor plumbing; does not touch the planner library.
target_link_libraries(rgbd_to_pointcloud_node
  ${catkin_LIBRARIES}
)

add_executable(open_apero_node
  src/open_apero_node.cpp
)
add_dependencies(open_apero_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(open_apero_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    collectible_points_visualization_node
    rgbd_to_pointcloud_node
    open_apero_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

catkin_install_python(PROGRAMS
    nodes/mission_coordinator_node
    scripts/compare_paper_sweep.py
    scripts/evaluate_experiment.py
    scripts/generate_points.py
    scripts/paper_sweep.py
    scripts/plot_normalized_gain.py
    scripts/plot_performance_curves.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS
    scripts/run_experiments.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# Runtime resources that launch files and nodes resolve through $(find
# open_apero_ros) / package:// URIs. Without these the package works only from a
# devel space; an installed build would resolve the paths to missing files.
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
# Referenced by package:// URIs in collectible_points_visualization_node.cpp.
install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)
# The PointsEvaluator input (data/points_3d.csv).
install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)

# Tests. The C++ ones need no ROS master. The Python suites use pytest fixtures, so they run under
# pytest and report through the JUnit file catkin_test_results reads.
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/test_main.cpp
    test/test_plugin_factory_ros.cpp
    test/test_points_evaluator.cpp
    test/test_status_overlay.cpp
    test/test_marker_conversion.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})

  set(pytest_results_dir ${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME})
  catkin_run_tests_target(pytest ${PROJECT_NAME} pytest-${PROJECT_NAME}.xml
    COMMAND "\"${CMAKE_COMMAND}\" -E make_directory ${pytest_results_dir}"
            "\"${PYTHON_EXECUTABLE}\" -m pytest ${CMAKE_CURRENT_SOURCE_DIR}/test -p no:cacheprovider --junitxml=${pytest_results_dir}/pytest-${PROJECT_NAME}.xml"
  )
endif()
