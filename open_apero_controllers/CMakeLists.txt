cmake_minimum_required(VERSION 3.10)
project(open_apero_controllers)

# Dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  geometry_msgs
  open_apero_msgs
  rospy
  std_msgs
  tf2_ros
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Python module (python/open_apero_controllers): the node implementations.
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_install_python(PROGRAMS
    nodes/dummy_controller_node
    nodes/keyboard_teleop_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Python unit tests: the suites run under pytest and report through the JUnit file
# catkin_test_results reads.
if(CATKIN_ENABLE_TESTING)
  set(pytest_results_dir ${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME})
  catkin_run_tests_target(pytest ${PROJECT_NAME} pytest-${PROJECT_NAME}.xml
    COMMAND "\"${CMAKE_COMMAND}\" -E make_directory ${pytest_results_dir}"
            "\"${PYTHON_EXECUTABLE}\" -m pytest ${CMAKE_CURRENT_SOURCE_DIR}/test -p no:cacheprovider --junitxml=${pytest_results_dir}/pytest-${PROJECT_NAME}.xml"
  )
endif()
