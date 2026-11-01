cmake_minimum_required(VERSION 3.10)
project(open_apero_analysis)

find_package(catkin REQUIRED)

# Python module (python/open_apero_analysis): the analysis library. No compiled targets.
catkin_python_setup()

catkin_package()

# Python unit tests: the suites use pytest fixtures, so they run under pytest and
# report through the JUnit file catkin_test_results reads.
if(CATKIN_ENABLE_TESTING)
  set(pytest_results_dir ${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME})
  catkin_run_tests_target(pytest ${PROJECT_NAME} pytest-${PROJECT_NAME}.xml
    COMMAND "\"${CMAKE_COMMAND}\" -E make_directory ${pytest_results_dir}"
            "\"${PYTHON_EXECUTABLE}\" -m pytest ${CMAKE_CURRENT_SOURCE_DIR}/test -p no:cacheprovider --junitxml=${pytest_results_dir}/pytest-${PROJECT_NAME}.xml"
  )
endif()
