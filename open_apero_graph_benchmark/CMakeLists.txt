cmake_minimum_required(VERSION 3.10)
project(open_apero_graph_benchmark)

# Compiler configuration
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Werror=return-type)

# Dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  visualization_msgs
  jsk_rviz_plugins
  open_apero_core
  open_apero_ros_bridge
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

# The importable Python package (python/open_apero_graph_benchmark) the scripts build on.
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

add_library(${PROJECT_NAME}
  src/benchmark_status.cpp
  src/graph_manager.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(benchmark_node
  src/benchmark_node.cpp
)
add_dependencies(benchmark_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(benchmark_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(generate_graph_instance_node
  src/generate_graph_instance_node.cpp
)
add_dependencies(generate_graph_instance_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(generate_graph_instance_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Installation
install(TARGETS ${PROJECT_NAME} benchmark_node generate_graph_instance_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

catkin_install_python(PROGRAMS
    scripts/benchmark_reference.py
    scripts/compare_result_trees.py
    scripts/generate_clustered_gain_model.py
    scripts/generate_random_graph_topology.py
    scripts/plot_computation_time_ranges.py
    scripts/plot_final_gain_ranges.py
    scripts/render_graph.py
    scripts/verify_benchmark_results.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# Resources the launch files and tools resolve through $(find open_apero_graph_benchmark) / rospkg:
# the graph inputs and each dataset's reference.json. Results are written below data_root, never here.
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
  FILES_MATCHING PATTERN "*.txt" PATTERN "reference.json"
)

# Tests. The Python suites use pytest fixtures, so they run under pytest and report through the
# JUnit file catkin_test_results reads.
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/test_benchmark_status.cpp
    test/test_benchmark_sweep.cpp
    test/test_graph_manager.cpp
    test/test_main.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})

  set(pytest_results_dir ${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME})
  catkin_run_tests_target(pytest ${PROJECT_NAME} pytest-${PROJECT_NAME}.xml
    COMMAND "\"${CMAKE_COMMAND}\" -E make_directory ${pytest_results_dir}"
            "\"${PYTHON_EXECUTABLE}\" -m pytest ${CMAKE_CURRENT_SOURCE_DIR}/test -p no:cacheprovider --junitxml=${pytest_results_dir}/pytest-${PROJECT_NAME}.xml"
  )
endif()
