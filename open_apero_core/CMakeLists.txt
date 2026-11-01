cmake_minimum_required(VERSION 3.10)
project(open_apero_core)

# Compiler configuration
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Werror=return-type)

# Dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  eigen_catkin
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(assimp REQUIRED)
find_package(ompl REQUIRED)

# Package exports
catkin_package(
  INCLUDE_DIRS
    include
    ${OMPL_INCLUDE_DIRS}
    ${ASSIMP_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
    ${OMPL_LIBRARIES}
    assimp
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Build
# Suppress warnings originating in external headers.
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIRS}
)

set(SRC_FILES
  src/common/utils.cpp
  src/graph/tree_node.cpp
  src/graph/graph_node.cpp
  src/geometry/bounding_volume.cpp
  src/geometry/geometry_primitives.cpp
  src/gain/gain_cost_table.cpp
  src/gain/gain_suppression.cpp
  src/state/robot_state.cpp
  src/state/history_data.cpp
  src/state/observed_viewpoints.cpp
  src/logging/logging.cpp
  src/logging/csv_writer.cpp
  src/logging/performance_log.cpp
  src/logging/schema_fingerprint.cpp
  src/logging/run_manifest.cpp
  src/logging/episode_summary.cpp
  src/visualization/visualization_markers.cpp
  src/visualization/visualization_helpers.cpp
  src/visualization/colormap.cpp
  src/agent/agent.cpp
  src/plugin/layer/layer.cpp
  src/plugin/layer/planning_layer.cpp
  src/plugin/layer/backtracking_layer.cpp
  src/plugin/map/map.cpp
  src/plugin/map/surface_frontier_classifier.cpp
  src/plugin/backtracker/rotate_in_place.cpp
  src/plugin/backtracker/reverse.cpp
  src/plugin/backtracker/rotate_reverse.cpp
  src/plugin/cost_computer/identity_cost.cpp
  src/plugin/cost_computer/trajectory_length.cpp
  src/plugin/cost_computer/trajectory_time.cpp
  src/plugin/plugin_factory.cpp
  src/plugin/plugin_factory_config.cpp
  src/plugin/plugin_library.cpp
  src/plugin/plugin_registry.cpp
  src/plugin/plugin.cpp
  src/plugin/sensor_model/camera_model.cpp
  src/plugin/sensor_model/iterative_ray_caster_lidar.cpp
  src/plugin/sensor_model/iterative_ray_caster.cpp
  src/plugin/sensor_model/lidar_model.cpp
  src/plugin/sensor_model/simple_ray_caster.cpp
  src/plugin/sensor_model/sensor_model.cpp
  src/plugin/robot_model/robot_model.cpp
  src/plugin/robot_model/simple_robot.cpp
  src/plugin/robot_model/simple_fallback_robot.cpp
  src/plugin/robot_model/ompl_adapters.cpp
  src/plugin/evaluator/evaluator.cpp
  src/plugin/evaluator/surface_frontier_evaluator.cpp
  src/plugin/evaluator/naive_evaluator.cpp
  src/plugin/evaluator/simple_yaw_planning_evaluator.cpp
  src/plugin/evaluator/simulated_sensor_evaluator.cpp
  src/plugin/evaluator/voxel_weight_evaluator.cpp
  src/plugin/evaluator/yaw_planning_evaluator.cpp
  src/plugin/generator/tree_generator.cpp
  src/plugin/generator/rrt_star.cpp
  src/plugin/generator/rrt.cpp
  src/plugin/generator/rrag.cpp
  src/plugin/generator/graph_generator.cpp
  src/plugin/generator/generator.cpp
  src/plugin/path_planner/path_planner.cpp
  src/plugin/path_planner/helpers.cpp
  src/plugin/path_planner/no_op_path_planner.cpp
  src/plugin/path_planner/tree_best_value.cpp
  src/plugin/path_planner/graph_tsp_search.cpp
  src/plugin/path_planner/graph_node_wise_beam_search.cpp
  src/plugin/path_planner/graph_beam_search.cpp
  src/plugin/path_planner/graph_depth_wise_beam_search.cpp
  src/plugin/path_planner/graph_shortest_path_tree_search.cpp
)

# Built explicitly SHARED: the plugin registry stores its map in a single
# out-of-line translation unit, so registrations from other shared objects
# (e.g. a downstream plugin library) share one registry. A static core would give each
# consumer its own registry and break cross-DSO plugin lookup.
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
# Project targets only: warnings from SYSTEM include directories stay suppressed.
set(OPEN_APERO_CORE_WARNING_FLAGS -Wall -Wextra)
target_compile_options(${PROJECT_NAME} PRIVATE ${OPEN_APERO_CORE_WARNING_FLAGS})

# Regenerate git provenance (build_info.h) at build time so the recorded SHA
# reflects the source actually compiled. The generator rewrites the header only
# when its content changes, so this does not force a relink every build.
set(BUILD_INFO_DIR ${CMAKE_CURRENT_BINARY_DIR}/gen_include)
set(BUILD_INFO_HEADER ${BUILD_INFO_DIR}/open_apero_core/logging/build_info.h)
add_custom_target(${PROJECT_NAME}_build_info
  BYPRODUCTS ${BUILD_INFO_HEADER}
  COMMAND ${CMAKE_COMMAND}
    -DGIT_SRC_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    -DOUTPUT_FILE=${BUILD_INFO_HEADER}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_build_info.cmake
  COMMENT "Generating build_info.h (git provenance)"
  VERBATIM)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_build_info)
target_include_directories(${PROJECT_NAME} PRIVATE ${BUILD_INFO_DIR})

# The installed core and OR-Tools libraries live together, so resolve OR-Tools
# relative to the artifact. Keep only external OMPL search directories (needed
# when LD_LIBRARY_PATH is unset); never a source/build-tree path.
set(OPEN_APERO_CORE_INSTALL_RPATH "\$ORIGIN")
foreach(OMPL_LIBRARY_DIR IN LISTS OMPL_LIBRARY_DIRS)
  if(IS_ABSOLUTE "${OMPL_LIBRARY_DIR}" AND
     NOT "${OMPL_LIBRARY_DIR}" MATCHES "^${CMAKE_SOURCE_DIR}(/|$)" AND
     NOT "${OMPL_LIBRARY_DIR}" MATCHES "^${CMAKE_BINARY_DIR}(/|$)")
    list(APPEND OPEN_APERO_CORE_INSTALL_RPATH "${OMPL_LIBRARY_DIR}")
  endif()
endforeach()
list(REMOVE_DUPLICATES OPEN_APERO_CORE_INSTALL_RPATH)
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH "${OPEN_APERO_CORE_INSTALL_RPATH}"
  INSTALL_RPATH_USE_LINK_PATH FALSE
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# OR-Tools is installed manually, not through rosdep: fail at configure time
# rather than with a missing ortools/constraint_solver/routing.h deep in the build.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/ortools/lib/libortools.so")
  message(FATAL_ERROR "OR-Tools is not installed under open_apero_core/ortools; see docs/installation.md, 'OR-Tools Setup'.")
endif()

# Keep the OR-Tools headers out of the public include interface. SYSTEM silences their
# warnings; BEFORE keeps them searched ahead of the other system include directories.
target_include_directories(${PROJECT_NAME}
  SYSTEM BEFORE PRIVATE
    ortools/include
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  assimp
  ${CMAKE_CURRENT_SOURCE_DIR}/ortools/lib/libortools.so
  ${CMAKE_DL_LIBS}  # dlopen() in loadPluginLibrary(); a separate libdl before glibc 2.34
)

# Installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install the OR-Tools library with its symlink chain (libortools.so ->
# libortools.so.9 -> libortools.so.9.11.4210): the loader resolves the SONAME
# libortools.so.9, and install(DIRECTORY) copies symlinks as symlinks.
install(DIRECTORY ortools/lib/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  FILES_MATCHING PATTERN "libortools.so*"
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Tests
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/test_main.cpp
    test/test_plugin_registry.cpp
    test/test_plugin_library.cpp
    test/test_plugin_factory_config.cpp
    test/test_layer_stack.cpp
    test/test_logging.cpp
    test/test_detached_intermediate_start_node.cpp
    test/test_generator_sampling.cpp
    test/test_map_update_gating.cpp
    test/test_gain_cost_table.cpp
    test/test_gain_suppression.cpp
    test/test_gain_suppression_search.cpp
    test/test_history_data.cpp
    test/test_observed_viewpoints.cpp
    test/test_camera_observation_boundary.cpp
    test/test_iterative_ray_caster_resolution.cpp
    test/test_ray_hit_voxel.cpp
    test/test_path_planner_selection.cpp
    test/test_goal_constrained_search.cpp
    test/test_path_ledger.cpp
    test/test_previous_path_horizon.cpp
    test/test_raw_gain_correlation.cpp
    test/test_planning_layer_start_node.cpp
    test/test_planning_layer_validation.cpp
    test/test_retarget_trajectory_endpoints.cpp
    test/test_cost_lower_bound.cpp
    test/test_rrag_cluster_construction.cpp
    test/test_simple_fallback_robot.cpp
    test/test_simple_robot_collision_check_yaw.cpp
    test/test_simple_robot_sampling.cpp
    test/test_simple_robot_traversability.cpp
    test/test_stationing_positions.cpp
    test/test_surface_frontier_classifier.cpp
    test/test_translation_adjacency_modes.cpp
    test/test_utils.cpp
    test/test_node_copy.cpp
    test/test_node_markers.cpp
    test/test_zero_motion_edges.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})
  target_compile_options(test_${PROJECT_NAME} PRIVATE ${OPEN_APERO_CORE_WARNING_FLAGS})

  # Plugin libraries that test_plugin_library.cpp loads with loadPluginLibrary(); nothing links them.
  # Built into the build tree, not the devel lib directory, so no node can list them by accident.
  foreach(PLUGIN_LIBRARY loaded_plugin duplicate_plugin)
    add_library(test_${PROJECT_NAME}_${PLUGIN_LIBRARY} SHARED test/plugin_library/${PLUGIN_LIBRARY}.cpp)
    target_link_libraries(test_${PROJECT_NAME}_${PLUGIN_LIBRARY} ${PROJECT_NAME})
    set_target_properties(test_${PROJECT_NAME}_${PLUGIN_LIBRARY} PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test_plugin_library)
    add_dependencies(test_${PROJECT_NAME} test_${PROJECT_NAME}_${PLUGIN_LIBRARY})
  endforeach()
  target_compile_definitions(test_${PROJECT_NAME} PRIVATE
    "OPEN_APERO_TEST_LOADED_PLUGIN_LIBRARY=\"$<TARGET_FILE:test_${PROJECT_NAME}_loaded_plugin>\""
    "OPEN_APERO_TEST_DUPLICATE_PLUGIN_LIBRARY=\"$<TARGET_FILE:test_${PROJECT_NAME}_duplicate_plugin>\"")

  # --- Plugin conformance and contract tests (own binary: iterates the plugin registry) ---
  catkin_add_gtest(test_${PROJECT_NAME}_plugins
    test/test_main.cpp
    test/test_plugin_conformance.cpp
    test/test_plugin_behavior.cpp
    test/test_param_tracking.cpp
    test/test_graph_tsp_search.cpp
    test/test_plugin_preconditions.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}_plugins ${PROJECT_NAME} ${catkin_LIBRARIES})
  target_compile_options(test_${PROJECT_NAME}_plugins PRIVATE ${OPEN_APERO_CORE_WARNING_FLAGS})
  # --- end plugin tests ---
endif()
