cmake_minimum_required(VERSION 3.8)
project(pc_gvf)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(OpenMP QUIET)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pc_gvf_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Keep IEEE floating-point rules; optimize across static-core boundaries.
option(PC_GVF_ENABLE_IPO "Enable compiler-supported interprocedural optimization" ON)
if(PC_GVF_ENABLE_IPO AND POLICY CMP0069)
  cmake_policy(SET CMP0069 NEW)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT PC_GVF_IPO_SUPPORTED OUTPUT PC_GVF_IPO_DIAGNOSTIC LANGUAGES CXX)
  if(PC_GVF_IPO_SUPPORTED)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
  else()
    message(WARNING "IPO unavailable: ${PC_GVF_IPO_DIAGNOSTIC}")
  endif()
endif()

add_library(pc_gvf_depth_angular_core
  src/depth_angular_core.cpp
  src/dynamic_obstacles.cpp
  src/spherical_memory.cpp
  src/shared_obstacles.cpp
  src/depth_uncertainty.cpp
  src/paper_guidance.cpp
  src/active_navigation.cpp
  src/paper_replay.cpp
  src/verified_start.cpp
  src/omni_depth.cpp
  src/local_depth_path.cpp
)
target_compile_features(pc_gvf_depth_angular_core PUBLIC cxx_std_14)
# RTX 5090 manual-policy performance candidate: reuse only completely proved
# static geometry within the current observation/step, as in the goal kernel.
target_compile_definitions(pc_gvf_depth_angular_core PRIVATE PC_GVF_GOAL_TUBE_REUSE=1)
target_compile_options(pc_gvf_depth_angular_core PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(pc_gvf_depth_angular_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(pc_gvf_depth_angular_core PUBLIC Eigen3::Eigen OpenSSL::Crypto)
if(OpenMP_CXX_FOUND)
  target_link_libraries(pc_gvf_depth_angular_core PUBLIC OpenMP::OpenMP_CXX)
endif()

add_executable(paper_replay tools/paper_replay.cpp)
target_compile_features(paper_replay PRIVATE cxx_std_14)
target_link_libraries(paper_replay PRIVATE pc_gvf_depth_angular_core)

add_executable(depth_angular_controller src/depth_angular_controller_node.cpp)
target_compile_features(depth_angular_controller PRIVATE cxx_std_14)
target_compile_options(depth_angular_controller PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(depth_angular_controller pc_gvf_depth_angular_core)
ament_target_dependencies(depth_angular_controller
  geometry_msgs
  nav_msgs
  pc_gvf_msgs
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

# Retain separate historical binary names. The RTX 5090 candidate enables the
# same conservative frame-local geometric reuse for both kernels; nomination
# policy remains controlled by use_fixed_goal, independently of binary name.
# Observations, motion certificates, budgets and replay format are unchanged.
get_target_property(PC_GVF_CORE_SOURCES pc_gvf_depth_angular_core SOURCES)
add_library(pc_gvf_depth_angular_core_goal ${PC_GVF_CORE_SOURCES})
target_compile_features(pc_gvf_depth_angular_core_goal PUBLIC cxx_std_14)
target_compile_options(pc_gvf_depth_angular_core_goal PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(pc_gvf_depth_angular_core_goal PRIVATE PC_GVF_GOAL_TUBE_REUSE=1)
target_include_directories(pc_gvf_depth_angular_core_goal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(pc_gvf_depth_angular_core_goal PUBLIC Eigen3::Eigen OpenSSL::Crypto)
if(OpenMP_CXX_FOUND)
  target_link_libraries(pc_gvf_depth_angular_core_goal PUBLIC OpenMP::OpenMP_CXX)
endif()
add_executable(paper_replay_goal tools/paper_replay.cpp)
target_compile_features(paper_replay_goal PRIVATE cxx_std_14)
target_link_libraries(paper_replay_goal PRIVATE pc_gvf_depth_angular_core_goal)
add_executable(depth_angular_controller_goal src/depth_angular_controller_node.cpp)
target_compile_features(depth_angular_controller_goal PRIVATE cxx_std_14)
target_compile_options(depth_angular_controller_goal PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(depth_angular_controller_goal pc_gvf_depth_angular_core_goal)
ament_target_dependencies(depth_angular_controller_goal
  geometry_msgs nav_msgs pc_gvf_msgs rclcpp sensor_msgs std_msgs visualization_msgs)

# Keep the Python simulator and numerical oracle as acceptance infrastructure.
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/depth_angular_demo_sim
  scripts/depth_angular_core
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
    pc_gvf_depth_angular_core
    depth_angular_controller
    paper_replay
    pc_gvf_depth_angular_core_goal
    depth_angular_controller_goal
    paper_replay_goal
  EXPORT pc_gvfDepthAngularTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(timing_series_check test/cpp/timing_series_check.cpp)
  target_link_libraries(timing_series_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME timing_series_check COMMAND timing_series_check)

  foreach(CAMERA_SUFFIX IN ITEMS "" "_goal")
    add_executable(camera_evidence_cache_check${CAMERA_SUFFIX} test/cpp/camera_evidence_cache_check.cpp)
    target_link_libraries(camera_evidence_cache_check${CAMERA_SUFFIX} PRIVATE pc_gvf_depth_angular_core${CAMERA_SUFFIX})
    add_test(NAME camera_evidence_cache_check${CAMERA_SUFFIX} COMMAND camera_evidence_cache_check${CAMERA_SUFFIX})
  endforeach()

  foreach(MEMO_SUFFIX IN ITEMS "" "_goal")
    add_executable(proof_query_memo_check${MEMO_SUFFIX} test/cpp/proof_query_memo_check.cpp)
    target_link_libraries(proof_query_memo_check${MEMO_SUFFIX} PRIVATE pc_gvf_depth_angular_core${MEMO_SUFFIX})
    add_test(NAME proof_query_memo_check${MEMO_SUFFIX} COMMAND proof_query_memo_check${MEMO_SUFFIX})
  endforeach()
  foreach(TRANSACTION_SUFFIX IN ITEMS "" "_goal")
    foreach(TRANSACTION_CHECK published_command_search_check exhausted_budget_check)
      add_executable(${TRANSACTION_CHECK}${TRANSACTION_SUFFIX} test/cpp/${TRANSACTION_CHECK}.cpp)
      target_link_libraries(${TRANSACTION_CHECK}${TRANSACTION_SUFFIX} PRIVATE pc_gvf_depth_angular_core${TRANSACTION_SUFFIX})
      add_test(NAME ${TRANSACTION_CHECK}${TRANSACTION_SUFFIX} COMMAND ${TRANSACTION_CHECK}${TRANSACTION_SUFFIX})
    endforeach()
  endforeach()
  foreach(BOUNDS_SUFFIX IN ITEMS "" "_goal")
    add_executable(working_bounds_check${BOUNDS_SUFFIX} test/cpp/working_bounds_check.cpp)
    target_link_libraries(working_bounds_check${BOUNDS_SUFFIX} PRIVATE pc_gvf_depth_angular_core${BOUNDS_SUFFIX})
    add_test(NAME working_bounds_check${BOUNDS_SUFFIX} COMMAND working_bounds_check${BOUNDS_SUFFIX})
  endforeach()
  foreach(COMFORT_SUFFIX IN ITEMS "" "_goal")
    add_executable(recovery_comfort_check${COMFORT_SUFFIX} test/cpp/recovery_comfort_check.cpp)
    target_link_libraries(recovery_comfort_check${COMFORT_SUFFIX} PRIVATE pc_gvf_depth_angular_core${COMFORT_SUFFIX})
    add_test(NAME recovery_comfort_check${COMFORT_SUFFIX} COMMAND recovery_comfort_check${COMFORT_SUFFIX})
  endforeach()
  foreach(RECOVERY_SUFFIX IN ITEMS "" "_goal")
    add_executable(certified_recovery_check${RECOVERY_SUFFIX} test/cpp/certified_recovery_check.cpp)
    target_link_libraries(certified_recovery_check${RECOVERY_SUFFIX} PRIVATE pc_gvf_depth_angular_core${RECOVERY_SUFFIX})
    add_test(NAME certified_recovery_check${RECOVERY_SUFFIX} COMMAND certified_recovery_check${RECOVERY_SUFFIX})
  endforeach()

  add_executable(navigation_progress_check test/cpp/navigation_progress_check.cpp)
  target_link_libraries(navigation_progress_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME navigation_progress_check COMMAND navigation_progress_check)
  foreach(CEILING_SUFFIX IN ITEMS "" "_goal")
    add_executable(free_space_ceiling_check${CEILING_SUFFIX} test/cpp/free_space_ceiling_check.cpp)
    target_link_libraries(free_space_ceiling_check${CEILING_SUFFIX} PRIVATE pc_gvf_depth_angular_core${CEILING_SUFFIX})
    add_test(NAME free_space_ceiling_check${CEILING_SUFFIX} COMMAND free_space_ceiling_check${CEILING_SUFFIX})
  endforeach()

  add_executable(native_history_recovery_check test/cpp/native_history_recovery_check.cpp)
  target_link_libraries(native_history_recovery_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME native_history_recovery_check COMMAND native_history_recovery_check)
  add_executable(native_history_recovery_check_goal test/cpp/native_history_recovery_check.cpp)
  target_link_libraries(native_history_recovery_check_goal PRIVATE pc_gvf_depth_angular_core_goal)
  add_test(NAME native_history_recovery_check_goal COMMAND native_history_recovery_check_goal)
  find_package(ament_cmake_pytest REQUIRED)
  add_executable(six_axis_replay_check test/cpp/six_axis_replay_check.cpp)
  target_link_libraries(six_axis_replay_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME six_axis_replay_check COMMAND six_axis_replay_check)
  add_executable(shared_obstacles_check test/cpp/shared_obstacles_check.cpp)
  target_link_libraries(shared_obstacles_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME shared_obstacles_check COMMAND shared_obstacles_check)
  add_executable(operator_intent_check test/cpp/operator_intent_check.cpp)
  target_link_libraries(operator_intent_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME operator_intent_check COMMAND operator_intent_check)
  add_executable(spherical_ingest_check test/cpp/spherical_ingest_check.cpp)
  target_link_libraries(spherical_ingest_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME spherical_ingest_check COMMAND spherical_ingest_check)
  add_executable(spherical_memory_check test/cpp/spherical_memory_check.cpp)
  target_link_libraries(spherical_memory_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME spherical_memory_check COMMAND spherical_memory_check)
  add_executable(incremental_field_check test/cpp/incremental_field_check.cpp)
  target_link_libraries(incremental_field_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME incremental_field_check COMMAND incremental_field_check)
  add_executable(dynamic_closed_loop_check test/cpp/dynamic_closed_loop_check.cpp)
  target_link_libraries(dynamic_closed_loop_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME dynamic_closed_loop_check COMMAND dynamic_closed_loop_check)
  add_executable(dynamic_obstacles_check test/cpp/dynamic_obstacles_check.cpp)
  target_link_libraries(dynamic_obstacles_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME dynamic_obstacles_check COMMAND dynamic_obstacles_check)
  add_executable(depth_uncertainty_check test/cpp/depth_uncertainty_check.cpp)
  target_link_libraries(depth_uncertainty_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME depth_uncertainty_check COMMAND depth_uncertainty_check)

  add_executable(omni_depth_check test/cpp/omni_depth_check.cpp)
  target_link_libraries(omni_depth_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME omni_depth_check COMMAND omni_depth_check)

  add_executable(union_refinement_check test/cpp/union_refinement_check.cpp)
  target_link_libraries(union_refinement_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME union_refinement_check COMMAND union_refinement_check)

  add_executable(history_repair_check test/cpp/history_repair_check.cpp)
  target_link_libraries(history_repair_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME history_repair_check COMMAND history_repair_check)

  add_executable(clearance_check test/cpp/clearance_check.cpp)
  target_link_libraries(clearance_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME clearance_check COMMAND clearance_check)

  add_executable(local_depth_path_check test/cpp/local_depth_path_check.cpp)
  target_link_libraries(local_depth_path_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME local_depth_path_check COMMAND local_depth_path_check)

  add_executable(camera_geometry_check test/cpp/camera_geometry_check.cpp)
  target_compile_features(camera_geometry_check PRIVATE cxx_std_14)
  target_compile_options(camera_geometry_check PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(camera_geometry_check PRIVATE pc_gvf_depth_angular_core)
  target_compile_definitions(camera_geometry_check PRIVATE
    PC_GVF_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/python_behavior_v1")
  add_test(NAME camera_geometry_check COMMAND camera_geometry_check)

  add_executable(depth_geometry_check test/cpp/depth_geometry_check.cpp)
  target_compile_features(depth_geometry_check PRIVATE cxx_std_14)
  target_compile_options(depth_geometry_check PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(depth_geometry_check PRIVATE pc_gvf_depth_angular_core)
  target_compile_definitions(depth_geometry_check PRIVATE
    PC_GVF_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/python_behavior_v1")
  add_test(NAME depth_geometry_check COMMAND depth_geometry_check)

  add_executable(angular_field_check test/cpp/angular_field_check.cpp)
  target_compile_features(angular_field_check PRIVATE cxx_std_14)
  target_compile_options(angular_field_check PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(angular_field_check PRIVATE pc_gvf_depth_angular_core)
  target_compile_definitions(angular_field_check PRIVATE
    PC_GVF_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/python_behavior_v1")
  add_test(NAME angular_field_check COMMAND angular_field_check)

  add_executable(motion_safety_check test/cpp/motion_safety_check.cpp)
  target_compile_features(motion_safety_check PRIVATE cxx_std_14)
  target_compile_options(motion_safety_check PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(motion_safety_check PRIVATE pc_gvf_depth_angular_core)
  target_compile_definitions(motion_safety_check PRIVATE
    PC_GVF_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/python_behavior_v1")
  add_test(NAME motion_safety_check COMMAND motion_safety_check)

  add_executable(guidance_composition_check
    test/cpp/guidance_composition_check.cpp)
  target_compile_features(guidance_composition_check PRIVATE cxx_std_14)
  target_compile_options(guidance_composition_check PRIVATE
    -Wall -Wextra -Wpedantic)
  target_link_libraries(guidance_composition_check PRIVATE
    pc_gvf_depth_angular_core)
  target_compile_definitions(guidance_composition_check PRIVATE
    PC_GVF_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/python_behavior_v1")
  add_test(NAME guidance_composition_check COMMAND guidance_composition_check)

  add_executable(horizontal_360_check test/cpp/horizontal_360_check.cpp)
  target_compile_features(horizontal_360_check PRIVATE cxx_std_14)
  target_compile_options(horizontal_360_check PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(horizontal_360_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME horizontal_360_check COMMAND horizontal_360_check)

  add_executable(paper_guidance_check test/cpp/paper_guidance_check.cpp)
  target_link_libraries(paper_guidance_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME paper_guidance_check COMMAND paper_guidance_check)

  add_executable(observed_space_check test/cpp/observed_space_check.cpp)
  target_link_libraries(observed_space_check PRIVATE pc_gvf_depth_angular_core)
  target_compile_definitions(observed_space_check PRIVATE PC_GVF_PROJECT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../..")
  add_test(NAME observed_space_check COMMAND observed_space_check)

  add_executable(continuous_recovery_check test/cpp/continuous_recovery_check.cpp)
  target_link_libraries(continuous_recovery_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME continuous_recovery_check COMMAND continuous_recovery_check)

  add_executable(free_space_floor_check test/cpp/free_space_floor_check.cpp)
  target_link_libraries(free_space_floor_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME free_space_floor_check COMMAND free_space_floor_check)

  add_executable(motion_consistency_check test/cpp/motion_consistency_check.cpp)
  target_link_libraries(motion_consistency_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME motion_consistency_check COMMAND motion_consistency_check)

  add_executable(intent_response_check test/cpp/intent_response_check.cpp)
  target_link_libraries(intent_response_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME intent_response_check COMMAND intent_response_check)

  add_executable(paper_closed_loop_check test/cpp/paper_closed_loop_check.cpp)
  target_link_libraries(paper_closed_loop_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME paper_closed_loop_check COMMAND paper_closed_loop_check)

  # Run the unchanged safety/negative-case fixtures against the goal kernel too.
  add_executable(active_navigation_check test/cpp/active_navigation_check.cpp)
  target_link_libraries(active_navigation_check PRIVATE pc_gvf_depth_angular_core)
  add_test(NAME active_navigation_check COMMAND active_navigation_check)

  foreach(goal_check paper_guidance_check observed_space_check union_refinement_check
      history_repair_check six_axis_replay_check continuous_recovery_check free_space_floor_check active_navigation_check)
    add_executable(${goal_check}_goal test/cpp/${goal_check}.cpp)
    target_link_libraries(${goal_check}_goal PRIVATE pc_gvf_depth_angular_core_goal)
    target_compile_definitions(${goal_check}_goal PRIVATE
      PC_GVF_PROJECT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../..")
    add_test(NAME ${goal_check}_goal COMMAND ${goal_check}_goal)
  endforeach()

  foreach(test_name
      test_depth_angular_core
      test_python_behavior_baseline
      test_manual_control_math
      test_navigation_benchmark
      test_long_runtime
      test_navigation_ground_guard
      test_beitong_joystick)
    ament_add_pytest_test(${test_name} test/${test_name}.py
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  endforeach()
endif()

ament_export_targets(pc_gvfDepthAngularTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 OpenSSL)
if(OpenMP_CXX_FOUND)
  ament_export_dependencies(OpenMP)
endif()
ament_package()
