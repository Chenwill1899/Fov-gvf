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

add_library(pc_gvf_depth_angular_core
  src/depth_angular_core.cpp
  src/dynamic_obstacles.cpp
  src/spherical_memory.cpp
  src/shared_obstacles.cpp
  src/depth_uncertainty.cpp
  src/paper_guidance.cpp
  src/paper_replay.cpp
  src/verified_start.cpp
  src/omni_depth.cpp
  src/local_depth_path.cpp
)
target_compile_features(pc_gvf_depth_angular_core PUBLIC cxx_std_14)
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

# The goal kernel is isolated at compile time: teleoperation retains the
# original core with no per-query cache branch or allocation. Both kernels use
# the same observations, certificates, budgets and serialized replay format.
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
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
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
  foreach(goal_check paper_guidance_check observed_space_check union_refinement_check
      history_repair_check six_axis_replay_check)
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
