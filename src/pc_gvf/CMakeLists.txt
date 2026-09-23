cmake_minimum_required(VERSION 3.8)
project(pc_gvf)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pc_gvf_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(pc_gvf_depth_angular_core
  src/depth_angular_core.cpp
)
target_compile_features(pc_gvf_depth_angular_core PUBLIC cxx_std_14)
target_compile_options(pc_gvf_depth_angular_core PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(pc_gvf_depth_angular_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(pc_gvf_depth_angular_core PUBLIC Eigen3::Eigen)

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

# Keep the Python simulator and numerical oracle as acceptance infrastructure.
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/depth_angular_demo_sim
  scripts/depth_angular_core
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
    pc_gvf_depth_angular_core
    depth_angular_controller
  EXPORT pc_gvfDepthAngularTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

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

  foreach(test_name
      test_depth_angular_core
      test_python_behavior_baseline
      test_manual_control_math
      test_beitong_joystick)
    ament_add_pytest_test(${test_name} test/${test_name}.py
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  endforeach()
endif()

ament_export_targets(pc_gvfDepthAngularTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
