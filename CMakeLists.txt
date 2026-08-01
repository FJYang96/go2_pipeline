cmake_minimum_required(VERSION 3.5)
project(go2_nn_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ZLIB REQUIRED)

if(NOT ONNXRUNTIME_ROOT AND DEFINED ENV{ONNXRUNTIME_ROOT})
  set(ONNXRUNTIME_ROOT $ENV{ONNXRUNTIME_ROOT})
endif()
if(NOT ONNXRUNTIME_ROOT)
  set(ONNXRUNTIME_ROOT /opt/onnxruntime)
endif()
find_path(ONNXRUNTIME_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  HINTS "${ONNXRUNTIME_ROOT}/include"
  REQUIRED)
find_library(ONNXRUNTIME_LIBRARY
  NAMES onnxruntime
  HINTS "${ONNXRUNTIME_ROOT}/lib"
  REQUIRED)
message(STATUS "ONNX Runtime include: ${ONNXRUNTIME_INCLUDE_DIR}")
message(STATUS "ONNX Runtime library: ${ONNXRUNTIME_LIBRARY}")

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PolicyObservation.msg"
  "msg/PolicyAction.msg"
  "msg/AppliedCommand.msg"
  "msg/SupervisorStatus.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

add_library(go2_control_core
  src/observation_wrappers.cpp
  src/unitree_crc.cpp
)
target_include_directories(go2_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(go2_control_core rclcpp unitree_go)

add_library(go2_policy_runtime
  src/policy/npy_array.cpp
  src/policy/policy_manifest.cpp
  src/policy/reference_trajectory.cpp
  src/policy/onnx_policy.cpp
  src/policy/residual_policy_runtime.cpp
  src/policy/parity_confirmation.cpp
)
target_include_directories(go2_policy_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ONNXRUNTIME_INCLUDE_DIR})
target_link_libraries(go2_policy_runtime
  PUBLIC go2_control_core
  PRIVATE yaml-cpp OpenSSL::Crypto ZLIB::ZLIB ${ONNXRUNTIME_LIBRARY})

add_executable(inspect_policy_package src/policy/inspect_policy_package_main.cpp)
target_link_libraries(inspect_policy_package go2_policy_runtime)

add_executable(policy_runner src/policy_runner.cpp)
target_include_directories(policy_runner PRIVATE include)
ament_target_dependencies(policy_runner rclcpp)
target_link_libraries(policy_runner go2_policy_runtime)
rosidl_target_interfaces(policy_runner ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(safety_supervisor src/safety_supervisor.cpp)
target_include_directories(safety_supervisor PRIVATE include)
ament_target_dependencies(safety_supervisor rclcpp std_msgs std_srvs unitree_go)
target_link_libraries(safety_supervisor go2_policy_runtime)
rosidl_target_interfaces(safety_supervisor ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(keyboard_control src/keyboard_control.cpp)
ament_target_dependencies(keyboard_control rclcpp std_srvs)

install(TARGETS
  go2_control_core
  go2_policy_runtime
  inspect_policy_package
  policy_runner
  safety_supervisor
  keyboard_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/run_risk_test.sh DESTINATION lib/${PROJECT_NAME})
install(FILES README.md TESTING.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_program(GO2_PYTHON3 NAMES python3 REQUIRED)

  set(GO2_POLICY_TEST_FIXTURES_DIR
      "${CMAKE_CURRENT_BINARY_DIR}/policy_test_fixtures")
  add_custom_command(
    OUTPUT
      "${GO2_POLICY_TEST_FIXTURES_DIR}/valid_package/MANIFEST.sha256"
      "${GO2_POLICY_TEST_FIXTURES_DIR}/valid_package_f32/MANIFEST.sha256"
    COMMAND "${GO2_PYTHON3}"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/scripts/generate_policy_fixtures.py"
            "--output" "${GO2_POLICY_TEST_FIXTURES_DIR}"
    DEPENDS
      "${CMAKE_CURRENT_SOURCE_DIR}/test/scripts/generate_policy_fixtures.py"
    COMMENT "Generating policy package test fixtures"
    VERBATIM)
  add_custom_target(go2_policy_test_fixtures DEPENDS
    "${GO2_POLICY_TEST_FIXTURES_DIR}/valid_package/MANIFEST.sha256"
    "${GO2_POLICY_TEST_FIXTURES_DIR}/valid_package_f32/MANIFEST.sha256")

  ament_add_gtest(test_observation_wrappers test/test_observation_wrappers.cpp)
  if(TARGET test_observation_wrappers)
    target_include_directories(test_observation_wrappers PRIVATE include)
    target_link_libraries(test_observation_wrappers go2_control_core)
  endif()
  ament_add_gtest(test_unitree_crc test/test_unitree_crc.cpp)
  if(TARGET test_unitree_crc)
    target_include_directories(test_unitree_crc PRIVATE include)
    ament_target_dependencies(test_unitree_crc unitree_go)
    target_link_libraries(test_unitree_crc go2_control_core)
  endif()
  ament_add_gtest(test_policy_manifest test/test_policy_manifest.cpp)
  if(TARGET test_policy_manifest)
    target_include_directories(test_policy_manifest PRIVATE include test)
    target_link_libraries(test_policy_manifest go2_policy_runtime OpenSSL::Crypto)
    target_compile_definitions(test_policy_manifest PRIVATE
      GO2_POLICY_TEST_FIXTURES_DIR="${GO2_POLICY_TEST_FIXTURES_DIR}")
    add_dependencies(test_policy_manifest go2_policy_test_fixtures)
  endif()
  ament_add_gtest(test_reference_trajectory test/test_reference_trajectory.cpp)
  if(TARGET test_reference_trajectory)
    target_include_directories(test_reference_trajectory PRIVATE include test)
    target_link_libraries(test_reference_trajectory go2_policy_runtime OpenSSL::Crypto)
    target_compile_definitions(test_reference_trajectory PRIVATE
      GO2_POLICY_TEST_FIXTURES_DIR="${GO2_POLICY_TEST_FIXTURES_DIR}")
    add_dependencies(test_reference_trajectory go2_policy_test_fixtures)
  endif()
  ament_add_gtest(test_residual_policy_runtime test/test_residual_policy_runtime.cpp)
  if(TARGET test_residual_policy_runtime)
    target_include_directories(test_residual_policy_runtime PRIVATE include test)
    target_link_libraries(test_residual_policy_runtime go2_policy_runtime)
    target_compile_definitions(test_residual_policy_runtime PRIVATE
      GO2_POLICY_TEST_FIXTURES_DIR="${GO2_POLICY_TEST_FIXTURES_DIR}"
      GO2_HOP_POLICY_DIR="/misc/hop_robustify_20260724_164647")
    add_dependencies(test_residual_policy_runtime go2_policy_test_fixtures)
  endif()
  ament_add_pytest_test(test_config_schema test/test_config_schema.py)
endif()

ament_export_include_directories(include)
ament_export_libraries(go2_control_core go2_policy_runtime)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
