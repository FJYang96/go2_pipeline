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

add_executable(policy_runner src/policy_runner.cpp)
target_include_directories(policy_runner PRIVATE include)
ament_target_dependencies(policy_runner rclcpp)
target_link_libraries(policy_runner go2_control_core)
rosidl_target_interfaces(policy_runner ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(safety_supervisor src/safety_supervisor.cpp)
target_include_directories(safety_supervisor PRIVATE include)
ament_target_dependencies(safety_supervisor rclcpp std_msgs std_srvs unitree_go)
target_link_libraries(safety_supervisor go2_control_core)
rosidl_target_interfaces(safety_supervisor ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(keyboard_control src/keyboard_control.cpp)
ament_target_dependencies(keyboard_control rclcpp std_srvs)

install(TARGETS go2_control_core policy_runner safety_supervisor keyboard_control
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
  ament_add_pytest_test(test_config_schema test/test_config_schema.py)
endif()

ament_export_include_directories(include)
ament_export_libraries(go2_control_core)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
