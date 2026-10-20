cmake_minimum_required(VERSION 3.16)
project(fss_manager LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fss_interfaces REQUIRED)

add_library(manager_core src/mode_fsm.cpp)
target_compile_features(manager_core PUBLIC cxx_std_17)
target_include_directories(manager_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(manager_ros src/manager_node.cpp)
target_link_libraries(manager_ros PUBLIC manager_core)
ament_target_dependencies(manager_ros PUBLIC rclcpp fss_interfaces)

add_executable(manager_node src/main.cpp)
target_link_libraries(manager_node manager_ros)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(manager_core PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(manager_ros PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS manager_core manager_ros manager_node
  EXPORT export_fss_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  enable_testing()
  add_executable(cpp_manager_test ../../../tests/cpp_manager_test.cpp)
  target_link_libraries(cpp_manager_test manager_core)
  add_test(NAME cpp_manager_core COMMAND cpp_manager_test)
  add_executable(cpp_manager_ros_test ../../../tests/cpp_manager_ros_test.cpp)
  target_link_libraries(cpp_manager_ros_test manager_ros)
  add_test(NAME cpp_manager_ros COMMAND cpp_manager_ros_test)
  set_tests_properties(cpp_manager_ros PROPERTIES TIMEOUT 30)
endif()

ament_export_targets(export_fss_manager HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp fss_interfaces)
ament_package()
