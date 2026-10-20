cmake_minimum_required(VERSION 3.14)
project(fss_gnc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fss_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(fss_gnc_node src/gnc_node.cpp)
target_include_directories(fss_gnc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(fss_gnc_node rclcpp ament_index_cpp fss_interfaces)
target_link_libraries(fss_gnc_node yaml-cpp)
target_compile_options(fss_gnc_node PRIVATE -Wall -Wextra -Wpedantic)

add_executable(gnc_node src/main.cpp)
target_link_libraries(gnc_node fss_gnc_node)
ament_target_dependencies(gnc_node rclcpp)
install(TARGETS gnc_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS fss_gnc_node EXPORT fss_gncTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

# Research backend only; the C++ flight loop never imports Python or pybind11.
option(FSS_GNC_BUILD_PYTHON_BINDINGS "Build the optional Python PID/LQR/QP research bindings" OFF)
if(FSS_GNC_BUILD_PYTHON_BINDINGS)
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
  find_package(pybind11 REQUIRED)
  pybind11_add_module(_gnc_core src/gnc_bindings.cpp)
  target_include_directories(_gnc_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_get_python_install_dir(PYTHON_INSTALL_DIR)
  install(TARGETS _gnc_core LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/fss_gnc")
endif()
ament_python_install_package(fss_gnc)
install(PROGRAMS fss_gnc/gnc_node.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  ament_add_gtest(test_gnc_runtime test/test_gnc_runtime.cpp)
  target_include_directories(test_gnc_runtime PRIVATE include)
  target_link_libraries(test_gnc_runtime yaml-cpp)
  target_compile_definitions(test_gnc_runtime PRIVATE
    FSS_TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../config")
  ament_add_gtest(test_gnc_node test/test_gnc_node.cpp)
  target_link_libraries(test_gnc_node fss_gnc_node)
  ament_target_dependencies(test_gnc_node rclcpp fss_interfaces rosgraph_msgs)
  target_compile_definitions(test_gnc_node PRIVATE
    FSS_TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../config")
endif()

ament_export_targets(fss_gncTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp ament_index_cpp fss_interfaces yaml-cpp)
ament_package()
