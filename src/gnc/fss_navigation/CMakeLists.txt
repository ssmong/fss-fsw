cmake_minimum_required(VERSION 3.14)
project(fss_navigation)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(fss_interfaces REQUIRED)

add_library(navigation_core src/ekf.cpp src/navigation.cpp)
target_compile_features(navigation_core PUBLIC cxx_std_17)
target_include_directories(navigation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(navigation_core PUBLIC Eigen3::Eigen)

add_executable(ekf_node src/ekf_node.cpp)
target_link_libraries(ekf_node navigation_core)
ament_target_dependencies(ekf_node rclcpp sensor_msgs fss_interfaces)

install(TARGETS navigation_core EXPORT navigation_core_targets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS ekf_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(navigation_core_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_gtest(cpp_navigation_test test/test_navigation.cpp)
  target_link_libraries(cpp_navigation_test navigation_core)
  add_executable(cpp_navigation_parity_runner test/parity_runner.cpp)
  target_link_libraries(cpp_navigation_parity_runner navigation_core)
  ament_add_test(cpp_navigation_golden
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_golden.py
      $<TARGET_FILE:cpp_navigation_parity_runner>)
  ament_add_test(cpp_navigation_ros_node
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ros_node.py
      $<TARGET_FILE:ekf_node>
    TIMEOUT 45)
endif()

ament_package()
