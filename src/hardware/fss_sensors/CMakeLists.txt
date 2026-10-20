cmake_minimum_required(VERSION 3.16)
project(fss_sensors)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(fss_interfaces REQUIRED)
find_package(fss_hal REQUIRED)

foreach(node vicon_bridge_node imu_bridge_node)
  add_executable(${node} src/${node}.cpp)
  target_compile_features(${node} PRIVATE cxx_std_17)
  target_compile_options(${node} PRIVATE -Wall -Wextra -Wpedantic)
  ament_target_dependencies(${node} rclcpp sensor_msgs fss_interfaces fss_hal)
  target_link_libraries(${node} fss_hal::fss_hal_cpp)
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

# Python calibration and research imports remain available without Python runtime entrypoints.
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_executable(cpp_sensors_test ${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/cpp_sensors.cpp)
  target_compile_features(cpp_sensors_test PRIVATE cxx_std_17)
  target_compile_options(cpp_sensors_test PRIVATE -Wall -Wextra -Wpedantic)
  ament_target_dependencies(cpp_sensors_test fss_hal)
  target_link_libraries(cpp_sensors_test fss_hal::fss_hal_cpp)
  add_test(NAME cpp_sensors_test COMMAND cpp_sensors_test)
  add_executable(cpp_sensors_udp_test
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/cpp_sensors_udp.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../fss_hal/src/vicon_hal.cpp)
  target_compile_features(cpp_sensors_udp_test PRIVATE cxx_std_17)
  target_compile_options(cpp_sensors_udp_test PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(cpp_sensors_udp_test PRIVATE ../fss_hal/include)
  target_link_libraries(cpp_sensors_udp_test PRIVATE nlohmann_json::nlohmann_json)
  target_link_options(cpp_sensors_udp_test PRIVATE -Wl,--wrap=recvfrom)
  add_test(NAME cpp_sensors_udp_test COMMAND cpp_sensors_udp_test)
  add_test(NAME cpp_sensors_ros_test COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/cpp_sensors_ros.py
    $<TARGET_FILE:imu_bridge_node> $<TARGET_FILE:vicon_bridge_node>)
  set_tests_properties(cpp_sensors_ros_test PROPERTIES TIMEOUT 45)
endif()

ament_package()
