cmake_minimum_required(VERSION 3.16)
project(fss_hal)

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED IMPORTED_TARGET libmodbus)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(fss_hal_cpp SHARED src/plc_hal.cpp src/reaction_wheel.cpp src/vicon_hal.cpp)
target_compile_features(fss_hal_cpp PUBLIC cxx_std_17)
target_compile_options(fss_hal_cpp PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(fss_hal_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(fss_hal_cpp PUBLIC PkgConfig::MODBUS yaml-cpp nlohmann_json::nlohmann_json)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS fss_hal_cpp EXPORT fss_halTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(fss_halTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(PkgConfig yaml-cpp nlohmann_json)
# Downstream imported targets reference the system libmodbus pkg-config target.
ament_package(CONFIG_EXTRAS cmake/fss_hal-extras.cmake)
