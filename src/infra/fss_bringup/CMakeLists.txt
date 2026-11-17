cmake_minimum_required(VERSION 3.8)
project(fss_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch "${CMAKE_CURRENT_SOURCE_DIR}/../../../config"
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

ament_package()
