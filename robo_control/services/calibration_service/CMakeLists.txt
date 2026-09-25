cmake_minimum_required(VERSION 3.10)
project(calibration_service)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robo_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(common REQUIRED)
find_package(transport REQUIRED)
find_package(ethercat_sdk REQUIRED)
find_package(spdlog REQUIRED)

add_executable(calibration_service
  src/calibration_service.cpp
  src/main.cpp
)

target_include_directories(calibration_service PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(calibration_service
  rclcpp
  robo_msgs
  common
  transport
  ethercat_sdk
)

target_link_libraries(calibration_service
  spdlog::spdlog_header_only
  yaml-cpp::yaml-cpp
)

install(TARGETS calibration_service
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
