cmake_minimum_required(VERSION 3.10)
project(service_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robo_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(common REQUIRED)
find_package(spdlog REQUIRED)


add_executable(service_manager
  src/service_manager.cpp
)

ament_target_dependencies(service_manager
  rclcpp
  robo_msgs
  common
)

target_link_libraries(service_manager
  spdlog::spdlog_header_only
  yaml-cpp::yaml-cpp
)




install(TARGETS service_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
