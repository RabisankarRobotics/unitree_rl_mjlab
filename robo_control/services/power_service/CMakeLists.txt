cmake_minimum_required(VERSION 3.10)
project(power_service)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robo_msgs REQUIRED)
find_package(spdlog REQUIRED)
find_package(common REQUIRED)
find_package(transport REQUIRED)
find_package(power_dist_board_sdk REQUIRED)

set(SOURCES
    src/power_service.cpp
)

add_library(power_service_lib SHARED ${SOURCES})

target_include_directories(power_service_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(power_service_lib
    spdlog::spdlog_header_only
    yaml-cpp::yaml-cpp
)

ament_target_dependencies(power_service_lib
    rclcpp
    robo_msgs
    common
    transport
    power_dist_board_sdk
)

add_executable(power_service src/main.cpp)
target_link_libraries(power_service
    power_service_lib
)

ament_target_dependencies(power_service
    rclcpp
)

install(TARGETS power_service_lib
    LIBRARY DESTINATION lib
)

install(TARGETS power_service
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
