cmake_minimum_required(VERSION 3.10)
project(control_service)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robo_msgs REQUIRED)
find_package(spdlog REQUIRED)
find_package(common REQUIRED)
find_package(transport REQUIRED)
find_package(ethercat_sdk REQUIRED)
find_package(can_sdk REQUIRED)
find_package(transmission_sdk REQUIRED)
find_package(Eigen3 REQUIRED)

option(CONTROL_LOOP_BENCHMARK
    "Enable per-stage timing instrumentation in the control loop" OFF)

# Source files
set(SOURCES
    src/ethercat_backend.cpp
    src/can_backend.cpp
    src/robot_controller.cpp
)

# Create library
add_library(control_service_lib SHARED ${SOURCES})

# Target include directories
target_include_directories(control_service_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(CONTROL_LOOP_BENCHMARK)
    target_compile_definitions(control_service_lib
        PUBLIC CONTROL_LOOP_BENCHMARK)
endif()

# Link dependencies
target_link_libraries(control_service_lib
    spdlog::spdlog_header_only
    yaml-cpp::yaml-cpp
    Eigen3::Eigen
)


ament_target_dependencies(control_service_lib
    rclcpp
    robo_msgs
    common
    transport
    ethercat_sdk
    can_sdk
    transmission_sdk
)

# Main executable
add_executable(control_service src/main.cpp)
target_link_libraries(control_service
    control_service_lib
)

ament_target_dependencies(control_service
    rclcpp
)

# Install targets
install(TARGETS control_service_lib
    LIBRARY DESTINATION lib
)

install(TARGETS control_service
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
