cmake_minimum_required(VERSION 3.10)
project(policy_service)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robo_msgs REQUIRED)
find_package(spdlog REQUIRED)
find_package(common REQUIRED)
find_package(transport REQUIRED)

# Find MNN inference library
find_path(MNN_INCLUDE_DIR MNN/Interpreter.hpp
    HINTS
        $ENV{HOME}/.local/include
        /usr/local/include
        /usr/include
)

find_library(MNN_LIBRARY NAMES MNN
    HINTS
        $ENV{HOME}/.local/lib
        /usr/local/lib
        /usr/lib
)

if(NOT MNN_INCLUDE_DIR OR NOT MNN_LIBRARY)
    message(FATAL_ERROR "MNN library not found. Please install MNN or set MNN_INCLUDE_DIR and MNN_LIBRARY.")
endif()

message(STATUS "Found MNN: ${MNN_LIBRARY}")
message(STATUS "MNN include: ${MNN_INCLUDE_DIR}")

# Source files
set(SOURCES
    src/mnn_policy.cpp
    src/policy_service.cpp
)

# Create library
add_library(policy_service_lib SHARED ${SOURCES})

# Target include directories
target_include_directories(policy_service_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${MNN_INCLUDE_DIR}
)

# Link dependencies
target_link_libraries(policy_service_lib
    spdlog::spdlog_header_only
    ${MNN_LIBRARY}
    yaml-cpp::yaml-cpp
)


ament_target_dependencies(policy_service_lib
    rclcpp
    robo_msgs
    common
    transport
)

# Main executable
add_executable(policy_service src/main.cpp)
target_link_libraries(policy_service
    policy_service_lib
)

ament_target_dependencies(policy_service
    rclcpp
)

# Install targets
install(TARGETS policy_service_lib
    LIBRARY DESTINATION lib
)

install(TARGETS policy_service
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
