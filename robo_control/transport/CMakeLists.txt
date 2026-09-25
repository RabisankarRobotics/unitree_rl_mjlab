cmake_minimum_required(VERSION 3.10)
project(transport)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robo_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(common REQUIRED)

# Transport is now a header-only library
add_library(transport INTERFACE)

# Set include directories
target_include_directories(transport INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link ROS2 dependencies
ament_target_dependencies(transport INTERFACE
    rclcpp
    robo_msgs
    common
)

target_link_libraries(transport INTERFACE
    yaml-cpp::yaml-cpp
)


# Export dependencies
ament_export_dependencies(rclcpp robo_msgs)
ament_export_include_directories(include)

# Install headers
install(DIRECTORY include/
    DESTINATION include
)

ament_package()
