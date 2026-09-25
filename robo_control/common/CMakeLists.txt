cmake_minimum_required(VERSION 3.10)
project(common)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(spdlog REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

# Create interface library (header-only)
add_library(common INTERFACE)

# Link dependencies
target_link_libraries(common INTERFACE
    spdlog::spdlog_header_only
    Threads::Threads
    yaml-cpp::yaml-cpp
)



# Set include directories
target_include_directories(common INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Export
ament_export_include_directories(include)
ament_export_dependencies(spdlog yaml-cpp)

# Install headers
install(DIRECTORY include/
    DESTINATION include
)

ament_package()
