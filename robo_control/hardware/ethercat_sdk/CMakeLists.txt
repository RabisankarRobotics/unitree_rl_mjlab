cmake_minimum_required(VERSION 3.10)
project(ethercat_sdk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(soem REQUIRED)
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)
find_package(common REQUIRED)

add_library(ethercat_sdk SHARED
    src/master.cpp
)

target_include_directories(ethercat_sdk PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(ethercat_sdk
    soem
    Threads::Threads
    spdlog::spdlog_header_only
)

ament_target_dependencies(ethercat_sdk common)

# Export
ament_export_include_directories(include)
ament_export_libraries(ethercat_sdk)
ament_export_dependencies(common)

# Install
install(TARGETS ethercat_sdk
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

add_subdirectory(examples)

ament_package()
