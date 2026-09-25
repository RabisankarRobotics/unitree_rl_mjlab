cmake_minimum_required(VERSION 3.10)
project(imu_sdk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

add_library(imu_sdk SHARED
    src/bno08x_imu.cpp
)

target_include_directories(imu_sdk PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(imu_sdk PRIVATE Threads::Threads)

# Export
ament_export_include_directories(include)
ament_export_libraries(imu_sdk)

# Install
install(TARGETS imu_sdk
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

add_subdirectory(examples)

ament_package()
