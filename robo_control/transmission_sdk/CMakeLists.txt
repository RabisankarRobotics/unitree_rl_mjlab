cmake_minimum_required(VERSION 3.10)
project(transmission_sdk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(transmission_sdk INTERFACE)

target_include_directories(transmission_sdk INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(transmission_sdk INTERFACE
    Eigen3::Eigen
)

ament_export_include_directories(include)
ament_export_dependencies(Eigen3)

install(DIRECTORY include/
    DESTINATION include
)

add_subdirectory(examples)

ament_package()
