cmake_minimum_required(VERSION 3.10)
project(power_dist_board_sdk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

add_library(power_dist_board_sdk SHARED
    src/power_dist_board.cpp
)

target_include_directories(power_dist_board_sdk PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(power_dist_board_sdk PRIVATE Threads::Threads)

# Export
ament_export_include_directories(include)
ament_export_libraries(power_dist_board_sdk)

# Install
install(TARGETS power_dist_board_sdk
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

add_subdirectory(examples)

ament_package()
