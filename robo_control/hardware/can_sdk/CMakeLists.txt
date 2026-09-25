cmake_minimum_required(VERSION 3.10)
project(can_sdk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

add_library(can_sdk SHARED
    src/master.cpp
)

target_include_directories(can_sdk PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(can_sdk PRIVATE Threads::Threads)

# Export
ament_export_include_directories(include)
ament_export_libraries(can_sdk)

# Install
install(TARGETS can_sdk
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

# Examples (disabled for now)
# add_subdirectory(examples)

ament_package()
