cmake_minimum_required(VERSION 3.10)
project(joystick_sdk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

add_library(joystick_sdk SHARED
    src/joystick.cpp
)

target_include_directories(joystick_sdk PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(joystick_sdk PRIVATE Threads::Threads)

# Export
ament_export_include_directories(include)
ament_export_libraries(joystick_sdk)

# Install
install(TARGETS joystick_sdk
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
