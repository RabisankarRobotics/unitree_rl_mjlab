cmake_minimum_required(VERSION 3.16)
project(media_service)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robo_msgs REQUIRED)
find_package(spdlog REQUIRED)
find_package(common REQUIRED)
find_package(transport REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET gstreamer-1.0)
pkg_check_modules(GST_VIDEO IMPORTED_TARGET gstreamer-video-1.0)
pkg_check_modules(GST_AUDIO IMPORTED_TARGET gstreamer-audio-1.0)
pkg_check_modules(GIO REQUIRED IMPORTED_TARGET gio-unix-2.0)

find_package(peel REQUIRED)
peel_generate(Gst 1.0 RECURSIVE)
peel_generate(Gio 2.0)

# Work around int/pointer mismatches in peel-generated headers
add_compile_options(-fpermissive)

add_library(media_service_lib SHARED
    src/media_service.cpp
    src/gst_pipeline_manager.cpp
    src/signalling_server.cpp
)

target_include_directories(media_service_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(media_service_lib
    spdlog::spdlog_header_only
    yaml-cpp::yaml-cpp
    PkgConfig::GST
    PkgConfig::GIO
    peel::peel
    peel::Gst
    peel::Gio
)

ament_target_dependencies(media_service_lib
    rclcpp
    robo_msgs
    common
    transport
)

add_executable(media_service src/main.cpp)
target_link_libraries(media_service media_service_lib)
ament_target_dependencies(media_service rclcpp)

install(TARGETS media_service_lib LIBRARY DESTINATION lib)
install(TARGETS media_service DESTINATION lib/${PROJECT_NAME})

ament_package()
