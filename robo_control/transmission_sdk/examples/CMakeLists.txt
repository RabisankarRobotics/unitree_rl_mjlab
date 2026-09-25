add_executable(ankle_roundtrip
    ankle_roundtrip.cpp
)
target_link_libraries(ankle_roundtrip
    PRIVATE
        transmission_sdk
)

add_executable(ankle_benchmark
    ankle_benchmark.cpp
)
target_link_libraries(ankle_benchmark
    PRIVATE
        transmission_sdk
)

# Ankle Control TUI - drives two motors via ethercat_sdk to validate the
# Tahiti C1 parallel ankle transmission on real hardware.
find_package(ethercat_sdk REQUIRED)
add_executable(ankle_control
    ankle_control.cpp
)
target_link_libraries(ankle_control
    transmission_sdk
)
ament_target_dependencies(ankle_control ethercat_sdk)

install(TARGETS ankle_roundtrip ankle_benchmark ankle_control
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
