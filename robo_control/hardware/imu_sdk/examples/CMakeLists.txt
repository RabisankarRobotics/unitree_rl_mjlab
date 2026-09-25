# BNO08x Example
add_executable(bno08x_example
    bno08x_example.cpp
)
target_link_libraries(bno08x_example
    PRIVATE
        imu_sdk
)
