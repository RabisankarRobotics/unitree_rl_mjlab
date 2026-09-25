# Simple Control Example
add_executable(can_simple_control
    simple_control.cpp
)
target_link_libraries(can_simple_control
    PRIVATE
        can_sdk
)

# Latency Benchmark
add_executable(can_latency_benchmark
    latency_benchmark.cpp
)
target_link_libraries(can_latency_benchmark
    PRIVATE
        can_sdk
)
