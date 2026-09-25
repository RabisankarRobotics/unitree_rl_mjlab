# Power Distribution Board Example
add_executable(power_dist_example
    power_dist_example.cpp
)
target_link_libraries(power_dist_example
    PRIVATE
        power_dist_board_sdk
)
