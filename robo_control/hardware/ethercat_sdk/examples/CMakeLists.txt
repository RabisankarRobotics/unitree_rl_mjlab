# Simple Control Example
add_executable(simple_control
    simple_control.cpp
)
target_link_libraries(simple_control
    PRIVATE
        ethercat_sdk
)

# PVT Control Example
add_executable(pvt_control
    pvt_control.cpp
)
target_link_libraries(pvt_control
    PRIVATE
        ethercat_sdk
)

# Torque Control Example
add_executable(torque_control
    torque_control.cpp
)
target_link_libraries(torque_control
    PRIVATE
        ethercat_sdk
)

# PDO Mapping Inspector
add_executable(pdo_inspect
    pdo_inspect.cpp
)
target_link_libraries(pdo_inspect
    PRIVATE
        soem
)

# Read Positions CLI
add_executable(read_positions
    read_positions.cpp
)
target_link_libraries(read_positions
    PRIVATE
        ethercat_sdk
)

# Set Alias CLI
add_executable(set_alias
    set_alias.cpp
)
target_link_libraries(set_alias
    PRIVATE
        ethercat_sdk
)

# Joint Range Finder
add_executable(joint_range
    joint_range.cpp
)
target_link_libraries(joint_range
    PRIVATE
        ethercat_sdk
)

# Cycle Benchmark
add_executable(ethercat_cycle_benchmark
    cycle_benchmark.cpp
)
target_link_libraries(ethercat_cycle_benchmark
    PRIVATE
        ethercat_sdk
        yaml-cpp::yaml-cpp
)
ament_target_dependencies(ethercat_cycle_benchmark common)
